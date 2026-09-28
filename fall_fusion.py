"""Fusion fall-detection module for Raspberry Pi 4B elderly monitoring.

Standalone: ``import fall_fusion`` from ``main_pi.py`` (or run directly for self-test).
No hardware imports at module import time -- RPi.GPIO / smbus are lazily
imported inside the ``*_live`` helper functions only, so this file is importable
and unit-testable on any machine with stdlib only.

Data shapes reused from ``main_pi.py`` (do NOT modify that file):
  * radar targets: list of dicts ``{dist, x, y, speed}`` produced by
    ``ld2450_worker`` (UART2 ``/dev/ttyAMA2`` 256000). ``dist`` is metres
    (``sqrt(x*x+y*y)/1000``), ``x/y/speed`` are raw LD2450 units.
    Only targets with ``dist > 0`` are valid (see main loop ``valid_targets``).
  * MPU6050 raw accel: signed ints from ``read_raw_data(bus, addr)`` on bus 1,
    addr ``0x68``, registers ``ACCEL_XOUT_H (0x3B)`` / +2 / +4.
  * ``SharedState`` fields consumed: ``radar_targets``; produced: nothing
    (caller decides what to do with the returned event dict).

Fusion idea: an MPU impact spike FOLLOWED BY radar stillness within T seconds
= FALL. Either signal alone = SUSPECT observation only (never an alarm).
A stillness confirmation window plus an alarm cooldown cut false positives.

Typical wiring into ``main_pi.py`` (example, not executed here)::

    from fall_fusion import FallDetector
    det = FallDetector()
    # in the main loop, replacing/augmenting section 6:
    #   event = det.update(ax, ay, az, curr_targets)
    #   if event and event["type"] == "FALL":
    #       requests.post(FALL_API_URL, json=..., timeout=2)

Target interpreter: Python 3.13 on the Pi. Stdlib only.
"""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime, timezone

# Default alarm log. Compatible event dict: {time, type, detail}.
# ``main_pi.py`` currently POSTs fall alerts to a backend; this module instead
# appends a JSON line locally AND returns the event dict -- it never does
# network I/O itself (no requests/Flask), so the caller chooses how to forward.
DEFAULT_ALARM_FILE = r"D:\esp32-butler\alarms.jsonl"

# Button wiring (documentation contract):
# Physical button on BCM GPIO17 (see main_pi.py BUTTON_PIN = 17, PUD_UP).
# A press there is treated as an explicit help call -> immediate FALL alarm
# via button_pressed_to_event(). Wiring/monitoring of the pin itself stays in
# main_pi.py's sensor_loop; this module only builds the event dict.
BUTTON_PIN = 17


# ---------------------------------------------------------------------------
# Platform-layer config (rules.json)
# ---------------------------------------------------------------------------

def load_rules(path: str | None = None) -> dict:
    """Load fusion tunables from a JSON file (platform-layer config).

    Args:
        path: JSON file path. Defaults to ``rules.json`` next to this file
            (on the Pi that is ``~/food_project/rules.json``).

    Returns:
        The parsed dict, or ``{}`` silently when the file is missing,
        corrupt, or not a JSON object. Stdlib only, no I/O besides the read.
    """
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rules.json")
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def _rule_or_explicit(rules: dict, key: str, explicit: float, default: float) -> float:
    """Resolve one tunable: explicit constructor arg wins over rules.json.

    The rules value for ``key`` is used only when the caller left the
    constructor arg at its built-in ``default``; unknown keys are never
    consulted here, so they are ignored. Missing or non-numeric rules values
    fall back to ``default`` so a bad config can never crash the monitor.
    """
    if explicit != default:
        return float(explicit)
    try:
        return float(rules[key])
    except (KeyError, TypeError, ValueError):
        return float(default)


# ---------------------------------------------------------------------------
# Pure helpers (unit-testable, no I/O, no time dependency)
# ---------------------------------------------------------------------------

def accel_magnitude(ax: float, ay: float, az: float) -> float:
    """Magnitude of a raw MPU6050 accel sample.

    Args:
        ax, ay, az: raw signed ints as returned by ``read_raw_data``.

    Returns:
        Euclidean norm ``sqrt(ax^2 + ay^2 + az^2)`` in the same raw units.
        At rest this is ~1g ~= 16384 LSB (MPU6050 +/-2g full scale).
    """
    return math.sqrt(float(ax) * float(ax) + float(ay) * float(ay) + float(az) * float(az))


def is_spike(mag: float, baseline: float = 16384.0, spike_threshold: float = 25000.0) -> bool:
    """True when a magnitude sample counts as an impact spike.

    The comparison is absolute (``mag > spike_threshold``), matching the step
    detector style in ``main_pi.py`` (``curr_mag > step_threshold`` with
    ``step_threshold = 25000``). ``baseline`` documents the 1g rest level and
    is used by callers reasoning about "jump vs 1g"; the threshold itself is
    the operative gate.
    """
    return mag > spike_threshold


def pick_primary_target(targets) -> dict | None:
    """Return the first valid radar target, or None.

    Mirrors ``main_pi.py``: ``valid_targets = [t for t in curr_targets
    if t['dist'] > 0]`` and then ``valid_targets[0]``. A target with
    ``dist <= 0`` (or missing keys) means "no person tracked in this slot".
    """
    if not targets:
        return None
    for t in targets:
        try:
            if float(t.get("dist", 0)) > 0:
                return t
        except (Type/ValueError, AttributeError):
            continue
    return None


def target_speed(target: dict | None) -> float | None:
    """Extract ``|speed|`` from a target dict, or None if unusable."""
    if target is None:
        return None
    try:
        return abs(float(target.get("speed", 0.0)))
    except (TypeError, ValueError):
        return None


def utc_now_iso() -> str:
    """Current UTC time as ISO string (same style as main_pi.py temp upload)."""
    return datetime.now(timezone.utc).isoformat()


# ---------------------------------------------------------------------------
# FallDetector
# ---------------------------------------------------------------------------

class FallDetector:
    """MPU-spike + LD2450-stillness fusion fall detector.

    All tunables are ``__init__`` params (documented below). State is plain
    instance attributes; inject ``now`` (epoch seconds) in updates for tests.
    """

    def __init__(
        self,
        # -- MPU spike side ---------------------------------------------
        # accel_baseline: rest magnitude ~= 1g = 16384 LSB at +/-2g scale.
        #   Reference, not a gate; documents what "jump vs 1g" means.
        accel_baseline: float = 16384.0,
        # spike_threshold: magnitude above this = impact spike.
        #   Default 25000.0 mirrors main_pi.py sensor_loop step_threshold
        #   = 25000, so behaviour is consistent with the existing code's
        #   sense of "a vigorous motion event" in raw LSB units.
        spike_threshold: float = 25000.0,
        # spike_cooldown: seconds to ignore further spikes after one fires.
        #   Debounce so a single impact (ringing samples) counts once.
        #   Longer than the 0.3 s step debounce because falls need only one.
        spike_cooldown: float = 1.0,
        # -- radar stillness side ----------------------------------------
        # active_speed_threshold: |speed| at/above this = person moving.
        #   Default 100.0 mirrors main_pi.py FALL_SPEED_THRESHOLD = 100.0,
        #   reusing the codebase's existing notion of "significant motion".
        active_speed_threshold: float = 100.0,
        # still_speed_threshold: |speed| below this = (near-)motionless.
        #   Well under the active line so walk-vs-lying separates cleanly.
        still_speed_threshold: float = 50.0,
        # stillness_duration (N seconds): still speed must persist this long
        #   AFTER a prior active period to count as confirmed stillness.
        #   This is the N-second confirmation window that rejects momentary
        #   pauses and radar dropouts.
        stillness_duration: float = 3.0,
        # -- fusion / alarm ----------------------------------------------
        # fusion_window (T seconds): confirmed stillness must complete within
        #   this long AFTER the spike to fuse into FALL (spike THEN stillness).
        #   A lone spike that never leads to stillness stays SUSPECT.
        fusion_window: float = 10.0,
        # alarm_cooldown: minimum seconds between FALL alarms.
        #   Default 10.0 mirrors main_pi.py section 6 guard
        #   (current_time - last_fall_time >= 10.0).
        alarm_cooldown: float = 10.0,
        # alarm_file: JSON-lines log path; each FALL appends one line
        #   {time, type, detail}. Caller may pass an alternate path in tests.
        alarm_file: str = DEFAULT_ALARM_FILE,
        # rules: platform-layer config dict as returned by load_rules()
        #   (rules.json next to this file). Applied only for keys present;
        #   an explicitly passed constructor arg always wins over rules;
        #   unknown keys are ignored. None/{} keeps built-in defaults.
        rules: dict | None = None,
    ) -> None:
        rules = dict(rules) if rules else {}
        self.accel_baseline = float(accel_baseline)
        self.spike_threshold = _rule_or_explicit(rules, "spike_threshold", spike_threshold, 25000.0)
        self.spike_cooldown = float(spike_cooldown)
        self.active_speed_threshold = _rule_or_explicit(rules, "active_speed_threshold", active_speed_threshold, 100.0)
        self.still_speed_threshold = _rule_or_explicit(rules, "still_speed_threshold", still_speed_threshold, 50.0)
        self.stillness_duration = _rule_or_explicit(rules, "stillness_duration", stillness_duration, 3.0)
        self.fusion_window = _rule_or_explicit(rules, "fusion_window", fusion_window, 10.0)
        self.alarm_cooldown = _rule_or_explicit(rules, "alarm_cooldown", alarm_cooldown, 10.0)
        self.alarm_file = alarm_file

        self._last_spike_time: float | None = None   # last accepted spike
        self._last_alarm_time: float = 0.0           # epoch; 0 => first alarm free
        self._had_active: bool = False               # seen motion before stillness
        self._still_since: float | None = None       # start of current still run
        self._last_speed: float | None = None

    # -- introspection ----------------------------------------------------
    @property
    def last_spike_time(self) -> float | None:
        return self._last_spike_time

    @property
    def still_since(self) -> float | None:
        return self._still_since

    def stillness_elapsed(self, now: float) -> float | None:
        """Seconds of continuous stillness up to ``now``, or None."""
        if self._still_since is None:
            return None
        return max(0.0, float(now) - self._still_since)

    def reset(self) -> None:
        """Clear fusion state (keeps alarm cooldown + config)."""
        self._last_spike_time = None
        self._had_active = False
        self._still_since = None
        self._last_speed = None

    # -- per-sensor updates ------------------------------------------------
    def update_mpu(self, ax: float, ay: float, az: float, now: float | None = None) -> bool:
        """Feed one raw MPU sample. Returns True on a fresh accepted spike.

        A spike is ``accel_magnitude(ax,ay,az) > spike_threshold`` and outside
        ``spike_cooldown`` of the previous accepted spike.
        """
        t = time.time() if now is None else float(now)
        mag = accel_magnitude(ax, ay, az)
        if mag <= self.spike_threshold:
            return False
        if self._last_spike_time is not None and (t - self._last_spike_time) < self.spike_cooldown:
            return False
        self._last_spike_time = t
        return True

    def update_radar(self, targets, now: float | None = None) -> str:
        """Feed one radar frame. Returns ``"ACTIVE"`` / ``"STILL"`` / ``"ABSENT"``.

        * ``ABSENT``: no valid target (all ``dist <= 0``) -- stillness run reset.
        * ``ACTIVE``: ``|speed| >= still_speed_threshold`` -- marks ``_had_active``,
          resets any still run (motion breaks stillness).
        * ``STILL``: valid target with ``|speed| < still_speed_threshold`` --
          starts/continues the still run. Confirmation (elapsed >=
          ``stillness_duration`` with prior ``_had_active``) is checked in
          :meth:`stillness_confirmed`.
        """
        t = time.time() if now is None else float(now)
        tgt = pick_primary_target(targets)
        spd = target_speed(tgt)
        self._last_speed = spd
        if spd is None:
            self._still_since = None
            return "ABSENT"
        if spd >= self.still_speed_threshold:
            if spd >= self.active_speed_threshold:
                self._had_active = True
            self._still_since = None
            return "ACTIVE"
        if self._still_since is None:
            self._still_since = t
        return "STILL"

    def stillness_confirmed(self, now: float | None = None) -> bool:
        """True once stillness has persisted ``stillness_duration`` after activity."""
        t = time.time() if now is None else float(now)
        if not self._had_active or self._still_since is None:
            return False
        return (t - self._still_since) >= self.stillness_duration

    # -- fusion -------------------------------------------------------------
    def check_fusion(self, now: float | None = None) -> dict | None:
        """Evaluate the fusion rule; emit FALL (file + return) or SUSPECT (return).

        * spike recorded AND stillness just confirmed within ``fusion_window``
          after the spike -> FALL alarm (``_emit_fall``, cooldown-guarded).
        * spike recorded but stillness not (yet/anymore) confirmed -> SUSPECT
          observation dict (NOT written to the alarm file, NOT cooldown-bound).
        * stillness confirmed with no recent spike -> SUSPECT dict likewise.
        * neither -> None.

        After a FALL is emitted the fusion state resets (cooldown retained) so
        one event yields one alarm.
        """
        t = time.time() if now is None else float(now)
        spike_ok = (
            self._last_spike_time is not None
            and 0.0 <= (t - self._last_spike_time) <= self.fusion_window
        )
        still_ok = self.stillness_confirmed(t)
        if spike_ok and still_ok:
            event = self._emit_fall("spike-then-stillness fusion", now=t)
            self.reset()
            return event  # None if suppressed by alarm cooldown
        if spike_ok:
            return {
                "time": utc_now_iso(),
                "type": "SUSPECT",
                "detail": "MPU spike without confirmed stillness",
            }
        if still_ok:
            return {
                "time": utc_now_iso(),
                "type": "SUSPECT",
                "detail": "prolonged stillness without MPU spike",
            }
        return None

    def update(self, ax: float, ay: float, az: float, targets, now: float | None = None) -> dict | None:
        """One-call fusion step for ``main_pi.py``: MPU sample + radar frame.

        Returns the FALL event dict (also appended to the alarm file), a
        SUSPECT dict (not logged), or None. Never raises on bad sensor data
        and never performs network I/O.
        """
        t = time.time() if now is None else float(now)
        try:
            self.update_mpu(ax, ay, az, now=t)
        except (TypeError, ValueError):
            pass
        try:
            self.update_radar(targets, now=t)
        except (TypeError, ValueError, AttributeError):
            pass
        return self.check_fusion(now=t)

    # -- alarm output ---------------------------------------------------------
    def _emit_fall(self, detail: str, now: float | None = None) -> dict | None:
        """Build a FALL event; append one JSON line unless in cooldown.

        Event shape ``{time, type, detail}`` matches the required log schema.
        Returns the event dict, or None when suppressed by ``alarm_cooldown``.
        """
        t = time.time() if now is None else float(now)
        if (t - self._last_alarm_time) < self.alarm_cooldown:
            return None
        event = {"time": utc_now_iso(), "type": "FALL", "detail": str(detail)}
        try:
            parent = os.path.dirname(os.path.abspath(self.alarm_file))
            if parent and not os.path.exists(parent):
                os.makedirs(parent, exist_ok=True)
            with open(self.alarm_file, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(event, ensure_ascii=False) + "\n")
        except OSError:
            # Logging must never break detection; event is still returned.
            pass
        self._last_alarm_time = t
        return event


# ---------------------------------------------------------------------------
# Button GPIO17 -- immediate FALL alarm source
# ---------------------------------------------------------------------------

def button_pressed_to_event(detail: str = "button GPIO17 pressed", now: float | None = None) -> dict:
    """Build the immediate FALL event for a press of the button on GPIO17.

    The button (BCM pin 17, see ``BUTTON_PIN``) is the manual help call: unlike
    the fusion path it needs no spike/stillness evidence and no cooldown is
    applied here -- the caller (``main_pi.py`` sensor_loop) decides debouncing
    and logging, e.g. ``det._emit_fall(...)`` or its own POST. Pin monitoring
    itself (GPIO.setup/input) stays in ``main_pi.py``; hardware is NOT touched
    in this module so it stays import-safe and testable.

    Returns:
        Event dict ``{time, type: "FALL", detail}`` naming GPIO17 as source.
    """
    _ = now  # accepted for signature symmetry with detector updates
    return {
        "time": utc_now_iso(),
        "type": "FALL",
        "detail": f"{detail} (immediate alarm source: button GPIO{BUTTON_PIN})",
    }


# ---------------------------------------------------------------------------
# Lazy-hardware helpers (only place RPi.GPIO / smbus may be imported)
# ---------------------------------------------------------------------------

def read_live_mpu(bus_index: int = 1, addr: int = 0x68):
    """Read one live (ax, ay, az) tuple from the MPU6050. Lazy-imports smbus.

    Mirrors ``main_pi.py``: bus ``1``, addr ``0x68``, base ``ACCEL_XOUT_H``.
    Raises the underlying exception on failure (caller handles it).
    """
    import smbus2 as smbus  # lazy: keeps module import hardware-free

    base = 0x3B

    def _raw(bus, reg: int) -> int:
        high = bus.read_byte_data(addr, reg)
        low = bus.read_byte_data(addr, reg + 1)
        value = (high << 8) | low
        return value - 65536 if value > 32768 else value

    bus = smbus.SMBus(bus_index)
    return (_raw(bus, base), _raw(bus, base + 2), _raw(bus, base + 4))


def wait_for_button_press(poll_s: float = 0.05):
    """Block until the GPIO17 button is pressed. Lazy-imports RPi.GPIO.

    Convenience for scripts that want the manual alarm path outside
    ``main_pi.py``. Returns the FALL event dict from button_pressed_to_event().
    """
    import RPi.GPIO as GPIO  # lazy: keeps module import hardware-free

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    last = GPIO.input(BUTTON_PIN)
    try:
        while True:
            cur = GPIO.input(BUTTON_PIN)
            if cur == GPIO.HIGH and last == GPIO.LOW:
                return button_pressed_to_event()
            last = cur
            time.sleep(poll_s)
    finally:
        GPIO.cleanup()


# ---------------------------------------------------------------------------
# __main__ self-test on synthetic data (no hardware, no network)
# ---------------------------------------------------------------------------

def _rest_sample():
    """MPU at rest: 1g on Z only (~16384 LSB)."""
    return (0, 0, 16384)


def _walk_target(step: int) -> dict:
    """Synthetic walking target: valid dist, clearly active speed."""
    speeds = (120.0, 150.0, 90.0, 170.0, 110.0)
    return {"dist": 1.5, "x": 100.0 + step, "y": 1500.0, "speed": speeds[step % len(speeds)]}


def _still_target() -> dict:
    """Synthetic post-fall target: valid dist, near-zero speed."""
    return {"dist": 1.2, "x": 50.0, "y": 1200.0, "speed": 4.0}


def _run_case(name: str, fn) -> bool:
    try:
        ok, note = fn()
    except Exception as exc:  # noqa: BLE001 -- self-test must report, not crash
        print(f"FAIL: {name} -- raised {type(exc).__name__}: {exc}")
        return False
    print(f"{'PASS' if ok else 'FAIL'}: {name} -- {note}")
    return bool(ok)


def _case_normal_walk():
    """Normal walk: rest-level accel + active radar -> never FALL/SUSPECT-alarm."""
    import tempfile

    path = os.path.join(tempfile.gettempdir(), "fall_fusion_test_walk.jsonl")
    try:
        os.remove(path)
    except OSError:
        pass
    det = FallDetector(alarm_file=path)
    t0 = 1_700_000_000.0
    saw_fall = False
    for i in range(40):  # 4 s @ 10 Hz
        t = t0 + i * 0.1
        ev = det.update(*_rest_sample(), [_walk_target(i)], now=t)
        if ev is not None and ev.get("type") == "FALL":
            saw_fall = True
    logged = os.path.exists(path)
    ok = not saw_fall and not logged
    return ok, "no FALL over 4 s of synthetic walk" + ("" if ok else " (unexpected alarm)")


def _case_spike_then_stillness():
    """Sudden spike + stillness -> must produce a FALL alarm (file + return)."""
    import tempfile

    path = os.path.join(tempfile.gettempdir(), "fall_fusion_test_fall.jsonl")
    try:
        os.remove(path)
    except OSError:
        pass
    det = FallDetector(alarm_file=path)
    t0 = 1_700_000_000.0
    # 1 s of walking first (marks _had_active), then one impact spike ...
    for i in range(10):
        det.update(*_rest_sample(), [_walk_target(i)], now=t0 + i * 0.1)
    spike_t = t0 + 1.0
    det.update(0, 0, 31000, [_walk_target(99)], now=spike_t)  # > 25000 spike
    # ... then near-zero speed held past stillness_duration (3 s default).
    alarm = None
    for k in range(40):  # 4 s @ 10 Hz
        alarm = det.update(*_rest_sample(), [_still_target()], now=spike_t + 0.1 * (k + 1))
        if alarm is not None and alarm.get("type") == "FALL":
            break
    try:
        with open(path, encoding="utf-8") as fh:
            lines = [ln for ln in fh.read().splitlines() if ln.strip()]
        logged = len(lines) == 1 and json.loads(lines[0]).get("type") == "FALL"
    except OSError:
        logged = False
    ok = (
        alarm is not None
        and alarm.get("type") == "FALL"
        and set(("time", "type", "detail")) <= set(alarm)
        and logged
    )
    return ok, "spike+stillness raised FALL and wrote 1 jsonl line" if ok else "expected FALL alarm, got none"


def _case_lone_spike():
    """Lone spike, motion continues -> must NOT alarm (SUSPECT at most)."""
    import tempfile

    path = os.path.join(tempfile.gettempdir(), "fall_fusion_test_lone.jsonl")
    try:
        os.remove(path)
    except OSError:
        pass
    det = FallDetector(alarm_file=path)
    t0 = 1_700_000_000.0
    for i in range(10):
        det.update(*_rest_sample(), [_walk_target(i)], now=t0 + i * 0.1)
    spike_t = t0 + 1.0
    first = det.update(0, 0, 31000, [_walk_target(99)], now=spike_t)
    saw_fall = bool(first and first.get("type") == "FALL")
    # Keep walking: stillness never confirms, so no fusion.
    for k in range(40):
        ev = det.update(*_rest_sample(), [_walk_target(k)], now=spike_t + 0.1 * (k + 1))
        if ev is not None and ev.get("type") == "FALL":
            saw_fall = True
    logged = os.path.exists(path)
    ok = not saw_fall and not logged
    return ok, "lone spike stayed SUSPECT/None, no FALL, no file" if ok else "lone spike wrongly alarmed"


def _case_button():
    """GPIO17 helper returns an immediate FALL event naming the pin."""
    ev = button_pressed_to_event()
    ok = (
        ev.get("type") == "FALL"
        and "GPIO17" in str(ev.get("detail", ""))
        and set(("time", "type", "detail")) <= set(ev)
    )
    return ok, "button helper -> FALL mentioning GPIO17" if ok else f"bad button event: {ev!r}"


def _case_cooldown():
    """Second FALL inside 10 s alarm cooldown is suppressed."""
    import tempfile

    path = os.path.join(tempfile.gettempdir(), "fall_fusion_test_cd.jsonl")
    try:
        os.remove(path)
    except OSError:
        pass
    det = FallDetector(alarm_file=path)
    e1 = det._emit_fall("first", now=1000.0)
    e2 = det._emit_fall("second", now=1005.0)  # within default 10 s cooldown
    ok = e1 is not None and e1.get("type") == "FALL" and e2 is None
    return ok, "cooldown suppressed 2nd alarm at +5 s" if ok else "cooldown failed"


def main() -> int:
    print("fall_fusion self-test (synthetic data, no hardware, no network)")
    results = [
        _run_case("normal walk -> no alarm", _case_normal_walk),
        _run_case("spike + stillness -> FALL alarm", _case_spike_then_stillness),
        _run_case("lone spike (motion continues) -> NO alarm", _case_lone_spike),
        _run_case("button GPIO17 -> immediate FALL event", _case_button),
        _run_case("10 s alarm cooldown suppresses repeat", _case_cooldown),
    ]
    passed = sum(results)
    print(f"{passed}/{len(results)} cases passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
