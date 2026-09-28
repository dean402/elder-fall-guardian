# elder-fall-guardian · 基于树莓派 + 毫米波雷达的老人跌倒监护原型（边缘计算）

> LD2450 看人在哪、融合算法判摔没摔，DHT / MPU / MAX30102 采环境体征，大按键一键求助。
> 树莓派本机独立判断 + 网页看板，断网照样报警落盘。

- 主控：树莓派 4B + Python（边缘端，不依赖云）
- 状态：L1 闭环已通，可演示
- 协议：MIT

## 功能

1. **存在 + 轨迹**：LD2450 实时坐标，有人 / 无人、活动量
2. **跌倒报警**：MPU 加速度尖峰 + 雷达静止双信号融合，确认窗 + 冷却去重，写 `alarms.jsonl` + 网页横幅 + 触发拍照（无摄像头时跳过）
3. **环境体征**：DHT11 温湿度、MPU 活动量、MAX30102 心率血氧（需指夹，否则为野值）
4. **一键求助**：大按键按下即报警（GPIO17，三脚有源模块，VCC 接 3V3）
5. **Web 看板**：同局域网浏览器打开 `http://<树莓派IP>:8080`，总览 / 轨迹 / 报警 / 设备 / 关于，2 秒轮询
6. **离线优先**：融合判断、落盘、拍照全在派本机，断网照常报警，联网后补看记录

## 硬件清单（全现货）

| 模块 | 接口 | 说明 |
|---|---|---|
| 树莓派 4B | 主控 | Python 3.13，`food_project` + `webapp` 双 systemd 自启 |
| LD2450 24G 轨迹雷达 | UART2 `/dev/ttyAMA2` 256000 | 在用，多目标坐标 |
| LD2410 存在雷达 | UART4 256000 + GPIO | 在流，存在检测 |
| MPU6050 六轴 | 硬 I2C bus1 `0x68` | 跌倒尖峰源 + 计步 |
| DHT11 温湿度 | GPIO5 单总线 | 每 2 秒读一次 |
| MAX30102 心率血氧 | 软 I2C bus4 `0x57` | 需手指夹住才准 |
| 三脚有源大按键 | GPIO17，VCC→3V3 | 空闲 LOW / 按下 HIGH（代码已做逻辑适配） |
| CSI 摄像头 | CSI 排线（rpicam） | 报警联动拍照，可选 |
| HLK-60MI01 | UART0 | 已封存（疑似模块损坏），跌倒改走 LD2450+MPU 融合 |

详细接线见 `docs/接线总表.md`，四层架构见 `docs/架构说明.md`。

## 融合算法（一句话）

`fall_fusion.FallDetector`：先捕捉 MPU 加速度尖峰（默认阈值 25000，基线约 16384），再看雷达是否随后持续 N 秒近乎静止（默认 3 秒），两条件在 10 秒融合窗口内按序成立才判 FALL，10 秒冷却去重。任一单信号只记观察、不报警。

可调参数收敛在 `rules.json`（与 `webapp/rules.json` 同内容），调参不用改代码：

```json
{"active_speed_threshold":100.0,"alarm_cooldown":10.0,"fusion_window":10.0,"spike_threshold":25000,"still_speed_threshold":50,"stillness_duration":3.0}
```

无硬件单机自测：`fall_fusion.py` 纯函数零硬件依赖，可直接 import 调。

## 快速开始

```bash
# 1. 树莓派上拉代码（示例路径 ~/food_project）
git clone https://github.com/denglizibinghua/elder-fall-guardian.git ~/food_project
cd ~/food_project
pip install -r requirements.txt   # 仅 main_pi.py 需要硬件库

# 2. 跑主循环 + 看板（前台联调）
python3 main_pi.py
python3 webapp/app.py             # 另开一终端，:8080

# 3. 同 WiFi 的手机/电脑浏览器打开
#    http://<树莓派IP>:8080
```

生产用 systemd 双服务自启，模板见 `webapp/app.py` 头部注释（按你的用户名和路径改 `User=` / `WorkingDirectory=` 即可）。

## 演示四步

1. **存在**：人走过 →“有人”+ 坐标动；近处挥手 → 坐标跟手
2. **跌倒模拟**：蹲下慢躺保持几秒别动 → FALL 横幅 + `alarms.jsonl` 落盘 + 自动拍照
3. **按键求助**：按下红色按钮 → 状态翻转 + 落盘 + 拍照
4. **断网加分**：断网重演一次 → 本地照样报警落盘（看板当时不可看，联网后补刷验证）

## 仓库结构

```
elder-fall-guardian/
├── README.md               本文件
├── LICENSE                 MIT
├── requirements.txt        树莓派硬件库（融合+看板仅标准库）
├── rules.json              融合可调参数
├── fall_fusion.py          跌倒融合引擎（FallDetector，无硬件依赖）
├── main_pi.py              派端主循环快照（多线程采集+落盘+拍照）
├── webapp/
│   ├── app.py              本地看板（单文件 http.server，:8080，零依赖）
│   ├── devices.json        设备注册表
│   └── rules.json          规则配置（与根 rules.json 同步）
└── docs/
    ├── 架构说明.md          物联网四层 + 数据流 + 报告直抄三段
    ├── 功能描述.md          一句话 + 功能 + 验收
    ├── 接线总表.md          派侧接线实测版
    ├── 学长代码说明.md       旧 food_project 整理 + 开源对照
    ├── ROADMAP.md          L0-L3 进阶路线
    ├── 进阶计划.md          看板三修 + 误报采集 + 录屏 + 报告
    └── 误报率记录表.md       P2 实测记录模板（日常30次+模拟跌倒10次）
```

说明：本地排障过程脚本（`pi_stage*.sh`）、派全量快照、宿舍实拍图、运行期生成的 `status.json` / `alarms.jsonl` / `__pycache__` 均未收入本库。旧二次元管家方向草稿（`brain_api_draft.py`）已作废不收录。`main_pi.py` 中遗留的远端上传 URL 已改为占位 `http://192.0.2.1:8000`（文档示例地址段），可用 `BACKEND_BASE_URL` 环境变量覆盖；本地监护链路不依赖它。

## 踩坑速记

- 双服务抢 CPU：旧 `food-monitor` / `dashboard` 已停用，只留 `food_project` + `webapp`
- 按键是三脚有源模块，VCC 只能接 3V3，禁 5V
- 面包板同排 5 孔内部直通，别把 GPIO 和 GND 落同排
- 写派脚本别带 BOM；SSH 卡顿多半是派 CPU 忙，晾一会儿再试

## License

MIT，见 `LICENSE`。
