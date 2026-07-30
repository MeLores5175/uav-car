# 陆空协同无人机系统 HTML 地面站 V1

这是面向 D 题的第一版地面站，采用：

- Python `aiohttp` 后端；
- 原生 WebSocket 实时推送；
- UDP 同时连接无人机和 ESP32 小车；
- 纯本地 HTML / CSS / JavaScript，不使用登录、数据库、CDN 或在线资源；
- 横向比赛地图，显示无人机和小车位置、航向及历史轨迹；
- 实时文字显示起飞、搜索、伴飞、抛投、动态降落、返航等状态；
- 关键控制命令使用短文本，遥测使用 JSON；
- `LAND` 按住 1 秒触发，防止误触。

## 1. 文件结构

```text
land_air_ground_station_v1/
├── app.py                    地面站主程序
├── config.json               实机网络配置
├── config.mock.json          本地模拟配置
├── requirements.txt          Python 依赖
├── protocol_v1.1.md          UDP 通信协议
├── templates/
│   └── index.html            网页结构
├── static/
│   ├── app.js                WebSocket、按钮、地图和状态更新
│   └── style.css             界面样式
├── tools/
│   └── mock_devices.py       无人机和小车本地模拟器
└── assets/
    ├── field_map_reference.png
    └── field_map_dimensioned.png
```

## 2. 安装

建议 Python 3.9 或更高版本。

```bash
python -m pip install -r requirements.txt
```

## 3. 先运行本地演示

终端 1：

```bash
python tools/mock_devices.py
```

终端 2：

```bash
python app.py --config config.mock.json
```

也可以在 Windows 双击：

```text
start_demo.bat
```

浏览器会自动打开：

```text
http://127.0.0.1:5000
```

推荐依次测试：

1. 选择任务 1 或任务 2；
2. 点击“准备所选任务”；
3. 点击“PING 双方”；
4. 点击“START 小车”；
5. 查看地图位置、轨迹、状态和日志；
6. 按住红色 LAND 按钮 1 秒，测试中止降落。

## 4. 连接实机

修改 `config.json`：

```json
{
  "network": {
    "bind_ip": "0.0.0.0",
    "gs_port": 8889
  },
  "devices": {
    "uav": {
      "ip": "192.168.151.102",
      "port": 8888
    },
    "car": {
      "ip": "192.168.151.103",
      "port": 8890
    }
  }
}
```

然后运行：

```bash
python app.py
```

Windows 第一次运行时，若防火墙弹窗，请允许 Python 在“专用网络”通信。

## 5. 当前按钮对应的 UDP 命令

### 准备任务 1

```text
GS → UAV  CMD:<id>:BOOT:T1
GS → CAR  CMD:<id>:MODE:T1
```

### 准备任务 2

```text
GS → UAV  CMD:<id>:BOOT:T2
GS → CAR  CMD:<id>:MODE:T2
```

### 开始任务

```text
GS → CAR  CMD:<id>:START:Rxxx
```

正式任务只向小车发送 START。小车启动后，再按协议向无人机发送 START。

### 安全降落

```text
GS → UAV  CMD:<id>:LAND:Rxxx
```

LAND 的含义是中止任务并受控降落，不是直接停桨。

## 6. 地图坐标

设备上报仍使用赛题原始坐标：

- 左下角为 `(0,0)`；
- `x_cm` 范围 `0~400`；
- `y_cm` 范围 `0~500`。

为适配横屏，网页把地图顺时针旋转 90°，显示转换为：

```text
网页横坐标 = y_cm
网页纵坐标 = x_cm
```

无人机与小车端不需要为横屏修改坐标。

## 7. 目前支持的报文

```text
ACK:<cmd_id>:<action>:<result>[:detail]
ERR:<cmd_id>:<action>:<error_code>[:detail]
EVT:<event>[:arg1]...
HB:UAV:<seq>:<state>
HB:CAR:<seq>:<state>
TEL:UAV:{json}
TEL:CAR:{json}
```

同时兼容旧仓库中的：

```text
ACK:PING:...
STATUS:key=value;...
VISION:IMAGE:{json}
```

## 8. 比赛前必须确认

- 笔记本、小车、无人机处于同一局域网；
- 三者 IP 和端口与 `config.json` 一致；
- 小车和无人机遥测坐标使用厘米；
- 地面站在断开外网的情况下仍可完整运行；
- 无人机端 LAND 接口确实执行受控降落，而不是直接停桨；
- 正式测试前关闭本地模拟器，避免占用 8888、8889、8890 端口。


## V1.5：现场通信设置

主页顶部新增“设置”按钮，会打开独立设置页。可以修改无人机和小车的 IPv4 地址，
保存后立即生效并写回当前配置文件，不需要重启地面站。

- 实机默认：UAV `192.168.151.102`，CAR `192.168.151.103`
- Mock 默认：UAV/CAR 均为 `127.0.0.1`
- 设置页只允许修改 IP，UDP 端口仍由配置文件固定
- 任务运行或执行 LAND 时禁止修改 IP
- 每次保存会生成 `.bak` 备份


## V1.6：删除任务计时并优化主界面

- 删除非必需的任务计时器，以及后端对应的累计时间字段。
- LAND 完成并收到 `UAV_LANDED` 后，任务状态显示为“任务已中止”。
- 任务完成、中止或故障后，必须先 RESET，避免误触再次 START。
- 任务概览、设备状态、控制区和日志区重新整理；1080p 屏幕下尽量在一屏内显示。


## V1.7：恢复页面滚动

- 删除顶部副标题和地图坐标说明。
- 取消桌面端 `body { overflow: hidden; }` 和固定视口高度。
- 页面可以正常上下滚动，右侧通信日志不再被裁掉。
- 日志列表仍支持独立滚动。
