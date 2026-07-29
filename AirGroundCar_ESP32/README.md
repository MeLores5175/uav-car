# AirGroundCar_ESP32 初版架构

这是从旧骨架调整出的第一版基础工程，目前目标不是直接驱动实车，而是先固定模块接口，并通过仿真跑通标准路线和任务状态。

## 当前已经具备

- `mission_manager`：T1/T2、READY、ARMED、RUNNING、FINISHED 状态。
- `route_controller`：A-B 直线、B-C 半圆、C-D 直线、D-A 半圆。
- `speed_planner`：线速度和角速度平滑变化，换算左右轮目标速度。
- `drive_backend`：统一仿真后端与未来真实步进后端。
- `vehicle_sim`：当前默认启用，可在没有底盘时运行完整路线。
- `stepper_driver`：只搭好接口，尚未输出 PUL/DIR。
- `odometry`：根据左右轮速度推算 x、y、yaw。
- `udp_comm`：基础 Wi-Fi/UDP 框架与 PING、STATUS、MODE、ARM、RESET 命令。
- `telemetry`：串口和 UDP 输出 `HB:CAR`、`TEL:CAR`、关键点事件。

## 当前默认设置

在 `mission_config.h` 中：

```cpp
constexpr bool USE_SIMULATION = true;
constexpr bool ENABLE_WIFI_UDP = false;
constexpr bool ALLOW_REMOTE_START = false;
```

因此第一次上传后无需联网，直接通过串口测试。

## 串口测试顺序

波特率：115200。

依次发送：

```text
1     选择任务 1
A     设置 run_id=R001 并进入 ARMED
S     模拟按下小车本地启动按键
```

也可以使用：

```text
2     选择任务 2
0     任务复位
R     只复位路线、速度、里程和仿真后端
P     立即输出一次状态
```

正常情况下会依次出现 B、C、D、A_FINISH 事件，最后进入 FINISHED。

## UDP 初步启用

1. 在 `mission_config.cpp` 填入 Wi-Fi 名称和密码。
2. 将 `ENABLE_WIFI_UDP` 改为 `true`。
3. 小车监听端口为 8890。

当前支持：

```text
CMD:0101:PING
CMD:0102:STATUS
CMD:0103:MODE:T1
CMD:0104:MODE:T2
CMD:0105:ARM:R001
CMD:0106:RESET
```

正式版默认拒绝远程 START，使用串口 `S` 代替未来实体按键。

## 当前没有完成

- 闭环步进驱动器的 PUL/DIR/ENA/ALM 实现。
- 实体启动按键完整消抖。
- 小车发给无人机的 START 命令 ACK、超时重发和去重。
- 轮径、轮距、电子齿轮比和左右轮补偿标定。
- 循迹、IMU、视觉或其他位置纠偏。

这些内容等底盘、电机和驱动器型号确定后再继续补充。
