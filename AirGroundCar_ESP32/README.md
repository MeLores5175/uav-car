# AirGroundCar_ESP32 板中心建模版

本版本在原有小车任务、UDP、仿真和里程计框架上，加入了学长要求的建模逻辑：

1. 采用阿克曼等效自行车/差速后轮运动学；
2. 轨迹约束点改为起降平台的板中心，而不是后轮轴中心；
3. 后两轮允许独立控制；
4. 通过板中心逆运动学求解后轮轴中心的 `v、ω`；
5. 再由 `speed_planner` 换算左右轮目标速度；
6. 弧线运行时输出 `ARC_SAMPLE`，用于后续将弧形路径左右轮参数固化到 ESP32；
7. 已加入真实步进驱动后端（PUL/DIR/ENA + LEDC 脉冲），但每圈等效脉冲数仍保持未标定保护值。

## 已写入的机械参数

单位均使用 cm：

```cpp
WHEEL_TRACK_CM       = 62.85
WHEEL_DIAMETER_CM    = 6.37
WHEEL_WIDTH_CM       = 1.35
WHEELBASE_CM         = 53.75
BOARD_TO_REAR_CM     = 27.50
BOARD_TO_FRONT_CM    = 26.25
```

场地轨迹：

```cpp
BOARD_PATH_RADIUS_CM = 75.0
STRAIGHT_LENGTH_CM   = 150.0
```

A 点被定义为板中心坐标：

```cpp
START_BOARD_X_CM = 150.0
START_BOARD_Y_CM = 200.0
START_YAW_RAD     = 90 deg
```

里程计内部的后轮轴中心起点会自动反算为约：

```text
rear_x = 150.0 cm
rear_y = 172.5 cm
```

## 主要文件

- `board_path_model.h/.cpp`
  - 新增模块；生成板中心参考轨迹并完成逆运动学。
- `route_controller.h/.cpp`
  - 不再用后轮轴累计里程定义官方路径进度；按照板中心参考路径推进。
- `odometry.h/.cpp`
  - 内部仍保存后轮轴中心位姿，同时增加板中心位置和左右轮累计行程。
- `mission_config.h`
  - 写入实测车体尺寸、建模开关、误差增益和仿真响应时间。
- `AirGroundCar_ESP32.ino`
  - 修改初始位姿、控制器调用方式，并记录弧线左右轮速度样本。
- `telemetry.cpp`
  - `x_cm/y_cm` 现在表示板中心；新增后轮轴位置、参考点和板中心误差。
- `vehicle_sim.cpp`
  - 车轮响应时间由 `SIM_WHEEL_TIME_CONSTANT_S` 配置。

另外修复了原工程 `udp_comm.cpp` 中 `START` 分支括号位置错误，避免 `RESET` 分支落到函数外部。

## 当前默认模式

```cpp
constexpr bool USE_SIMULATION = false;
constexpr bool ARC_MODEL_GENERATION_MODE = true;
constexpr float SIM_WHEEL_TIME_CONSTANT_S = 0.0f;
constexpr float STEPPER_PULSES_PER_REV = 0.0f;
```

含义：

- 选择真实步进后端；
- 在线求解板中心轨迹并输出弧线样本；
- `STEPPER_PULSES_PER_REV = 0.0f` 表示尚未标定，驱动初始化会明确报错且不会输出步进脉冲；
- 仿真响应时间参数仍保留，切换到仿真后端时可继续使用。

上电后串口会检查 `DriveBackend::begin()` 返回值，并输出：

```text
[DRIVE][STATUS] begun=... config_valid=... pulse_generator_ready=...
```

如果只是每圈脉冲数未标定，会额外提示设置 `STEPPER_PULSES_PER_REV > 0`。

公式验证通过后，可将：

```cpp
SIM_WHEEL_TIME_CONSTANT_S = 0.15f;
```

再测试加减速和车轮响应滞后。

## 实体启动按键

当前实体启动按键使用：

```text
ESP32 3V3 -> 瞬时按钮 -> GPIO32
```

GPIO32 配置为 `INPUT_PULLDOWN`：

- 松开：`LOW`；
- 按下：`HIGH`；
- 使用 ESP32 内部下拉，不需要额外连接 10 kΩ 下拉电阻；
- 软件采用 30 ms 非阻塞消抖，只在稳定上升沿触发一次。

该引脚不与当前步进驱动引脚冲突：

```text
左轮：STP=26, DIR=27, EN=25
右轮：STP=19, DIR=21, EN=18
按键：GPIO32
```

## 串口测试

串口波特率：115200。

依次发送：

```text
1
A
S
```

含义：

```text
1  选择任务 T1
A  ARM，run_id=R001
S  模拟本地启动
```

也可以使用：

```text
2  选择任务 T2
0  整体任务复位
R  只复位路线、速度、里程和仿真后端
P  立即输出一次遥测
```

正常路线事件顺序：

```text
B
C
D
A_FINISH
```

## 弧线参数输出

任务启动后会先输出：

```text
ARC_SAMPLE_FORMAT,segment,progress01,left_cm_s,right_cm_s
```

在 B-C、D-A 两段半圆中，每 50 ms 输出一行：

```text
ARC_SAMPLE,2,0.12345,左轮速度,右轮速度
ARC_SAMPLE,4,0.12345,左轮速度,右轮速度
```

其中路段编号：

```text
2 = ARC_BC
4 = ARC_DA
```

`progress01`：

```text
0.0 = 刚进入半圆
1.0 = 半圆结束
```

左右轮速度单位为 `cm/s`。

这些数据是后续 `arc_profile` 参数表的来源。当前工程没有填入假数据，先运行仿真、检查轨迹后再固化。

## 遥测新增字段

`TEL:CAR` 中：

- `x_cm、y_cm`：板中心；
- `rear_x_cm、rear_y_cm`：后轮轴中心；
- `ref_x_cm、ref_y_cm`：板中心参考点；
- `board_error_cm`：板中心实际位置到参考点的误差；
- `route_progress_cm`：板中心参考路径累计进度；
- `segment_progress`：当前路段归一化进度；
- `left_distance_cm、right_distance_cm`：左右轮累计行程。
- `driver_ready`：当前后端是否具备实际输出条件；
- `stepper_config_valid`：步进机械换算参数是否有效；
- `stepper_pulse_generator_ready`：左右 LEDC 脉冲通道是否均挂接成功；
- `driver_alarm`：预留的驱动器 ALM 状态；当前 ALM 未接线，因此固定为 `false`。

## 轨迹检查公式

A-B：

```text
x_board ≈ 150 cm
```

B-C：

```text
(x_board - 225)^2 + (y_board - 350)^2 ≈ 75^2
```

C-D：

```text
x_board ≈ 300 cm
```

D-A：

```text
(x_board - 225)^2 + (y_board - 200)^2 ≈ 75^2
```

一圈结束时板中心应回到 A 点附近。

注意：当前约束核心是“板中心沿轨迹”。仅约束一个偏置点的位置时，闭合轨迹结束处的车身航向不一定严格等于起始航向；若比赛还要求停车姿态，需要后续单独加入终点航向整理动作。

## 当前仍未完成

- 根据仿真输出生成 `arc_profile.h/.cpp` 并切换到弧线查表调用；
- 驱动器 ALM 输入接线与故障上报；
- 驱动器细分、传动比和每圈等效脉冲数；
- 有效轮径、有效轮距、左右轮补偿标定；
- 实体启动按键完整消抖；
- UAV START 命令 ACK、超时重发和去重；
- 实车编码器、IMU、视觉等位置纠偏。

## 推荐下一步

1. 保持 `SIM_WHEEL_TIME_CONSTANT_S = 0.0f` 跑通 T1/T2；
2. 保存串口中的 `ARC_SAMPLE`；
3. 检查 `board_error_cm` 和路线方向；
4. 改为 `SIM_WHEEL_TIME_CONSTANT_S = 0.15f` 再测动态响应；
5. 根据稳定结果生成弧线查表参数；
6. 标定驱动器细分、传动比和 `STEPPER_PULSES_PER_REV`，再做轮径、轮距和左右轮补偿标定。
