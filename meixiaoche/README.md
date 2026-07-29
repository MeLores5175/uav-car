# d26_air_ground_uav

2026 年 D 题“陆空协同无人机系统”无人机端 ROS1 代码包。

代码直接包含两套完整任务：

- `drop`：小车启动 → 无人机起飞至 1.5 m → 首次满足位置/高度/速度阈值后立即开始固定 3 s 悬停计时（期间失稳不重置，3 s 未再次达标也继续任务）→ 追踪小车预测位置完成截获 → 仅在 C-D 直线段带投放装置 XY 偏置完成投放 → 返回 H 点 → `AUTO.LAND`。
- 航向策略：全任务统一使用 `takeoff.yaw_deg`，小车 yaw 不参与无人机航向或跟随偏置计算；半圆伴飞时无人机只平移、不自转。
- 投放兜底：进入 `DROP_ALIGN` 后优先按精确阈值投放；若超过 `align_timeout_s` 且仍在 C-D 有效窗口内，则直接发送投放命令；已进入瞄准后到达窗口末端时也直接投放。离开 C-D 后不盲投。
- `dynamic_land`：小车启动 → 无人机起飞并截获 → 伴飞等待 C-D → 仅在 C-D 直线段连续下降并着陆 → 自动上锁并停留 5 s → 带小车速度前馈重新起飞 → 返回 H 点 → `AUTO.LAND`。

动态降落只有一个窗口：**C-D 直线段**。代码中没有 A-B 降落逻辑。

## 目录

```text
d26_air_ground_uav/
├── CMakeLists.txt
├── package.xml
├── config/air_ground_mission.yaml
├── launch/d26_real_flight.launch
├── launch/d26_sitl_test.launch
├── scripts/air_ground_mission_fsm.py
├── scripts/uav_udp_gateway.py
├── scripts/landing_marker_vision.py
├── scripts/drop_actuator_serial.py
├── scripts/car_udp_bridge.py          # 旧协议兼容，正式 launch 默认关闭
├── scripts/status_udp_sender.py       # 旧协议兼容，正式 launch 默认关闭
├── scripts/car_state_simulator.py
├── scripts/real_flight_fake_car.py
├── launch/d26_real_flight_virtual_car_drop_test.launch
└── docs/INTERFACES.md
```

## 直接部署

```bash
cd /home/nvidia/catkin_ws/src
unzip -o d26_air_ground_uav.zip
cd /home/nvidia/catkin_ws
catkin_make
source devel/setup.bash
```

- 第 1 行进入 ROS 工作空间源码目录。
- 第 2 行解压代码包，生成 `d26_air_ground_uav` 包。
- 第 3 行返回工作空间根目录。
- 第 4 行编译 ROS 包。
- 第 5 行加载本次编译生成的 ROS 环境。

## 实飞启动

```bash
source /home/nvidia/catkin_ws/devel/setup.bash
roslaunch d26_air_ground_uav d26_real_flight.launch
```

- 第 1 行加载 ROS 工作空间。
- 第 2 行同时启动 MAVROS、Livox、Faster-LIO、任务状态机、下视视觉、UDP V1.1 网关和投放串口。

若 MAVROS、Livox、Faster-LIO 已由你们原来的一键脚本启动：

```bash
roslaunch d26_air_ground_uav d26_real_flight.launch \
  start_mavros:=false \
  start_livox:=false \
  start_faster_lio:=false
```

- `start_mavros:=false` 避免重复启动 MAVROS。
- `start_livox:=false` 避免重复启动 Mid-360 驱动。
- `start_faster_lio:=false` 避免重复启动 Faster-LIO。

## 任务选择与启动（UDP V1.1）

无人机统一监听：

```text
192.168.151.102:8888
```

地面站先选择任务并发送：

```text
CMD:0001:BOOT:T1
```

或：

```text
CMD:0001:BOOT:T2
```

无人机回复 `ACK`，准备完成后发送：

```text
EVT:UAV_BOOT_READY:T1
```

正式 START 由小车发送：

```text
CMD:1001:START:R001:T1
```

或：

```text
CMD:1001:START:R002:T2
```

无人机确认：

```text
ACK:1001:START:OK:R001
```

地面站安全降落命令：

```text
CMD:0005:LAND:R001
```

落地后复位：

```text
CMD:0006:RESET
```

仍可使用 ROS 话题做单机调试，但正式比赛以 V1.1 UDP 命令为准。

## 实飞前必须修改的参数

打开：

```text
config/air_ground_mission.yaml
```

至少实测并修改以下内容：

1. `udp_protocol.ground_station.ip`、`udp_protocol.car.ip`：现场地面站和小车 IP。
2. `udp_protocol.field_transform.h_field_x_cm`、`h_field_y_cm`：H 点在场地坐标中的位置。
3. `udp_protocol.field_transform.local_x_to_field_yaw_deg`：MAVROS local +X 在场地坐标中的方向。
4. `mission.dynamic_landing.platform_height_m`：H 点地面到小车平台表面的实际高度。
5. `mission.dynamic_landing.range_topic`：确认该话题能实时发布下视测距；默认要求测距或 PX4 落地状态确认后才上锁。
6. `vision.image_down_to_body_forward_sign` 与 `image_right_to_body_left_sign`：下视相机方向符号。
7. `actuator.port`：投放控制板串口。
8. `mission.drop.release_offset_x_m`、`release_offset_y_m`：投放出口相对无人机中心的 XY 偏置。
9. `udp_protocol.track.straight_length_m`、`radius_m`：用于由 `path_s_cm` 计算区段进度的赛道几何参数。

## 小车连续遥测（V1.1 修订版）

小车向无人机 `8888` 端口发送：

```text
TEL:CAR:{"seq":310,"time_ms":18390,"run":"R002","task":2,"state":"RUNNING","x_cm":318.5,"y_cm":305.2,"speed_cm_s":12.0,"yaw_deg":270.0,"point":"C","segment":"CD","path_s_cm":512.4,"line_detected":true,"battery":82,"error":0}
```

新增的两个必需字段：

- `segment`：`AB/BC/CD/DA/UNKNOWN`，任务二只有 `CD` 才允许进入下降；
- `path_s_cm`：本轮从 A 点开始沿黑线顺时针累计里程，单位 cm，每轮开始清零。

网关会使用 `path_s_cm` 计算 `segment_progress`，并与 `segment` 交叉校验。明显不一致时把控制区段置为 `UNKNOWN`，可以继续伴飞，但禁止动态下降。

完整协议见 `docs/UDP_PROTOCOL_V1.1.md`。

## 安全控制

```bash
rostopic pub -1 /uav/stop std_msgs/Bool "data: true"
```

立即中止任务并请求 `AUTO.LAND`。

```bash
rostopic pub -1 /uav/land std_msgs/Bool "data: true"
```

中止当前任务并安全降落。

动态下降中出现以下情况会停止下降或退出：

- 下视平台视觉短时丢失：保持当前目标高度；
- 视觉丢失超过 `vision_abort_timeout_s`：退出本次下降并重新伴飞；
- 小车离开 C-D 或超过 `abort_progress`：放弃动态着陆并返回 H 点；
- 小车通信或无人机定位超时：按安全逻辑返航或降落；
- 不使用移动平台 `AUTO.LAND`，只有返回固定 H 点后才切 `AUTO.LAND`。

## 已完成的离线检查

- 所有 Python 文件通过 `py_compile` 语法检查；
- YAML 可正常解析；
- `package.xml` 和 launch XML 可正常解析；
- 脚本已设置可执行权限。

这些检查不能替代 FS-J310 实机的坐标、相机、平台高度、PX4 降落检测和投放串口标定。


## 本版关键控制规则

- 任务一首次满足起飞位置、目标高度和速度阈值时立即进入 `DROP_HOVER`，从该时刻固定计时 3 秒；计时期间不因失稳清零，未满 3 秒绝不提前离开。
- 截获阶段使用 `car_position + car_velocity × intercept_prediction_s` 的预测位置，而不是追逐当前测量点。
- 投放对准和等待执行 ACK 期间均使用投放出口偏置补偿；YAML 偏置表示投放出口相对无人机控制中心的位置，代码自动取反得到无人机中心目标。
- 地面站只显示状态，不进入控制闭环；需要接收的信息见 `docs/GROUND_STATION_DATA.md`。

## 真实无人机 + 虚拟小车的任务一测试

不要直接把 `car_state_simulator.py` 用于实飞。请使用专用节点和 launch：

```bash
roslaunch d26_air_ground_uav d26_real_flight_virtual_car_drop_test.launch car_speed:=0.12
```

确认定位和 H 点正常后，再执行：

```bash
rostopic pub -1 /fake_car/start_mission std_msgs/Bool "data: true"
```

该 launch 只用于任务一低速航迹验证：关闭真实小车 UDP、真实视觉和真实投放机构，使用虚拟小车、合成视觉和模拟投放 ACK。完整说明见 `docs/REAL_FLIGHT_FAKE_CAR_TEST.md`。
