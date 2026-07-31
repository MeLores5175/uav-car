# ROS 与 UDP V1.1 接口

## 外部 UDP

无人机统一监听 `udp_protocol.uav.listen_port`，默认 `8888`。正式 launch 只启动 `uav_udp_gateway.py`，旧版 `car_udp_bridge.py` 和 `status_udp_sender.py` 默认关闭。

### 地面站到无人机

```text
CMD:<id>:PING
CMD:<id>:STATUS
CMD:<id>:BOOT:T1
CMD:<id>:BOOT:T2
CMD:<id>:LAND:<run_id>
CMD:<id>:RESET
```

### 小车到无人机

```text
CMD:<id>:START:<run_id>:T1
CMD:<id>:START:<run_id>:T2
TEL:CAR:{json}
EVT:CAR_POINT:<run_id>:B/C/D/A_FINISH
```

`TEL:CAR` 必须包含 `segment` 和 `path_s_cm`。完整定义见 `UDP_PROTOCOL_V1.1.md`。

### 无人机到地面站

```text
ACK:...
ERR:...
HB:UAV:<seq>:<state>
TEL:UAV:{json}
EVT:...
```

## ROS 输入

| 话题 | 类型 | 说明 |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | 飞控连接、模式、解锁状态 |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | PX4 着陆状态 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Faster-LIO/MAVROS 本地位姿 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | 本地速度 |
| `/mavros/distance_sensor/rangefinder_pub` | `sensor_msgs/Range` | 下视测距 |
| `/car/state` | `std_msgs/String` JSON | 网关转换后的相对 H 点小车状态 |
| `/uav/platform_vision` | `std_msgs/String` JSON | 下视平台相对位置 |
| `/uav/mission_command` | `std_msgs/String` JSON | 原子化 START 命令 |
| `/uav/start` | `std_msgs/Bool` | 旧式 ROS 调试启动 |
| `/uav/mission_type` | `std_msgs/String` | `drop` / `dynamic_land` |
| `/uav/stop` | `std_msgs/Bool` | 急停降落 |
| `/uav/land` | `std_msgs/Bool` | 中止并受控降落 |
| `/uav/reset` | `std_msgs/Bool` | 落地上锁后复位 |
| `/uav/drop_ack` | `std_msgs/String` JSON | 投放机构 ACK |

## ROS 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | OFFBOARD 指令 |
| `/uav/mission_status` | `std_msgs/String` JSON | 无人机内部完整状态，供 UDP 网关转换 |
| `/uav/mission_event` | `std_msgs/String` JSON | 关键事件 |
| `/uav/mission_command_result` | `std_msgs/String` JSON | START 接受/拒绝结果 |
| `/uav/platform_scan_enable` | `std_msgs/Bool` | 视觉节点启停 |
| `/uav/drop_cmd` | `std_msgs/String` JSON | 幂等投放命令 |

## `/car/state` 内部 JSON

```json
{
  "x": 1.2,
  "y": 2.1,
  "vx": 0.0,
  "vy": -0.12,
  "yaw": -1.5708,
  "segment": "CD",
  "segment_reported": "CD",
  "segment_from_path": "CD",
  "segment_consistent": true,
  "segment_progress": 0.35,
  "path_s": 5.12,
  "path_s_cm": 512.0,
  "running": true,
  "run_id": "R002"
}
```

默认 `x/y` 是相对 H 点的 MAVROS local 坐标，单位 m；FSM 会加 `home_x/home_y` 后用于控制。

## 投放出口 XY 偏置

YAML 中 `release_offset_frame=body` 时，`+X` 为机头前方，`+Y` 为机体左方。字段表示投放出口相对无人机控制中心的位置，控制器自动取反补偿，使投放出口对准小车平台中心。

## 单路舵机投放接口

地面站起飞前手动控制：

```text
CMD:<cmd_id>:SERVO:LOCK
CMD:<cmd_id>:SERVO:RELEASE
```

无人机成功执行后回复：

```text
ACK:<cmd_id>:SERVO:OK:LOCK
ACK:<cmd_id>:SERVO:OK:RELEASE
```

安全限制：FCU 必须未解锁、FSM 状态必须为 `WAIT_START`，否则返回 `ERR`。
网关和 FSM 都只发布 `/uav/drop_cmd`；只有 `drop_actuator_serial.py` 打开 ESP32 串口。
`A30` 为锁止，`A90` 为释放。
