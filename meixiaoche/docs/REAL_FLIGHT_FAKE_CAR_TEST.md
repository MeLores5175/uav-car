# 真实无人机 + 虚拟小车测试

本测试只用于任务一 `drop` 的低速航迹实飞验证。虚拟节点同时发布：

- `/car/state`：虚拟小车位置、速度、区段和进度；
- `/uav/platform_vision`：由真实无人机位姿与虚拟小车位置计算的合成相对位置。

专用 launch 已关闭真实小车 UDP 网关、真实相机视觉和真实投放机构，避免话题冲突与误投放。

## 启动

```bash
source /home/nvidia/catkin_ws/devel/setup.bash
roslaunch d26_air_ground_uav d26_real_flight_virtual_car_drop_test.launch \
  car_speed:=0.12
```

## 检查

```bash
rostopic echo -n 1 /fake_car/status
rostopic echo -n 1 /car/state
rostopic echo -n 1 /uav/mission_status
```

确认：

- `home_ready=true`；
- `pose_ready=true`；
- 虚拟小车初始 `running=false`；
- FSM 为 `WAIT_START`；
- `/car/state` 只有 `/real_flight_fake_car` 一个发布者；
- `/uav/platform_vision` 只有 `/real_flight_fake_car` 一个发布者。

## 开始任务

```bash
rostopic pub -1 /fake_car/start_mission std_msgs/Bool "data: true"
```

节点先选择 `drop` 任务，再触发 FSM 开始。虚拟小车不会立即运动；无人机完成起飞与 3 秒悬停，进入 `INTERCEPT` 后虚拟小车才自动开始沿赛道运动。

## 随时停止

```bash
rostopic pub -1 /uav/stop std_msgs/Bool "data: true"
```

FSM 进入返航、固定点降落、应急降落或等待复位时，虚拟小车会自动停止。

## 手动控制虚拟小车

```bash
rostopic pub -1 /fake_car/run std_msgs/Bool "data: false"
rostopic pub -1 /fake_car/run std_msgs/Bool "data: true"
rostopic pub -1 /fake_car/speed std_msgs/Float32 "data: 0.08"
rostopic pub -1 /fake_car/reset std_msgs/Bool "data: true"
```

## 禁止事项

- 不得把本 launch 改为 `dynamic_land` 后在空场地直接测试；合成视觉会让状态机认为虚拟平台真实存在。
- 不得同时启动 `uav_udp_gateway.py`、`car_udp_bridge.py` 或真实 `landing_marker_vision.py`；否则同一话题会有多个发布者。
- 必须先标定 YAML 中 `car_udp.track.a_x_m`、`a_y_m`、`ab_yaw_deg`，否则无人机会沿错误的实际坐标飞行。
- 首次测试应拆除载荷、降低速度，并准备遥控器模式切换和 `/uav/stop`。
