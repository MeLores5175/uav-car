# d26_air_ground_uav：任务一真车正式版

本包只保留当前正式运行需要的四个 ROS 节点：

- `air_ground_mission_fsm.py`：任务一飞行状态机；
- `uav_udp_gateway.py`：地面站/ESP32 小车 UDP 通信；
- `platform_vision_node.py`：YOLO TensorRT + AprilTag 融合视觉；
- `drop_actuator_serial.py`：ESP32 单路舵机串口控制。

虚拟小车节点 `real_flight_fake_car.py`、离线映射检查脚本以及相关测试 launch 已删除。

## 目录

```text
d26_air_ground_uav/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── air_ground_mission.yaml
│   └── platform_vision.yaml
├── launch/
│   └── d26_task1_real.launch
├── scripts/
│   ├── air_ground_mission_fsm.py
│   ├── drop_actuator_serial.py
│   ├── platform_vision_node.py
│   └── uav_udp_gateway.py
├── docs/
└── esp32/
```

## 编译

```bash
cp -r d26_air_ground_uav /home/nvidia/catkin_ws/src/
cd /home/nvidia/catkin_ws
catkin_make
source devel/setup.bash
```

## 唯一正式启动命令

```bash
roslaunch d26_air_ground_uav d26_task1_real.launch
```

底层 MAVROS、Livox、Faster-LIO 已由其他服务启动时：

```bash
roslaunch d26_air_ground_uav d26_task1_real.launch \
  start_mavros:=false \
  start_livox:=false \
  start_faster_lio:=false
```

显示视觉调试窗口：

```bash
roslaunch d26_air_ground_uav d26_task1_real.launch \
  vision_show_debug:=true
```

## 默认网络参数

```text
无人机监听：0.0.0.0:8888
小车 IP：192.168.77.102
地面站回传端口：8889
```

地面站地址默认从第一条合法命令动态学习。小车 IP 必须与 ESP32 固件保持一致。

## 投放舵机

- 上电锁止：`A30`
- FSM 自动投放：`A90`
- 地面站起飞前手动锁止/释放也通过同一串口节点执行，避免多个进程抢占串口。
