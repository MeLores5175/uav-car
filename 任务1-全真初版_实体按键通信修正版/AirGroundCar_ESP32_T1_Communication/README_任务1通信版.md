# 小车任务1通信实车版

## 正式流程

1. ESP32 上电，连接路由器并监听 UDP 8890。
2. 地面站选择任务：
   - GS -> UAV：`CMD:<id>:BOOT:T1/T2`
   - GS -> CAR：`CMD:<id>:MODE:T1/T2`
3. 小车回复：
   - `ACK:<id>:MODE:OK:T1/T2`
   - `EVT:CAR_READY:T1/T2`
4. 队员按下 GPIO32 外接按键。
5. 小车进入 10 秒倒计时，电机不动，并向地面站发送：
   - `EVT:CAR_COUNTDOWN:<run_id>:<task>:10`
6. 10 秒结束后，小车同时：
   - 开始实际运动；
   - 向无人机可靠发送 `CMD:<id>:START:<run_id>:T1/T2`；
   - 向地面站发送 `EVT:MISSION_START:<run_id>:T1/T2`。
7. 运动期间以 20 Hz 向无人机和地面站同时发送 `TEL:CAR:{json}`。
8. B/C/D/A_FINISH 事件同时发送给无人机和地面站。

## 配置

只修改 `car_config.h`。

其中包括：

- Wi-Fi 名称和密码；
- UAV、GS IP 与端口；
- DHCP/静态 IP；
- 10 秒倒计时；
- T1/T2 每一段速度；
- 场地坐标和机械尺寸；
- 步进电机引脚与 3200 pulse/rev。

## IP

默认 `USE_DHCP=true`，小车不写死自己的监听 IP。

必须在路由器中按 ESP32 Wi-Fi MAC 地址保留：

- CAR：192.168.77.102
- UAV：192.168.77.103
- GS：192.168.77.104

原因是当前无人机 gateway 和地面站仍按配置的小车 IP 识别/发送。

## path_s_cm

实际车体中心初始位于 A 后方 30 cm。协议中的 `path_s_cm` 从 A 轨迹点开始：

- 起步前 30 cm：`path_s_cm=0`
- 中心通过 A 后开始增加；
- 到 B 为 150 cm；
- 与 UAV gateway 的 150 cm / 75 cm 轨迹模型一致。

另有调试字段 `route_s_cm`，包含最前面的 30 cm。

## 分段速度

在 `car_config.h` 中独立修改：

- `T1_SPEED_APPROACH_A_CM_S`
- `T1_SPEED_AB_CM_S`
- `T1_SPEED_BC_CM_S`
- `T1_SPEED_CD_CM_S`
- `T1_SPEED_DA_CM_S`

当前全部保持已验证的 16 cm/s。

## 按键

```text
GPIO32 ---- 常开按键 ---- GND
```

使用内部上拉。

测试阶段默认允许再次按下取消倒计时或停止。正式比赛若不需要，把：

```cpp
ALLOW_BUTTON_CANCEL_OR_STOP = false;
```

## 烧录

完整解压后，打开与文件夹同名的：

```text
AirGroundCar_ESP32_T1_Communication.ino
```

Arduino IDE 会自动编译同目录所有 `.cpp/.h`。
