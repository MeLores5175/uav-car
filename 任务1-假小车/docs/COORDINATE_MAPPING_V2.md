# D26 坐标映射 V2.0

## 统一定义

FIELD 坐标单位为 cm：左下角为 `(0,0)`，X 向右，Y 向上。

本测试地图使用：

- H `(75,75)`
- A `(150,200)`
- B `(150,350)`
- C `(300,350)`
- D `(300,200)`

MAVROS/Faster-LIO 在 ROS 侧按 ENU 使用，假小车 `/car/state` 的 `x/y/vx/vy/yaw` 均为相对 H 的 MAVROS local 坐标，单位 m、m/s、rad。

## 变换

`local_x_to_field_yaw_deg = theta` 表示 MAVROS local +X 轴在 FIELD 坐标中的航向。

local 到 FIELD：

```text
P_field = H_field + R(theta) * P_local_relative_home
```

FIELD 到 local：

```text
P_local_relative_home = R(-theta) * (P_field - H_field)
```

当前 theta=0 时：

- H local `(0,0)` -> FIELD `(75,75)`cm
- A FIELD `(150,200)`cm -> local `(0.75,1.25)`m
- A->B FIELD yaw 90° -> local yaw 90°

原来的 A local `(1.25,0.75)`m 把 X/Y 交换了，会让假小车目标轨道与地面站静态轨道不重合。

## theta 现场标定

飞机未解锁并确保 Faster-LIO 正常后，在 H 点记录一次 local pose。随后把机体沿 FIELD +X（场地向右）平移一段距离，得到 local 位移 `(dx,dy)`：

```text
theta_deg = -atan2(dy, dx) * 180/pi
```

例如：

- 向 FIELD +X 移动时 local +X 增大：theta=0°
- local +Y 增大：theta=-90°（或 270°）
- local -X 增大：theta=180°
- local -Y 增大：theta=90°

修改 `udp_protocol.field_transform.local_x_to_field_yaw_deg` 后，无需重新手算 A local；`coordinate_mode: field` 会自动转换整个假小车赛道。

## 起飞前检查

```bash
python3 /home/nvidia/catkin_ws/src/d26_air_ground_uav/scripts/check_track_mapping.py
```

必须输出 `RESULT: OK`，并打印 A/B/C/D 与上面的 FIELD 坐标一致。
