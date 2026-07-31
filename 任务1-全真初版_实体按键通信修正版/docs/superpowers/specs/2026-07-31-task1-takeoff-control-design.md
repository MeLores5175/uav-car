# 任务1无人机独立起飞控制与悬停退出门槛设计

日期：2026-07-31

## 1. 目标

将任务1起飞和起飞后固定悬停所使用的水平、垂直控制参数从
`mission.follow`中分离，统一放入`mission.takeoff`；降低起飞过程中过大的
水平修正速度，同时采用更快的垂直上升参数。

固定悬停3秒的计时规则保持不变，但计时完成后必须满足一组宽松安全条件，
才能进入截获。安全条件不满足时继续悬停，不清零或重新开始3秒计时。

## 2. 修改范围

本次只修改：

- `TAKEOFF`阶段的定点速度控制参数来源；
- `DROP_HOVER`阶段的定点速度控制参数来源；
- `DROP_HOVER`计时完成后的退出条件；
- 定点控制函数对独立垂直比例增益的支持；
- 对应YAML配置和自动测试。

本次不修改：

- `WAIT_FCU`的OFFBOARD设定值预发送方式；
- 起飞8秒软超时判定及应急降落逻辑；
- 截获、伴飞、投掷下降和投掷对准控制；
- 动态降落专用控制；
- PX4参数；
- 实测超速告警或主动超速保护；
- 真车代码。

## 3. 独立起飞参数

`mission.takeoff`增加：

```yaml
takeoff:
  horizontal_position_kp: 0.80
  max_horizontal_speed_mps: 0.35
  max_horizontal_accel_mps2: 0.35
  max_horizontal_jerk_mps3: 0.60

  vertical_position_kp: 0.90
  max_vertical_speed_mps: 0.55
  max_vertical_accel_mps2: 0.75

  hover_exit_xy_tolerance_m: 0.30
  hover_exit_z_tolerance_m: 0.18
  hover_exit_speed_tolerance_mps: 0.35
```

现有起飞高度、首次到位判据、固定悬停时间和软超时参数继续保留。

## 4. 控制方式

`TAKEOFF`和`DROP_HOVER`使用完全相同的定点速度控制参数。

水平方向：

```text
e_xy = home_xy - uav_xy
v_xy_desired = horizontal_position_kp * e_xy
|v_xy_desired| <= max_horizontal_speed_mps
```

随后继续使用现有二维加速度和jerk限制器：

```text
|a_xy| <= 0.35 m/s²
|jerk_xy| <= 0.60 m/s³
```

垂直方向：

```text
e_z = target_z - uav_z
vz_desired = vertical_position_kp * e_z
|vz_desired| <= 0.55 m/s
```

垂直速度命令继续使用现有加速度限制：

```text
|az| <= 0.75 m/s²
```

最终仍只向PX4发布XYZ速度设定值和固定Yaw，不发送XYZ位置设定值。

## 5. 代码结构

扩展`publish_point_control()`，增加可选参数：

```python
vertical_kp=None
```

行为：

- 调用方显式提供`vertical_kp`时，使用调用方值；
- 其他现有调用不提供时，继续读取当前`mission.follow.vertical_kp`；
- 返航、定点降落等现有调用行为不变。

增加一个起飞专用包装函数：

```python
publish_takeoff_point_control(dt)
```

该函数固定目标为：

```text
x = home_x
y = home_y
z = home_z + cruise_height
yaw = fixed_yaw
```

并从`takeoff_cfg`读取全部水平、垂直起飞参数。`handle_takeoff()`和
`handle_drop_hover()`共用该函数，避免两处参数不一致。

## 6. 起飞完成与固定悬停

首次进入`DROP_HOVER`的规则保持当前行为：

```text
XY误差 <= 0.18 m
高度误差 <= 0.10 m
三维实测速度 <= 0.20 m/s
```

任务1第一次满足上述条件就进入`DROP_HOVER`并开始固定3秒计时。

8秒软超时逻辑保持原样：软超时只检查当前既有的高度误差条件，本次不调整。

## 7. 悬停退出安全门槛

`DROP_HOVER`每个控制周期调用`publish_takeoff_point_control(dt)`，获取：

- 当前XY误差；
- 当前高度误差；
- 当前三维实测速度。

状态转换规则：

```text
state_elapsed < 3.0 s
    -> 继续悬停

state_elapsed >= 3.0 s 且退出安全条件满足
    -> 进入 INTERCEPT

state_elapsed >= 3.0 s 但退出安全条件不满足
    -> 继续悬停，不重置state_enter_time，不重新计时
```

宽松退出安全条件：

```text
XY误差 <= 0.30 m
高度误差 <= 0.18 m
三维实测速度 <= 0.35 m/s
```

计时完成但条件不满足时，每秒最多输出一次节流日志，记录XY误差、高度误差和
三维速度。该日志只说明“尚未允许进入截获”，不是实测超速告警。

一旦条件恢复，无需再等待3秒，下一个控制周期立即进入`INTERCEPT`。

## 8. 配置与兼容

正式YAML明确写入全部新参数。代码读取采用现有`safe_float`模式，并使用本设计
数值作为默认值。

`mission.follow`中的全局水平和垂直参数继续保留，因为动态降落、起飞之外的
现有定点控制路径仍可能使用；但`TAKEOFF`和`DROP_HOVER`不得再读取这些参数。

## 9. 测试

自动测试覆盖：

1. 起飞专用控制读取`takeoff_cfg`中的水平Kp、速度、加速度和jerk；
2. 起飞专用控制读取快速档垂直Kp、最大速度和加速度；
3. 修改`follow`水平/垂直参数不会改变起飞专用控制参数；
4. `DROP_HOVER`不足3秒时不进入截获；
5. 达到3秒但宽松安全条件不满足时继续悬停；
6. 等待过程中不重置状态进入时间；
7. 3秒后条件恢复时立即进入截获；
8. 起飞8秒软超时分支保持存在且判定字段不变；
9. 截获、投掷控制和动态降落测试继续通过；
10. YAML真实解析、Python语法检查和完整单元测试通过。

## 10. 实飞验证建议

1. 桨叶拆除检查起飞阶段发布消息仍为XYZ速度有效、XYZ位置忽略；
2. 低高度测试最大水平指令不超过`0.35 m/s`；
3. 检查最大上升指令不超过`0.55 m/s`；
4. 检查垂直速度从0按`0.75 m/s²`限制平滑增加；
5. 人为轻推或制造小幅偏差，确认3秒结束时不安全会继续悬停；
6. 条件恢复后确认不会重新等待3秒，而是立即进入截获。

