# 任务1无人机ROS前馈+PD及XY速度设定值发布设计

日期：2026-07-31

## 1. 目标

将任务1中截获、伴飞、投掷下降和投掷对准的水平控制，从“PX4位置环 + ROS速度前馈”改为“ROS前馈+PD位置外环计算完整速度，并向PX4仅发布XY速度设定值”。

本设计要实现：

- ROS限制完整的XY速度指令，而不只限制小车速度前馈；
- 截获、伴飞、下降和精对准使用同一套控制算法，但采用不同参数组；
- 不加入ROS位置积分项；
- 投掷任务链只保留这一条水平控制路径，不保留运行时模式切换；
- 保留当前垂直速度控制、固定航向、状态切换和投掷判据；
- 不修改真车代码；
- 暂不修改PX4参数。

## 2. 非目标

- 不修改小车控制或通信协议；
- 不重构视觉融合和小车状态估计器；
- 不调整PX4位置环、速度环或姿态环参数；
- 不将起飞、投掷后刹停抬升、返航和定点降落纳入本次移动目标控制改造；
- 不改变`FOLLOW_CD`、`DYNAMIC_DESCENT`、`PLATFORM_DISARM`和
  `PLATFORM_TAKEOFF`等动态降落专用状态的现有水平控制行为；
- 不引入MPC、轨迹优化器或新的第三方依赖；
- 不兼容缺少新参数组的旧YAML配置；
- 不承诺真实速度在任何扰动下绝对不瞬时超限。ROS只能硬限制其发送的完整速度设定值，真实速度仍受PX4、惯性、风和定位误差影响。

## 3. 控制架构

移动目标状态统一执行以下水平控制：

```text
小车预测位置/速度
        |
        v
目标点偏置与投放口偏置
        |
        v
位置误差、相对速度误差
        |
        v
小车速度前馈 + ROS位置PD修正
        |
        v
修正速度矢量限幅
        |
        v
完整速度矢量限幅
        |
        v
加速度与jerk限制
        |
        v
最终速度矢量二次限幅
        |
        v
MAVROS PositionTarget：仅XY速度有效
        |
        v
PX4速度环与姿态环
```

水平方向忽略`PositionTarget`中的PX、PY，只启用VX、VY。垂直方向继续忽略PZ并启用VZ，Yaw继续启用，XYZ加速度与Yaw Rate继续忽略。

## 4. 控制公式

定义：

```text
e_pos = target_position - uav_position
e_vel = target_velocity - uav_velocity

v_ff = Kff * target_velocity
v_corr_raw = Kp * e_pos + Kd * e_vel
```

不使用积分项。

先限制相对小车的额外修正：

```text
v_corr = limit_vector(v_corr_raw, correction_speed_limit)
```

再限制完整对地速度：

```text
v_desired = limit_vector(v_ff + v_corr, total_speed_limit)
```

然后使用现有二维加速度/jerk限制器生成`cmd_xy`，最后再次执行：

```text
cmd_xy = limit_vector(cmd_xy, total_speed_limit)
```

如果最终限幅改变了`cmd_xy`，同步修正内部`cmd_acc_xy`，避免内部加速度状态与实际发布速度不一致。

## 5. 为什么分开限制总速度和修正速度

精对准阶段不能简单把无人机对地速度降到很小。小车若以`0.60 m/s`行驶，无人机也必须维持约`0.60 m/s`才能保持在平台上方。

因此：

- `total_speed_limit`是无人机完整XY速度设定值的安全上限；
- `correction_speed_limit`限制无人机相对小车额外追赶或横移的强度；
- 越接近投放，主要降低修正速度、加速度和jerk，而不是切断小车速度前馈。

## 6. 状态参数组

配置中增加独立的移动目标控制参数组。所有数值都是首轮保守实飞起点，后续应依据日志调参。

| 参数组 | 状态 | Kff | Kp | Kd | 总速度 m/s | 修正速度 m/s | 加速度 m/s² | jerk m/s³ |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `intercept` | `INTERCEPT` | 1.00 | 0.95 | 0.30 | 0.75 | 0.35 | 0.55 | 0.90 |
| `follow` | `FOLLOW_DROP` | 1.00 | 0.75 | 0.25 | 0.75 | 0.25 | 0.45 | 0.75 |
| `drop_descent` | `DROP_DESCENT` | 1.00 | 0.65 | 0.25 | 0.75 | 0.18 | 0.30 | 0.55 |
| `drop_align` | `DROP_ALIGN`、`DROP_WAIT_ACK` | 1.00 | 0.55 | 0.30 | 0.75 | 0.12 | 0.22 | 0.40 |
初始版本保持各移动目标状态的完整速度上限为现有配置值`0.75 m/s`，不在未实飞验证前擅自提高截获速度。动作差异主要由修正速度、增益、加速度和jerk产生。

`descent_horizontal_speed_scale`不再缩放小车速度前馈，否则小车接近估计器允许的`0.65 m/s`时，无人机可能因基础跟车速度不足而持续落后。该字段由`drop_descent`参数组替代。

## 7. 状态切换

移动目标状态共用现有`cmd_xy`和`cmd_acc_xy`，状态切换时不直接清零，避免指令突变。

切换后立即使用新状态的速度、修正、加速度和jerk上限。命令通过加速度/jerk限制逐步进入新参数组允许的范围；最终速度二次限幅保证任何周期发布的XY速度设定值不超过当前状态总速度上限。

状态映射：

```text
INTERCEPT       -> intercept
FOLLOW_DROP     -> follow
DROP_DESCENT    -> drop_descent
DROP_ALIGN      -> drop_align
DROP_WAIT_ACK   -> drop_align
```

当前`POST_DROP_FOLLOW`名称虽然包含“FOLLOW”，实际代码行为是XY平滑刹停并在Z轴抬升，不再跟随小车。本次保持该状态原有逻辑，不把它误归入移动目标参数组。

## 8. 垂直控制与投掷逻辑

本次不改变垂直控制公式：

```text
vz_desired = clamp(Kp_z * z_error, -max_vz, max_vz)
```

继续对VZ执行垂直加速度限制。

下降暂停条件、视觉超时、投掷窗口、位置误差、相对速度、预测落点、稳定计时和投掷ACK重试逻辑均保持不变。它们读取的`position_error`和`relative_speed`定义保持不变。

## 9. 数据失效与安全行为

- 小车估计有效时正常运行前馈+PD控制；
- 视觉短时失效仍沿用当前估计器和状态机容错，不在本次修改中改变数据源选择；
- 下降阶段继续在跟踪误差或视觉条件不满足时暂停下降；
- ROS发布速度始终经过完整速度二次限幅；
- 增加日志指标：当前参数组、原始修正速度、限幅后修正速度、限幅前总速度、最终命令速度、实测无人机速度；
- 实测速度超过当前总速度上限时进行节流告警，但本次不新增突然置零或自动切状态，避免未经验证的激烈制动行为。

## 10. 配置结构与严格校验

配置结构固定为：

```yaml
follow:
  velocity_profiles:
    intercept:
      velocity_feedforward_gain: 1.00
      position_kp: 0.95
      relative_velocity_kd: 0.30
      max_horizontal_speed_mps: 0.75
      max_correction_speed_mps: 0.35
      max_horizontal_accel_mps2: 0.55
      max_horizontal_jerk_mps3: 0.90
    follow:
      velocity_feedforward_gain: 1.00
      position_kp: 0.75
      relative_velocity_kd: 0.25
      max_horizontal_speed_mps: 0.75
      max_correction_speed_mps: 0.25
      max_horizontal_accel_mps2: 0.45
      max_horizontal_jerk_mps3: 0.75
    drop_descent:
      velocity_feedforward_gain: 1.00
      position_kp: 0.65
      relative_velocity_kd: 0.25
      max_horizontal_speed_mps: 0.75
      max_correction_speed_mps: 0.18
      max_horizontal_accel_mps2: 0.30
      max_horizontal_jerk_mps3: 0.55
    drop_align:
      velocity_feedforward_gain: 1.00
      position_kp: 0.55
      relative_velocity_kd: 0.30
      max_horizontal_speed_mps: 0.75
      max_correction_speed_mps: 0.12
      max_horizontal_accel_mps2: 0.22
      max_horizontal_jerk_mps3: 0.40
```

删除投掷任务运行时模式选择及只供`legacy_outer_pd`使用的配置：

- `velocity_setpoint_mode`
- `position_kp`
- `relative_velocity_kd`
- `descent_horizontal_speed_scale`

全局`velocity_feedforward_gain`、`max_horizontal_speed_mps`、
`max_horizontal_accel_mps2`和`max_horizontal_jerk_mps3`继续保留，供起飞、
动态降落专用控制和其他现有路径使用。垂直控制、预测时间、稳定判据等其他配置继续保留。

节点初始化时必须检查四个参数组及每个必需字段：

- 字段缺失、不是有限数值或取值范围非法时，记录明确错误并拒绝进入任务；
- `Kff`、`Kp`、`Kd`不得为负；
- 总速度、修正速度、加速度和jerk必须大于零；
- 修正速度上限不得大于总速度上限；
- 不通过代码默认值静默补齐正式控制参数。

正式代码中不再保留`feedforward_only`和`legacy_outer_pd`运行时分支：

- `publish_drop_follow_control()`只服务`INTERCEPT`、`FOLLOW_DROP`、
  `DROP_DESCENT`、`DROP_ALIGN`和`DROP_WAIT_ACK`，始终忽略PX/PY并仅发布
  完整XY速度设定值；
- `publish_dynamic_follow_control()`只服务动态降落专用状态，固定保留当前
  “PX4位置环 + 小车速度前馈”行为，不提供YAML模式切换。

## 11. 验证

### 自动测试

1. 零误差时，命令等于受限的小车速度前馈；
2. 大位置误差时，修正速度不超过当前参数组上限；
3. 前馈与修正相加后，总速度不超过`total_speed_limit`；
4. 加速度/jerk更新后，最终发布速度仍不超过总速度上限；
5. 截获、伴飞、下降、精对准和投掷后状态选择正确参数组；
6. 发布消息时PX/PY被忽略，VX/VY/VZ有效；
7. 任意参数组或必需字段缺失时，配置校验失败并拒绝进入任务；
8. 非法参数值能够被配置校验拒绝；
9. 动态降落专用状态仍调用位置+速度控制函数且行为保持不变；
10. 垂直速度、投掷判据和状态切换逻辑保持回归一致；
11. Python语法检查通过。

### 实飞验证顺序

1. 桨叶拆除检查消息掩码与速度上限；
2. 静止小车低高度截获与悬停；
3. 小车低速直线伴飞；
4. 小车正常速度直线伴飞；
5. 半圆段方向变化测试；
6. CD段只下降不投掷；
7. 空载执行投掷舵机流程；
8. 最后进行真实投放。

每一步检查位置误差、相对速度、命令速度、实测速度和限幅触发频率。若长期频繁触发总速度上限，应先判断小车速度、预测延迟和截获距离，不直接提高增益。
