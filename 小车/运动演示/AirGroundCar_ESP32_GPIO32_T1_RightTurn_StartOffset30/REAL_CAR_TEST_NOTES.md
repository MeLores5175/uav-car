# 实车参数修改说明（2026-07-30）

## 已写入参数

- `STEPPER_PULSES_PER_REV = 3200.0f`
- `STEPPER_MAX_PULSE_HZ = 5000`
- LEFT 通道：STEP26 / DIR27 / EN25，前进 DIR=HIGH
- RIGHT 通道：STEP19 / DIR21 / EN18，前进 DIR=LOW
- 左右轮方向已改为独立配置。

## 首次测试建议

1. 先架空后轮烧录，串口确认出现：
   `pulses_per_rev=3200 max_pulse_hz=5000 left_forward_dir=HIGH right_forward_dir=LOW`
2. 通过地面站选择 T1、ARM、START 前，保证小车前方有足够空间。
3. 第一次落地测试建议不放无人机，先只验证小车路线和转弯方向。
4. 如果小车第一段不是沿 A→B 前进，立即断开电机电源并反馈实际运动方向。

## 未修改

- 路径模型、场地尺寸、速度规划、UDP 协议和无人机代码均未改动。
