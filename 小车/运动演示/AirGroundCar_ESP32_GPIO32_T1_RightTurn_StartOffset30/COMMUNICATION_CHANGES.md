# 真车通信修改

1. `TEL:CAR` 由只发地面站改为同时发送无人机和地面站。
2. 遥测频率由 5 Hz 改为 20 Hz。
3. `segment` 改为协议规定的 `AB/BC/CD/DA/UNKNOWN`。
4. 增加 `path_s_cm` 和 `line_detected` 必需字段。
5. 小车发送 UAV START 后等待 ACK，300 ms 间隔最多发送 10 次，同一命令号用于去重。
6. 能正确接收 UAV 的 `ACK/ERR`，不再把 ACK 当作错误格式返回。
7. 当前地址统一为：GS `.104`、UAV `.103`、CAR `.102`，网段 `192.168.77.0/24`。

注意：`STEPPER_PULSES_PER_REV` 当前仍为 0，必须完成步进电机每圈脉冲标定后才能真正输出运动脉冲。
