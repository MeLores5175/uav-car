# D26 无人机 BOOT 自启动安装说明

## 文件对应位置

把本补丁中的文件覆盖/复制到：

```text
~/catkin_ws/src/d26_air_ground_uav/
├── scripts/
│   ├── d26_boot_launcher.py       ← 新增
│   └── uav_udp_gateway.py         ← 覆盖
└── launch/
    └── d26_task1_real.launch      ← 覆盖
```

systemd 文件：

```text
systemd/d26-uav-boot.service
```

复制到：

```text
/etc/systemd/system/d26-uav-boot.service
```

## 正式通信流程

```text
无人机上电
→ systemd启动d26_boot_launcher.py
→ 监听0.0.0.0:8888

GS → UAV：
CMD:<id>:BOOT:T1

BOOT启动器 → GS：
ACK:<id>:BOOT:ACCEPTED:T1

BOOT启动器释放8888
→ roslaunch d26_air_ground_uav d26_task1_real.launch
→ gateway接管8888
→ FCU、定位、FSM全部READY后：
EVT:UAV_BOOT_READY:T1
```

BOOT不会让无人机起飞。小车实体按键倒计时结束后发送START，才会触发任务起飞。

## 关键固定IP

```text
小车：   192.168.77.102:8890
无人机： 192.168.77.103:8888
地面站： 192.168.77.104:8889
```
