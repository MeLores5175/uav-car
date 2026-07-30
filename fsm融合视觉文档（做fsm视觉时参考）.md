# FSM 与 Launch 接入说明：YOLO + Tag25h9 融合视觉

## 1. 给 FSM 负责人的结论

正式状态机只消费一个融合结果话题：

```text
/uav/platform_vision        std_msgs/String(JSON)
```

不要让 FSM 分别订阅 YOLO 和 AprilTag 原始话题。两种视觉来源的切换、短时保持、重捕获和质量判断均由 `platform_vision_node.py` 完成。

FSM 需要保留已有接口：

```text
发布：/uav/platform_scan_enable    std_msgs/Bool
订阅：/uav/platform_vision         std_msgs/String
```

并新增一个推荐接口：

```text
发布：/uav/platform_vision_mode    std_msgs/String
```

模式值只有：

```text
OFF
SEARCH
TRACK
PRECISION
```

节点同时订阅 `/uav/fsm_state`，因此短期内即使没有添加模式发布器，也能自动根据 FSM 状态推导模式；正式版本仍建议 FSM 显式发布模式。

---

## 2. 状态与视觉模式映射

| FSM 状态 | 发布模式 | 用途 |
|---|---|---|
| `WAIT_START` | `OFF` | 等待任务 |
| `WAIT_FCU` | `OFF` | 飞控准备 |
| `TAKEOFF` | `OFF` | 起飞 |
| `DROP_HOVER` | `OFF` | 任务一固定悬停 |
| `INTERCEPT` | `SEARCH` | YOLO 远距离搜索与截获 |
| `FOLLOW_DROP` | `TRACK` | 伴飞，Tag 优先，YOLO 重捕获 |
| `POST_DROP_FOLLOW` | `TRACK` | 投放后短时伴飞 |
| `FOLLOW_CD` | `TRACK` | 等待进入 C-D 段 |
| `DROP_DESCENT` | `PRECISION` | 跟车下降 |
| `DROP_ALIGN` | `PRECISION` | 投放精确对准 |
| `DROP_WAIT_ACK` | `PRECISION` | 投放机构动作期间保持锁定 |
| `DYNAMIC_DESCENT` | `PRECISION` | 动态降落 |
| `PLATFORM_DISARM` | `PRECISION` | 触地确认 |
| `PLATFORM_DWELL` | `OFF` | 平台停留 |
| `PLATFORM_TAKEOFF` | `TRACK` | 随车重新起飞 |
| `RETURN_HOME` | `OFF` | 返航 |
| `HOME_LAND` | `OFF` | H 点降落 |
| `EMERGENCY_LAND` | `OFF` | 应急优先 |
| `WAIT_RESET` | `OFF` | 等待复位 |

建议在 FSM 主循环中同时发布：

```python
self.platform_scan_pub.publish(
    Bool(data=self.fsm_state in self.VISION_STATES)
)
self.platform_vision_mode_pub.publish(
    String(data=self.vision_mode_for_state(self.fsm_state))
)
```

新增发布器：

```python
self.platform_vision_mode_pub = rospy.Publisher(
    "/uav/platform_vision_mode",
    String,
    queue_size=5,
    latch=True,
)
```

映射函数：

```python
@staticmethod
def vision_mode_for_state(state):
    search_states = {"INTERCEPT"}

    track_states = {
        "FOLLOW_DROP",
        "POST_DROP_FOLLOW",
        "FOLLOW_CD",
        "PLATFORM_TAKEOFF",
    }

    precision_states = {
        "DROP_DESCENT",
        "DROP_ALIGN",
        "DROP_WAIT_ACK",
        "DYNAMIC_DESCENT",
        "PLATFORM_DISARM",
    }

    if state in precision_states:
        return "PRECISION"
    if state in track_states:
        return "TRACK"
    if state in search_states:
        return "SEARCH"
    return "OFF"
```

---

## 3. `/uav/platform_vision` 统一消息

消息类型暂时继续使用 `std_msgs/String`，内容为 JSON。

典型 AprilTag 精修结果：

```json
{
  "target": "landing_platform",
  "stamp": 1785400000.123,
  "publish_stamp": 1785400000.151,
  "frame_id": 1368,
  "mode": "PRECISION",

  "detected": true,
  "stable": true,
  "stable_count": 8,

  "source": "APRILTAG",
  "quality": "HIGH",
  "confidence": 0.93,

  "center_u": 316.4,
  "center_v": 243.1,
  "forward_m": -0.007,
  "left_m": 0.009,

  "target_x_local": 2.316,
  "target_y_local": 1.428,
  "local_position_valid": true,

  "pose_time_offset_s": 0.012,
  "measurement_age_s": 0.028,
  "fusion_residual_m": 0.06,

  "yolo_detected": true,
  "yolo_confidence": 0.87,

  "tag_detected": true,
  "tag_family": "tag25h9",
  "tag_id": 0,
  "tag_decision_margin": 76.3,
  "tag_hamming": 0,
  "tag_size_px": 84.2,

  "offset_applied": false
}
```

`source` 的可能值：

```text
APRILTAG   Tag25h9 当前有效
YOLO       大靶标检测当前有效
PREDICTED  Tag 短时丢失，使用内部短时预测
NONE       没有可用测量
```

第一版不输出 `FUSED`，因为 Tag 目前不在平台中心，直接平均 YOLO 中心和 Tag 中心会产生人为跳变。

---

## 4. FSM 回调建议

现有回调可以继续保留：

```python
def platform_vision_cb(self, msg):
    try:
        data = json.loads(msg.data)
    except Exception as exc:
        rospy.logwarn_throttle(
            1.0,
            "Invalid /uav/platform_vision JSON: %s",
            str(exc),
        )
        return

    if str(data.get("target", "")).lower() != "landing_platform":
        return

    self.vision_data = data
    self.vision_rx_time = rospy.Time.now()
    self.vision_version += 1
```

建议新增对消息时间戳的检查，不只使用“ROS 接收时间”：

```python
def vision_measurement_age(self):
    if not self.vision_data:
        return float("inf")

    stamp = safe_float(self.vision_data.get("stamp", 0.0), 0.0)
    if stamp <= 0.0:
        return self.vision_age()

    return max(0.0, rospy.Time.now().to_sec() - stamp)
```

建议新版有效性判断：

```python
def vision_valid(self):
    if not self.vision_data:
        return False

    if self.vision_measurement_age() > self.vision_timeout:
        return False

    if not bool(self.vision_data.get("detected", False)):
        return False

    source = str(self.vision_data.get("source", "NONE")).upper()
    confidence = safe_float(self.vision_data.get("confidence", 0.0))
    stable_count = safe_int(self.vision_data.get("stable_count", 0))

    if source == "PREDICTED":
        return self.vision_measurement_age() <= 0.15

    return (
        source in {"YOLO", "APRILTAG"}
        and confidence >= self.vision_min_confidence
        and stable_count >= self.vision_min_stable_count
    )
```

---

## 5. 优先使用同步后的 local 目标位置

旧版 FSM 根据收到消息时的当前 UAV 位姿，将 `forward_m/left_m` 转成 local 坐标。移动飞行时会引入推理延迟误差。

新版视觉节点已经保存位姿历史，并用“图像采集时间”附近的 UAV 位姿计算：

```text
target_x_local
target_y_local
```

FSM 应优先使用这两个字段：

```python
def vision_abs_measurement(self):
    if not self.vision_valid():
        return None

    if bool(self.vision_data.get("local_position_valid", False)):
        x = safe_float(self.vision_data.get("target_x_local", 0.0))
        y = safe_float(self.vision_data.get("target_y_local", 0.0))
    else:
        # 兼容旧视觉消息
        if self.current_pose is None:
            return None

        forward = safe_float(self.vision_data.get("forward_m", 0.0))
        left = safe_float(self.vision_data.get("left_m", 0.0))

        dx = (
            math.cos(self.current_yaw) * forward
            - math.sin(self.current_yaw) * left
        )
        dy = (
            math.sin(self.current_yaw) * forward
            + math.cos(self.current_yaw) * left
        )

        p = self.current_pose.pose.position
        x = p.x + dx
        y = p.y + dy

    return {
        "x": x,
        "y": y,
        "source": str(self.vision_data.get("source", "NONE")).upper(),
        "confidence": safe_float(
            self.vision_data.get("confidence", 0.0)
        ),
        "stable_count": safe_int(
            self.vision_data.get("stable_count", 0)
        ),
        "residual": safe_float(
            self.vision_data.get("fusion_residual_m", 0.0)
        ),
        "age": self.vision_measurement_age(),
    }
```

---

## 6. FSM 融合权重

保留原有 Alpha-Beta 小车跟踪器，但不同来源采用不同视觉修正权重。

推荐 YAML：

```yaml
mission:
  estimator:
    vision_timeout_s: 0.25

    yolo_position_alpha: 0.35
    apriltag_position_alpha: 0.78

    yolo_residual_gate_m: 0.50
    apriltag_residual_gate_m: 0.35

    precision_require_apriltag: true
    precision_tag_min_stable_count: 3
```

权重函数：

```python
def visual_alpha(self, vision):
    source = vision["source"]
    confidence = clamp(vision["confidence"], 0.0, 1.0)

    if source == "APRILTAG":
        base = self.apriltag_position_alpha
    elif source == "YOLO":
        base = self.yolo_position_alpha
    else:
        return 0.0

    return base * clamp(confidence, 0.40, 1.0)
```

残差门控：

```python
def visual_gate(self, vision):
    source = vision["source"]
    residual = vision["residual"]

    if source == "APRILTAG":
        return residual <= self.apriltag_residual_gate_m
    if source == "YOLO":
        return residual <= self.yolo_residual_gate_m
    return False
```

`PREDICTED` 不应再次修正 Alpha-Beta 跟踪器：

```python
if vision["source"] == "PREDICTED":
    # 只能维持短时控制，不当成新的观测
    self.last_processed_vision_version = self.vision_version
```

---

## 7. 各阶段的视觉授权规则

### `INTERCEPT`

可接受：

```text
source = YOLO 或 APRILTAG
detected = true
stable_count >= 2
measurement_age_s < 0.20
```

YOLO 的职责是发现小车和把目标重新拉回预测位置附近。

### `FOLLOW_DROP` / `FOLLOW_CD`

可接受 YOLO 或 AprilTag。控制仍采用：

```text
小车遥测速度 → 速度前馈
融合目标位置 → PX4 位置目标
视觉 → 修正小车遥测/轨迹模型误差
```

### `DROP_DESCENT`

高空阶段可由 YOLO 或 AprilTag 保持 XY 跟随。

进入低空精修前，建议要求：

```text
source = APRILTAG
stable_count >= 3
tag_hamming = 0
```

Tag 丢失时：

```text
继续 XY 跟随
暂停 Z 下降
等待 Tag 重捕获
```

### `DROP_ALIGN`

第一版直接对准 Tag 中心。必须满足：

```text
source = APRILTAG
stable = true
tag_hamming = 0
measurement_age_s < 0.15
水平误差满足投放阈值
相对速度满足投放阈值
```

### `DYNAMIC_DESCENT`

较高高度：

```text
YOLO 或 AprilTag 可用于 XY 跟随
```

低空阶段：

```text
必须 AprilTag 稳定，只有 YOLO 时停止继续下降
```

最终触地继续结合：

```text
AprilTag / 测距 / 垂直速度 / landed state
```

### `PREDICTED`

只允许非常短时间保持：

```text
不开始新的下降
不触发投放
不确认触地
```

---

## 8. Tag 偏置的后续修改

当前配置：

```yaml
apply_tag_offset: false
tag_to_platform_forward_m: 0.0
tag_to_platform_left_m: 0.0
```

当前第一版把 Tag 中心近似为平台中心。

后续需要测量：

```text
Tag 中心到平台中心的距离
该偏置在小车/Tag 坐标中的方向
Tag 图案方向与小车前进方向之间的固定角度
```

偏置应当随小车航向或 Tag 平面角旋转，不能简单永久给 `forward_m` 加一个固定值。未完成方向标定前不要启用偏置。

---

## 9. Launch 接入

文件放置：

```text
scripts/platform_vision_node.py
launch/platform_vision.launch
config/platform_vision.yaml
```

权限：

```bash
chmod +x ~/catkin_ws/src/d26_air_ground_uav/scripts/platform_vision_node.py
```

编译：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

单独启动视觉：

```bash
roslaunch d26_air_ground_uav platform_vision.launch
```

任务总 launch 中加入：

```xml
<include file="$(find d26_air_ground_uav)/launch/platform_vision.launch"/>
```

正式运行时必须关闭：

```text
test_targer_engine_camera.py
test_tag25h9_camera.py
其他打开 /dev/video0 的程序
```

否则视觉节点无法独占下视相机。

---

## 10. 联调检查命令

```bash
rostopic echo /uav/platform_vision/status
rostopic echo /uav/platform_vision
rostopic echo /uav/platform_vision/yolo
rostopic echo /uav/platform_vision/apriltag
```

手动开启：

```bash
rostopic pub -1 /uav/platform_scan_enable std_msgs/Bool "data: true"
rostopic pub -1 /uav/platform_vision_mode std_msgs/String "data: 'SEARCH'"
```

切换精修：

```bash
rostopic pub -1 /uav/platform_vision_mode std_msgs/String "data: 'PRECISION'"
```

关闭：

```bash
rostopic pub -1 /uav/platform_scan_enable std_msgs/Bool "data: false"
```
