#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
2026 D题陆空协同无人机任务状态机。

支持两种任务：
1. drop：起飞、悬停3秒、截获小车、伴飞、仅在C-D直线段投放、返回H点降落；
2. dynamic_land：起飞、截获小车、伴飞、仅在C-D直线段连续下降并着陆、
   平台停留5秒、随车重新起飞、返回H点降落。

设计来源于原微型无人机项目：保留 MAVROS setpoint 封装、安全状态优先级、
速度前馈、加速度限幅、视觉超时、单次动作与 ACK 机制；重写为移动目标控制。
"""

import json
import math
import os
from copy import deepcopy

import rospy
import yaml

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import euler_from_quaternion


def clamp(value, low, high):
    return max(low, min(high, value))


def norm2(x, y):
    return math.sqrt(x * x + y * y)


def norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def limit_xy(x, y, maximum):
    length = norm2(x, y)
    if length <= maximum or length < 1e-9:
        return x, y
    scale = maximum / length
    return x * scale, y * scale


def limit_xy_change(old_xy, new_xy, max_delta):
    dx = new_xy[0] - old_xy[0]
    dy = new_xy[1] - old_xy[1]
    d = norm2(dx, dy)
    if d <= max_delta or d < 1e-9:
        return [new_xy[0], new_xy[1]]
    scale = max_delta / d
    return [old_xy[0] + dx * scale, old_xy[1] + dy * scale]


def safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


def safe_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return int(default)


class AlphaBetaTracker:
    """二维匀速目标跟踪器，融合小车里程位置与下视视觉位置。"""

    def __init__(self):
        self.initialized = False
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_time = None

    def reset(self):
        self.__init__()

    def initialize(self, x, y, vx, vy, now_sec):
        self.initialized = True
        self.x = float(x)
        self.y = float(y)
        self.vx = float(vx)
        self.vy = float(vy)
        self.last_time = float(now_sec)

    def predict_to(self, now_sec):
        if not self.initialized:
            return
        dt = clamp(float(now_sec) - self.last_time, 0.0, 0.25)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.last_time = float(now_sec)

    def correct_position(self, mx, my, alpha, beta, dt_hint=0.02):
        if not self.initialized:
            return
        rx = float(mx) - self.x
        ry = float(my) - self.y
        a = clamp(float(alpha), 0.0, 1.0)
        b = clamp(float(beta), 0.0, 1.0)
        dt = max(float(dt_hint), 0.02)
        self.x += a * rx
        self.y += a * ry
        self.vx += b * rx / dt
        self.vy += b * ry / dt

    def correct_velocity(self, mvx, mvy, alpha):
        if not self.initialized:
            return
        a = clamp(float(alpha), 0.0, 1.0)
        self.vx = (1.0 - a) * self.vx + a * float(mvx)
        self.vy = (1.0 - a) * self.vy + a * float(mvy)

    def snapshot(self, prediction_s=0.0):
        if not self.initialized:
            return None
        p = max(0.0, float(prediction_s))
        return {
            "x": self.x + self.vx * p,
            "y": self.y + self.vy * p,
            "vx": self.vx,
            "vy": self.vy,
        }


class AirGroundMissionFSM:
    ACTIVE_OFFBOARD_STATES = {
        "TAKEOFF", "DROP_HOVER", "INTERCEPT", "FOLLOW_DROP", "DROP_ALIGN",
        "DROP_WAIT_ACK", "POST_DROP_FOLLOW", "FOLLOW_CD", "DYNAMIC_DESCENT",
        "PLATFORM_DISARM", "PLATFORM_TAKEOFF", "RETURN_HOME"
    }

    VISION_STATES = {
        "INTERCEPT", "FOLLOW_DROP", "DROP_ALIGN", "DROP_WAIT_ACK",
        "POST_DROP_FOLLOW", "FOLLOW_CD", "DYNAMIC_DESCENT",
        "PLATFORM_DISARM", "PLATFORM_DWELL", "PLATFORM_TAKEOFF"
    }

    def __init__(self):
        rospy.init_node("air_ground_mission_fsm", anonymous=False)

        default_yaml = os.path.expanduser(
            "~/catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"
        )
        self.config_path = rospy.get_param("~mission_yaml", default_yaml)
        with open(self.config_path, "r", encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)

        self.mission_cfg = self.cfg.get("mission", {})
        self.takeoff_cfg = self.mission_cfg.get("takeoff", {})
        self.fcu_cfg = self.mission_cfg.get("fcu", {})
        self.follow_cfg = self.mission_cfg.get("follow", {})
        self.est_cfg = self.mission_cfg.get("estimator", {})
        self.drop_cfg = self.mission_cfg.get("drop", {})
        self.land_cfg = self.mission_cfg.get("dynamic_landing", {})
        self.return_cfg = self.mission_cfg.get("return_home", {})
        self.safety_cfg = self.mission_cfg.get("safety", {})

        self.rate_hz = safe_float(self.mission_cfg.get("control_rate_hz", 50.0), 50.0)
        self.status_rate_hz = safe_float(self.mission_cfg.get("status_rate_hz", 10.0), 10.0)
        self.frame_mode = str(self.mission_cfg.get("frame", "relative_home")).strip().lower()
        self.default_mission_type = self.normalize_mission_type(
            self.mission_cfg.get("default_type", "drop")
        )

        self.auto_set_mode = bool(self.fcu_cfg.get("auto_set_mode", True))
        self.auto_arm = bool(self.fcu_cfg.get("auto_arm", True))
        self.offboard_prestream_s = safe_float(self.fcu_cfg.get("offboard_prestream_s", 1.0), 1.0)
        self.mode_request_interval_s = safe_float(
            self.fcu_cfg.get("mode_request_interval_s", 1.0), 1.0
        )
        self.arm_request_interval_s = safe_float(
            self.fcu_cfg.get("arm_request_interval_s", 1.0), 1.0
        )

        self.cruise_height = safe_float(self.takeoff_cfg.get("cruise_height_m", 1.50), 1.50)
        self.takeoff_yaw = math.radians(safe_float(self.takeoff_cfg.get("yaw_deg", 0.0), 0.0))

        self.telemetry_timeout = safe_float(self.est_cfg.get("telemetry_timeout_s", 0.50), 0.50)
        self.vision_timeout = safe_float(self.est_cfg.get("vision_timeout_s", 0.35), 0.35)
        self.odom_timeout = safe_float(self.est_cfg.get("odometry_timeout_s", 0.40), 0.40)
        self.telemetry_position_alpha = safe_float(
            self.est_cfg.get("telemetry_position_alpha", 0.45), 0.45
        )
        self.telemetry_velocity_alpha = safe_float(
            self.est_cfg.get("telemetry_velocity_alpha", 0.65), 0.65
        )
        self.vision_position_alpha = safe_float(
            self.est_cfg.get("vision_position_alpha", 0.72), 0.72
        )
        self.position_beta = safe_float(self.est_cfg.get("position_beta", 0.015), 0.015)
        self.max_car_speed = safe_float(self.est_cfg.get("max_car_speed_mps", 0.45), 0.45)
        self.max_prediction_s = safe_float(self.est_cfg.get("max_prediction_s", 0.70), 0.70)
        self.vision_min_confidence = safe_float(
            self.est_cfg.get("vision_min_confidence", 0.55), 0.55
        )
        self.vision_min_stable_count = safe_int(
            self.est_cfg.get("vision_min_stable_count", 2), 2
        )
        self.car_state_is_relative_home = bool(
            self.est_cfg.get("car_state_is_relative_home", True)
        )

        self.current_state = State()
        self.extended_state = ExtendedState()
        self.extended_state_received = False
        self.current_pose = None
        self.current_vel = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        self.last_pose_rx = None
        self.last_vel_rx = None
        self.range_m = None
        self.last_range_rx = None

        self.home_ready = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.home_yaw = 0.0

        self.car_data = {}
        self.car_rx_time = None
        self.car_version = 0
        self.last_processed_car_version = -1
        self.vision_data = {}
        self.vision_rx_time = None
        self.vision_version = 0
        self.last_processed_vision_version = -1
        self.tracker = AlphaBetaTracker()
        self.last_estimator_time = None
        self.car_estimate = None

        self.pending_mission_type = self.default_mission_type
        self.mission_type = self.default_mission_type
        self.mission_id = 0
        self.run_id = "R000"
        self.mission_start_time = None
        self.mission_result = "IDLE"
        self.start_requested = False
        self.reset_required = False

        self.fsm_state = "WAIT_START"
        self.state_enter_time = rospy.Time.now()
        self.stable_since = None
        self.follow_stable_since = None
        self.drop_stable_since = None
        self.touchdown_since = None
        self.home_stable_since = None
        self.takeoff_stable_since = None
        # 任务一：首次满足起飞位置/速度判稳条件时，立即进入3秒悬停计时。
        # 进入 DROP_HOVER 后不再要求稳定条件持续成立，3秒到点即开始截获。
        self.drop_hover_started_by_stable = False

        # 最近一次移动目标控制指标，供地面站显示相对位置/相对速度误差。
        self.last_follow_metrics = None
        self.last_follow_metrics_time = None

        self.last_mode_request = rospy.Time(0)
        self.last_arm_request = rospy.Time(0)
        self.wait_fcu_prestream_start = None
        self.auto_land_start = None
        self.auto_land_prepare_start = None

        self.cmd_xy = [0.0, 0.0]
        self.cmd_vz = 0.0
        self.commanded_yaw = self.takeoff_yaw
        self.last_control_time = rospy.Time.now()

        self.landing_target_z = None
        self.landing_attempts = 0
        self.dwell_start = None
        self.platform_takeoff_prestream_start = None

        self.drop_action_id = 0
        self.drop_send_time = None
        self.drop_retry_count = 0
        self.drop_ack_success = False
        self.drop_ack_text = ""
        self.drop_done_time = None

        self.safety_state = "IDLE"
        self.abort_reason = ""
        self.expected_disarm = False
        self.emitted_events = set()

        self.last_status_publish = rospy.Time(0)

        self.raw_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=30
        )
        self.fsm_state_pub = rospy.Publisher("/uav/fsm_state", String, queue_size=10, latch=True)
        self.safety_state_pub = rospy.Publisher(
            "/uav/safety_state", String, queue_size=10, latch=True
        )
        self.mission_status_pub = rospy.Publisher(
            "/uav/mission_status", String, queue_size=20
        )
        self.platform_scan_pub = rospy.Publisher(
            "/uav/platform_scan_enable", Bool, queue_size=5
        )
        self.drop_cmd_pub = rospy.Publisher("/uav/drop_cmd", String, queue_size=10)
        self.mission_event_pub = rospy.Publisher(
            "/uav/mission_event", String, queue_size=20
        )
        self.command_result_pub = rospy.Publisher(
            "/uav/mission_command_result", String, queue_size=20
        )

        rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=20)
        rospy.Subscriber(
            "/mavros/extended_state", ExtendedState, self.extended_state_cb, queue_size=20
        )
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=30
        )
        rospy.Subscriber(
            "/mavros/local_position/velocity_local", TwistStamped, self.velocity_cb, queue_size=30
        )
        rospy.Subscriber("/car/state", String, self.car_state_cb, queue_size=50)
        rospy.Subscriber(
            "/uav/platform_vision", String, self.platform_vision_cb, queue_size=30
        )
        rospy.Subscriber("/uav/start", Bool, self.start_cb, queue_size=10)
        rospy.Subscriber(
            "/uav/mission_command", String, self.mission_command_cb, queue_size=10
        )
        rospy.Subscriber("/uav/mission_type", String, self.mission_type_cb, queue_size=10)
        rospy.Subscriber("/uav/stop", Bool, self.stop_cb, queue_size=10)
        rospy.Subscriber("/uav/land", Bool, self.land_cb, queue_size=10)
        rospy.Subscriber("/uav/disarm", Bool, self.disarm_cb, queue_size=10)
        rospy.Subscriber("/uav/reset", Bool, self.reset_cb, queue_size=10)
        rospy.Subscriber("/uav/drop_ack", String, self.drop_ack_cb, queue_size=10)

        range_topic = str(
            self.land_cfg.get("range_topic", "/mavros/distance_sensor/rangefinder_pub")
        )
        rospy.Subscriber(range_topic, Range, self.range_cb, queue_size=20)

        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        wait_s = safe_float(self.fcu_cfg.get("service_wait_s", 8.0), 8.0)
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=wait_s)
            rospy.wait_for_service("/mavros/cmd/arming", timeout=wait_s)
        except rospy.ROSException:
            rospy.logwarn("MAVROS services not ready yet; FSM will keep retrying after launch.")

        rospy.logwarn(
            "D26 air-ground FSM ready. yaml=%s default_task=%s dynamic landing window=C-D only",
            self.config_path,
            self.default_mission_type,
        )

    @staticmethod
    def normalize_mission_type(value):
        text = str(value).strip().lower()
        aliases = {
            "1": "drop", "t1": "drop", "task1": "drop", "drop": "drop", "投放": "drop",
            "2": "dynamic_land", "t2": "dynamic_land", "task2": "dynamic_land",
            "dynamic_land": "dynamic_land", "动态起降": "dynamic_land",
        }
        return aliases.get(text, "drop")

    def state_cb(self, msg):
        previous_armed = bool(self.current_state.armed)
        previous_mode = str(self.current_state.mode)
        self.current_state = msg

        if (
            previous_armed and not msg.armed and not self.expected_disarm and
            self.fsm_state in self.ACTIVE_OFFBOARD_STATES and
            self.fsm_state not in {"PLATFORM_DISARM"}
        ):
            self.abort_reason = "unexpected disarm during %s" % self.fsm_state
            self.mission_result = "ABORTED_UNEXPECTED_DISARM"
            self.reset_required = True
            self.start_requested = False
            self.enter_state("WAIT_RESET")
            return

        if (
            self.fsm_state in self.ACTIVE_OFFBOARD_STATES and
            previous_mode == "OFFBOARD" and msg.mode not in {"OFFBOARD", "AUTO.LAND"} and
            msg.armed
        ):
            self.request_emergency_land("OFFBOARD lost: %s" % msg.mode)

    def extended_state_cb(self, msg):
        self.extended_state = msg
        self.extended_state_received = True

    def pose_cb(self, msg):
        self.current_pose = msg
        self.last_pose_rx = rospy.Time.now()
        q = msg.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self.home_ready:
            self.home_x = msg.pose.position.x
            self.home_y = msg.pose.position.y
            self.home_z = msg.pose.position.z
            self.home_yaw = self.current_yaw
            self.home_ready = True
            self.commanded_yaw = self.takeoff_yaw
            rospy.logwarn(
                "Home captured: x=%.3f y=%.3f z=%.3f yaw=%.1fdeg",
                self.home_x, self.home_y, self.home_z, math.degrees(self.home_yaw)
            )

    def velocity_cb(self, msg):
        self.current_vel = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ]
        self.last_vel_rx = rospy.Time.now()

    def range_cb(self, msg):
        value = safe_float(msg.range, -1.0)
        if value > 0.0 and math.isfinite(value):
            self.range_m = value
            self.last_range_rx = rospy.Time.now()

    def car_state_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Invalid /car/state JSON: %s", str(exc))
            return

        required_direct = all(k in data for k in ["x", "y"])
        if not required_direct:
            rospy.logwarn_throttle(1.0, "/car/state missing x/y after bridge mapping")
            return

        self.car_data = data
        self.car_rx_time = rospy.Time.now()
        self.car_version += 1
        if "mission_id" in data:
            incoming_id = safe_int(data.get("mission_id", self.mission_id), self.mission_id)
            if self.fsm_state == "WAIT_START":
                self.mission_id = incoming_id

    def platform_vision_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Invalid /uav/platform_vision JSON: %s", str(exc))
            return
        if str(data.get("target", "landing_platform")).lower() != "landing_platform":
            return
        self.vision_data = data
        self.vision_rx_time = rospy.Time.now()
        self.vision_version += 1

    @staticmethod
    def run_id_to_int(run_id):
        digits = "".join(ch for ch in str(run_id) if ch.isdigit())
        return safe_int(digits, 0) if digits else 0

    def publish_command_result(self, cmd_id, action, ok, error="", detail=""):
        payload = {
            "cmd_id": str(cmd_id),
            "action": str(action).upper(),
            "ok": bool(ok),
            "error": str(error),
            "detail": str(detail),
            "run_id": self.run_id,
            "task": "T2" if self.mission_type == "dynamic_land" else "T1",
            "stamp": rospy.Time.now().to_sec(),
        }
        self.command_result_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def publish_event_once(self, event):
        event = str(event).strip().upper()
        if not event or event in self.emitted_events:
            return
        self.emitted_events.add(event)
        payload = {
            "event": event,
            "run_id": self.run_id,
            "task": "T2" if self.mission_type == "dynamic_land" else "T1",
            "stamp": rospy.Time.now().to_sec(),
        }
        self.mission_event_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def mission_command_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn("Invalid /uav/mission_command JSON: %s", str(exc))
            return
        action = str(data.get("action", "")).strip().upper()
        cmd_id = str(data.get("cmd_id", ""))
        if action != "START":
            self.publish_command_result(cmd_id, action, False, "UNKNOWN_CMD")
            return
        mission_type = self.normalize_mission_type(data.get("task", "T1"))
        if self.reset_required or self.fsm_state == "WAIT_RESET":
            self.publish_command_result(cmd_id, action, False, "NOT_READY", "RESET_REQUIRED")
            return
        if self.fsm_state != "WAIT_START":
            self.publish_command_result(cmd_id, action, False, "BUSY", self.fsm_state)
            return
        if not self.home_ready:
            self.publish_command_result(cmd_id, action, False, "NOT_READY", "HOME_NOT_READY")
            return
        self.pending_mission_type = mission_type
        self.run_id = str(data.get("run_id", "R000"))
        self.mission_id = self.run_id_to_int(self.run_id)
        self.start_mission()
        self.publish_command_result(cmd_id, action, True)

    def mission_type_cb(self, msg):
        mission_type = self.normalize_mission_type(msg.data)
        if self.fsm_state not in {"WAIT_START", "WAIT_RESET"}:
            rospy.logwarn("Mission type change ignored while task is active: %s", mission_type)
            return
        self.pending_mission_type = mission_type
        rospy.logwarn("Pending mission type set to %s", mission_type)

    def start_cb(self, msg):
        if not msg.data:
            return
        if self.reset_required or self.fsm_state == "WAIT_RESET":
            rospy.logerr("Start rejected: publish /uav/reset=True after disarm.")
            return
        if self.fsm_state != "WAIT_START":
            rospy.logwarn("Start ignored: FSM state=%s", self.fsm_state)
            return
        if not self.home_ready:
            rospy.logerr("Start rejected: local position/home not ready.")
            return
        self.start_mission()

    def stop_cb(self, msg):
        if msg.data:
            self.request_emergency_land("/uav/stop")

    def land_cb(self, msg):
        if not msg.data:
            return
        # 已经稳定停在小车平台且尚未重新解锁时，LAND 只取消后续起飞，
        # 保持上锁并等待 RESET，不能再进入 AUTO.LAND。
        if (
            self.fsm_state in {"PLATFORM_DISARM", "PLATFORM_DWELL", "PLATFORM_TAKEOFF"}
            and not self.current_state.armed
        ):
            self.start_requested = False
            self.reset_required = True
            self.expected_disarm = False
            self.mission_result = "ABORTED_LANDED_ON_CAR"
            self.safety_state = "LANDED"
            self.abort_reason = "LAND command while landed on car"
            self.publish_event_once("UAV_LANDED")
            self.enter_state("WAIT_RESET")
            return
        self.request_emergency_land("/uav/land")

    def disarm_cb(self, msg):
        if not msg.data:
            return
        if self.is_landed() or not self.current_state.armed:
            self.request_arm(False)
            self.reset_required = True
            self.enter_state("WAIT_RESET")
        else:
            self.request_emergency_land("/uav/disarm while airborne")

    def reset_cb(self, msg):
        if not msg.data:
            return
        if self.current_state.armed:
            rospy.logerr("Reset rejected: vehicle is armed.")
            return
        self.clear_runtime()
        self.reset_required = False
        self.start_requested = False
        self.mission_result = "IDLE"
        self.safety_state = "IDLE"
        self.abort_reason = ""
        self.enter_state("WAIT_START")
        rospy.logwarn("FSM reset complete; ready for next mission.")

    def drop_ack_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"success": "OK" in msg.data.upper(), "reply": msg.data}
        action_id = safe_int(data.get("action_id", self.drop_action_id), self.drop_action_id)
        if action_id != self.drop_action_id:
            return
        self.drop_ack_success = bool(data.get("success", False))
        self.drop_ack_text = str(data.get("reply", data.get("reason", "")))

    def start_mission(self):
        self.clear_runtime(keep_mission_id=True)
        self.mission_type = self.pending_mission_type
        if self.mission_id <= 0:
            self.mission_id = int(rospy.Time.now().to_sec())
        if self.run_id == "R000":
            self.run_id = "R%d" % self.mission_id
        self.start_requested = True
        self.mission_start_time = rospy.Time.now()
        self.mission_result = "RUNNING"
        self.safety_state = "NORMAL"
        self.wait_fcu_prestream_start = rospy.Time.now()
        self.commanded_yaw = self.takeoff_yaw
        self.enter_state("WAIT_FCU")
        self.publish_event_once("MISSION_START")
        rospy.logwarn(
            "Mission started: run=%s id=%d type=%s",
            self.run_id, self.mission_id, self.mission_type
        )

    def clear_runtime(self, keep_mission_id=False):
        self.tracker.reset()
        self.car_estimate = None
        self.last_processed_car_version = -1
        self.last_processed_vision_version = -1
        self.cmd_xy = [0.0, 0.0]
        self.cmd_vz = 0.0
        self.stable_since = None
        self.follow_stable_since = None
        self.drop_stable_since = None
        self.touchdown_since = None
        self.home_stable_since = None
        self.takeoff_stable_since = None
        self.drop_hover_started_by_stable = False
        self.last_follow_metrics = None
        self.last_follow_metrics_time = None
        self.landing_target_z = None
        self.landing_attempts = 0
        self.dwell_start = None
        self.platform_takeoff_prestream_start = None
        self.drop_send_time = None
        self.drop_retry_count = 0
        self.drop_ack_success = False
        self.drop_ack_text = ""
        self.drop_done_time = None
        self.expected_disarm = False
        self.emitted_events = set()
        self.mission_start_time = None
        self.auto_land_start = None
        self.auto_land_prepare_start = None
        if not keep_mission_id:
            self.mission_id = 0
            self.run_id = "R000"

    def enter_state(self, new_state):
        if self.fsm_state != new_state:
            rospy.logwarn("FSM: %s -> %s", self.fsm_state, new_state)
        self.fsm_state = new_state
        self.state_enter_time = rospy.Time.now()
        self.stable_since = None
        # 每次切换任务状态都重新累计伴飞稳定时间，避免上一状态的计时直接穿透。
        self.follow_stable_since = None
        if new_state != "DROP_ALIGN":
            self.drop_stable_since = None
        if new_state != "DYNAMIC_DESCENT":
            self.touchdown_since = None
        if new_state in {"FOLLOW_DROP", "FOLLOW_CD"}:
            self.publish_event_once("UAV_FOLLOW_ESTABLISHED")
        elif new_state == "PLATFORM_DWELL":
            self.publish_event_once("UAV_LAND_ON_CAR")
        elif new_state == "HOME_LAND":
            self.publish_event_once("UAV_LANDING")

    def state_elapsed(self):
        return max(0.0, (rospy.Time.now() - self.state_enter_time).to_sec())

    def make_target_msg(self):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        return msg

    def publish_velocity_yaw(self, vx, vy, vz, yaw):
        msg = self.make_target_msg()
        msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz
        msg.yaw = yaw
        self.raw_pub.publish(msg)

    def publish_position_velocity_yaw(self, x, y, z, vx, vy, vz, yaw):
        msg = self.make_target_msg()
        msg.type_mask = (
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE
        )
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz
        msg.yaw = yaw
        self.raw_pub.publish(msg)

    def publish_neutral(self):
        self.publish_velocity_yaw(0.0, 0.0, 0.0, self.current_yaw)

    def request_mode(self, mode):
        now = rospy.Time.now()
        if (now - self.last_mode_request).to_sec() < self.mode_request_interval_s:
            return False
        self.last_mode_request = now
        try:
            result = self.set_mode_srv(base_mode=0, custom_mode=mode)
            return bool(result.mode_sent)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(1.0, "SetMode(%s) failed: %s", mode, str(exc))
            return False

    def request_arm(self, arm):
        now = rospy.Time.now()
        if (now - self.last_arm_request).to_sec() < self.arm_request_interval_s:
            return False
        self.last_arm_request = now
        try:
            result = self.arm_srv(bool(arm))
            return bool(result.success)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(1.0, "Arming(%s) failed: %s", str(arm), str(exc))
            return False

    def try_offboard_and_arm(self, allow_arm=True):
        if self.auto_set_mode and self.current_state.mode != "OFFBOARD":
            self.request_mode("OFFBOARD")
        if self.auto_arm and allow_arm and not self.current_state.armed:
            self.request_arm(True)

    def is_landed(self):
        if not self.current_state.armed:
            return True
        if self.extended_state_received:
            return self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        return False

    def current_xyz(self):
        if self.current_pose is None:
            return None
        p = self.current_pose.pose.position
        return p.x, p.y, p.z

    def pose_age(self):
        if self.last_pose_rx is None:
            return float("inf")
        return (rospy.Time.now() - self.last_pose_rx).to_sec()

    def car_age(self):
        if self.car_rx_time is None:
            return float("inf")
        return (rospy.Time.now() - self.car_rx_time).to_sec()

    def vision_age(self):
        if self.vision_rx_time is None:
            return float("inf")
        return (rospy.Time.now() - self.vision_rx_time).to_sec()

    def range_age(self):
        if self.last_range_rx is None:
            return float("inf")
        return (rospy.Time.now() - self.last_range_rx).to_sec()

    def telemetry_valid(self):
        return self.car_age() <= self.telemetry_timeout and bool(self.car_data)

    def vision_valid(self):
        if self.vision_age() > self.vision_timeout or not self.vision_data:
            return False
        return (
            bool(self.vision_data.get("detected", False)) and
            safe_float(self.vision_data.get("confidence", 0.0)) >= self.vision_min_confidence and
            safe_int(self.vision_data.get("stable_count", 0)) >= self.vision_min_stable_count
        )

    def car_abs_measurement(self):
        if not self.telemetry_valid() or not self.home_ready:
            return None
        x = safe_float(self.car_data.get("x", 0.0))
        y = safe_float(self.car_data.get("y", 0.0))
        if self.car_state_is_relative_home or self.frame_mode == "relative_home":
            x += self.home_x
            y += self.home_y
        return {
            "x": x,
            "y": y,
            "vx": safe_float(self.car_data.get("vx", 0.0)),
            "vy": safe_float(self.car_data.get("vy", 0.0)),
            "yaw": safe_float(self.car_data.get("yaw", 0.0)),
            "segment": str(self.car_data.get("segment", "UNKNOWN")).upper(),
            "segment_progress": clamp(
                safe_float(self.car_data.get("segment_progress", -1.0)), -1.0, 1.0
            ),
            "running": bool(self.car_data.get("running", True)),
        }

    def vision_abs_measurement(self):
        if not self.vision_valid() or self.current_pose is None:
            return None
        forward = safe_float(self.vision_data.get("forward_m", 0.0))
        left = safe_float(self.vision_data.get("left_m", 0.0))
        dx = math.cos(self.current_yaw) * forward - math.sin(self.current_yaw) * left
        dy = math.sin(self.current_yaw) * forward + math.cos(self.current_yaw) * left
        p = self.current_pose.pose.position
        return {
            "x": p.x + dx,
            "y": p.y + dy,
            "confidence": safe_float(self.vision_data.get("confidence", 0.0)),
            "stable_count": safe_int(self.vision_data.get("stable_count", 0)),
        }

    def update_car_estimator(self, prediction_s):
        now = rospy.Time.now()
        now_sec = now.to_sec()
        if self.last_estimator_time is None:
            self.last_estimator_time = now
        dt = clamp((now - self.last_estimator_time).to_sec(), 0.01, 0.20)
        self.last_estimator_time = now

        if self.tracker.initialized:
            self.tracker.predict_to(now_sec)

        telemetry = self.car_abs_measurement()
        if telemetry is not None and self.car_version != self.last_processed_car_version:
            if not self.tracker.initialized:
                self.tracker.initialize(
                    telemetry["x"], telemetry["y"], telemetry["vx"], telemetry["vy"], now_sec
                )
            else:
                self.tracker.correct_position(
                    telemetry["x"], telemetry["y"],
                    self.telemetry_position_alpha, self.position_beta, dt
                )
                self.tracker.correct_velocity(
                    telemetry["vx"], telemetry["vy"], self.telemetry_velocity_alpha
                )
            self.last_processed_car_version = self.car_version

        vision = self.vision_abs_measurement()
        if vision is not None and self.vision_version != self.last_processed_vision_version:
            if not self.tracker.initialized:
                self.tracker.initialize(vision["x"], vision["y"], 0.0, 0.0, now_sec)
            else:
                alpha = self.vision_position_alpha * clamp(vision["confidence"], 0.4, 1.0)
                self.tracker.correct_position(
                    vision["x"], vision["y"], alpha, self.position_beta, dt
                )
            self.last_processed_vision_version = self.vision_version

        if not self.tracker.initialized:
            self.car_estimate = None
            return None

        # 视觉抖动不能把目标速度估计拉到不合理值。
        self.tracker.vx, self.tracker.vy = limit_xy(
            self.tracker.vx, self.tracker.vy, self.max_car_speed
        )

        prediction = clamp(float(prediction_s), 0.0, self.max_prediction_s)
        result = self.tracker.snapshot(prediction)
        if telemetry is not None:
            result.update({
                "yaw": telemetry["yaw"],
                "segment": telemetry["segment"],
                "segment_progress": telemetry["segment_progress"],
                "running": telemetry["running"],
            })
        else:
            result.update({
                "yaw": math.atan2(result["vy"], result["vx"])
                if norm2(result["vx"], result["vy"]) > 0.03 else self.commanded_yaw,
                "segment": str(self.car_data.get("segment", "UNKNOWN")).upper(),
                "segment_progress": safe_float(self.car_data.get("segment_progress", -1.0)),
                "running": bool(self.car_data.get("running", True)),
            })
        self.car_estimate = result
        return deepcopy(result)

    def target_yaw_for_car(self, car):
        """返回当前控制模式期望的最终航向，不改变航向限速器状态。"""
        mode = str(self.follow_cfg.get("yaw_mode", "car")).strip().lower()
        if mode == "car" and car is not None:
            return safe_float(car.get("yaw", self.commanded_yaw), self.commanded_yaw)
        return math.radians(
            safe_float(self.follow_cfg.get("fixed_yaw_deg", 0.0), 0.0)
        )

    def desired_yaw(self, car, dt):
        target = self.target_yaw_for_car(car)
        max_rate = math.radians(
            safe_float(self.follow_cfg.get("max_yaw_rate_deg_s", 80.0), 80.0)
        )
        delta = clamp(wrap_pi(target - self.commanded_yaw), -max_rate * dt, max_rate * dt)
        self.commanded_yaw = wrap_pi(self.commanded_yaw + delta)
        return self.commanded_yaw

    def drop_uav_center_offset(self, car):
        """
        根据投放装置相对无人机控制中心的位置，计算无人机中心应施加的补偿。

        YAML 中 release_offset_x_m / release_offset_y_m 表示“投放出口”相对无人机
        控制中心的位移。body 坐标下：+X 为机头前方，+Y 为机体左方；local 坐标
        下：+X/+Y 与 MAVROS local ENU 一致。为了让投放出口位于小车中心上方，
        无人机中心目标需要取该位移的相反数。
        """
        release_x = safe_float(self.drop_cfg.get("release_offset_x_m", 0.0), 0.0)
        release_y = safe_float(self.drop_cfg.get("release_offset_y_m", 0.0), 0.0)
        frame = str(self.drop_cfg.get("release_offset_frame", "body")).strip().lower()

        if frame == "local":
            release_local_x = release_x
            release_local_y = release_y
        else:
            yaw = self.target_yaw_for_car(car)
            release_local_x = math.cos(yaw) * release_x - math.sin(yaw) * release_y
            release_local_y = math.sin(yaw) * release_x + math.cos(yaw) * release_y

        return -release_local_x, -release_local_y

    def car_target_xy(self, car):
        yaw = safe_float(car.get("yaw", 0.0), 0.0)
        forward_offset = safe_float(self.follow_cfg.get("offset_forward_m", 0.0), 0.0)
        left_offset = safe_float(self.follow_cfg.get("offset_left_m", 0.0), 0.0)
        tx = (
            car["x"] + math.cos(yaw) * forward_offset - math.sin(yaw) * left_offset
        )
        ty = (
            car["y"] + math.sin(yaw) * forward_offset + math.cos(yaw) * left_offset
        )
        return tx, ty

    def publish_follow_control(
        self, car, target_z, dt, speed_scale=1.0, target_offset_xy=None
    ):
        if self.current_pose is None:
            return None
        p = self.current_pose.pose.position
        tx, ty = self.car_target_xy(car)
        if target_offset_xy is not None:
            tx += safe_float(target_offset_xy[0], 0.0)
            ty += safe_float(target_offset_xy[1], 0.0)
        ex = tx - p.x
        ey = ty - p.y
        evx = car["vx"] - self.current_vel[0]
        evy = car["vy"] - self.current_vel[1]

        ff = safe_float(self.follow_cfg.get("velocity_feedforward_gain", 1.0), 1.0)
        kp = safe_float(self.follow_cfg.get("position_kp", 0.95), 0.95)
        kd = safe_float(self.follow_cfg.get("relative_velocity_kd", 0.35), 0.35)
        desired_vx = ff * car["vx"] + kp * ex + kd * evx
        desired_vy = ff * car["vy"] + kp * ey + kd * evy

        max_speed = safe_float(
            self.follow_cfg.get("max_horizontal_speed_mps", 1.20), 1.20
        ) * speed_scale
        desired_vx, desired_vy = limit_xy(desired_vx, desired_vy, max_speed)
        max_acc = safe_float(
            self.follow_cfg.get("max_horizontal_accel_mps2", 0.80), 0.80
        )
        self.cmd_xy = limit_xy_change(
            self.cmd_xy, [desired_vx, desired_vy], max_acc * max(dt, 0.01)
        )

        z_error = target_z - p.z
        z_kp = safe_float(self.follow_cfg.get("vertical_kp", 0.90), 0.90)
        max_vz = safe_float(self.follow_cfg.get("max_vertical_speed_mps", 0.45), 0.45)
        desired_vz = clamp(z_kp * z_error, -max_vz, max_vz)
        max_az = safe_float(
            self.follow_cfg.get("max_vertical_accel_mps2", 0.60), 0.60
        )
        self.cmd_vz += clamp(desired_vz - self.cmd_vz, -max_az * dt, max_az * dt)

        yaw = self.desired_yaw(car, dt)
        self.publish_position_velocity_yaw(
            tx, ty, target_z,
            self.cmd_xy[0], self.cmd_xy[1], self.cmd_vz,
            yaw,
        )
        metrics = {
            "target_x": tx,
            "target_y": ty,
            "position_error": norm2(ex, ey),
            "relative_speed": norm2(evx, evy),
            "ex": ex,
            "ey": ey,
            "evx": evx,
            "evy": evy,
            "target_offset_x": safe_float(target_offset_xy[0], 0.0)
                if target_offset_xy is not None else 0.0,
            "target_offset_y": safe_float(target_offset_xy[1], 0.0)
                if target_offset_xy is not None else 0.0,
        }
        self.last_follow_metrics = deepcopy(metrics)
        self.last_follow_metrics_time = rospy.Time.now()
        return metrics

    def publish_point_control(self, tx, ty, tz, dt, max_speed, max_accel, kp):
        if self.current_pose is None:
            return None
        p = self.current_pose.pose.position
        ex = tx - p.x
        ey = ty - p.y
        desired_vx, desired_vy = limit_xy(kp * ex, kp * ey, max_speed)
        self.cmd_xy = limit_xy_change(
            self.cmd_xy, [desired_vx, desired_vy], max_accel * max(dt, 0.01)
        )
        z_kp = safe_float(self.follow_cfg.get("vertical_kp", 0.90), 0.90)
        max_vz = safe_float(self.follow_cfg.get("max_vertical_speed_mps", 0.45), 0.45)
        desired_vz = clamp(z_kp * (tz - p.z), -max_vz, max_vz)
        max_az = safe_float(self.follow_cfg.get("max_vertical_accel_mps2", 0.60), 0.60)
        self.cmd_vz += clamp(desired_vz - self.cmd_vz, -max_az * dt, max_az * dt)
        yaw = self.desired_yaw(None, dt)
        self.publish_position_velocity_yaw(
            tx, ty, tz, self.cmd_xy[0], self.cmd_xy[1], self.cmd_vz, yaw
        )
        return norm2(ex, ey), abs(tz - p.z), norm3(*self.current_vel)

    def in_segment_window(self, car, name, start, end):
        if car is None:
            return False
        return (
            str(car.get("segment", "")).upper() == str(name).upper() and
            start <= safe_float(car.get("segment_progress", -1.0)) <= end
        )

    def update_stable_timer(self, condition, attr_name, required_time):
        now = rospy.Time.now()
        start = getattr(self, attr_name)
        if condition:
            if start is None:
                setattr(self, attr_name, now)
                return False
            return (now - start).to_sec() >= required_time
        setattr(self, attr_name, None)
        return False

    def follow_is_stable(self, metrics, strict=False):
        if metrics is None or not self.vision_valid():
            return False
        if strict:
            pos_limit = safe_float(self.follow_cfg.get("stable_position_error_m", 0.12), 0.12)
            speed_limit = safe_float(
                self.follow_cfg.get("stable_relative_speed_mps", 0.10), 0.10
            )
            stable_time = safe_float(self.follow_cfg.get("stable_time_s", 0.50), 0.50)
        else:
            pos_limit = safe_float(self.follow_cfg.get("acquire_position_error_m", 0.20), 0.20)
            speed_limit = safe_float(
                self.follow_cfg.get("acquire_relative_speed_mps", 0.18), 0.18
            )
            stable_time = safe_float(
                self.follow_cfg.get("acquire_stable_time_s", 0.45), 0.45
            )
        return self.update_stable_timer(
            metrics["position_error"] <= pos_limit and metrics["relative_speed"] <= speed_limit,
            "follow_stable_since",
            stable_time,
        )

    def send_drop_command(self):
        payload = {
            "cmd": str(self.drop_cfg.get("command", "R1")),
            "mission_id": self.mission_id,
            "action_id": self.drop_action_id,
            "release_offset_frame": str(
                self.drop_cfg.get("release_offset_frame", "body")
            ),
            "release_offset_x_m": safe_float(
                self.drop_cfg.get("release_offset_x_m", 0.0), 0.0
            ),
            "release_offset_y_m": safe_float(
                self.drop_cfg.get("release_offset_y_m", 0.0), 0.0
            ),
            "stamp": rospy.Time.now().to_sec(),
        }
        self.drop_cmd_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.drop_send_time = rospy.Time.now()
        self.drop_retry_count += 1
        rospy.logwarn(
            "Drop command sent: action=%d retry=%d/%d",
            self.drop_action_id,
            self.drop_retry_count,
            safe_int(self.drop_cfg.get("max_retries", 3), 3),
        )

    def request_emergency_land(self, reason):
        if self.fsm_state in {"EMERGENCY_LAND", "WAIT_RESET"}:
            return
        self.abort_reason = str(reason)
        self.mission_result = "ABORTED"
        self.safety_state = "EMERGENCY_LAND"
        self.start_requested = False
        self.expected_disarm = True
        self.auto_land_prepare_start = rospy.Time.now()
        self.auto_land_start = None
        self.publish_event_once("UAV_ABORTING")
        self.enter_state("EMERGENCY_LAND")
        self.publish_event_once("UAV_LANDING")
        rospy.logerr("Emergency landing requested: %s", reason)

    def enter_return_home(self, result=None):
        if result is not None:
            self.mission_result = str(result)
        self.cmd_xy = [0.0, 0.0]
        self.cmd_vz = 0.0
        self.home_stable_since = None
        self.enter_state("RETURN_HOME")

    def handle_takeoff(self, dt):
        target_z = self.home_z + self.cruise_height
        metrics = self.publish_point_control(
            self.home_x, self.home_y, target_z, dt,
            safe_float(self.follow_cfg.get("max_horizontal_speed_mps", 1.2), 1.2),
            safe_float(self.follow_cfg.get("max_horizontal_accel_mps2", 0.8), 0.8),
            0.9,
        )
        self.try_offboard_and_arm(allow_arm=True)
        if metrics is None:
            return
        xy_err, z_err, speed = metrics
        stable = (
            xy_err <= safe_float(self.takeoff_cfg.get("xy_tolerance_m", 0.18), 0.18) and
            z_err <= safe_float(self.takeoff_cfg.get("z_tolerance_m", 0.10), 0.10) and
            speed <= safe_float(self.takeoff_cfg.get("speed_tolerance_mps", 0.20), 0.20)
        )
        # 任务一的规则：第一次满足位置+高度+速度阈值，就立刻开始3秒悬停计时；
        # 进入悬停后即使稳定条件暂时丢失，3秒到点仍然继续截获，不重新计时。
        if self.mission_type == "drop":
            if stable:
                self.drop_hover_started_by_stable = True
                rospy.logwarn(
                    "Task1 takeoff gate reached; start fixed %.2fs hover timer.",
                    safe_float(self.takeoff_cfg.get("drop_hover_time_s", 3.0), 3.0),
                )
                self.enter_state("DROP_HOVER")
                return

            if self.state_elapsed() > safe_float(
                self.takeoff_cfg.get("max_wait_s", 8.0), 8.0
            ):
                if z_err < safe_float(
                    self.takeoff_cfg.get("soft_timeout_z_error_m", 0.22), 0.22
                ):
                    self.drop_hover_started_by_stable = False
                    rospy.logwarn(
                        "Task1 takeoff soft timeout; start fixed hover timer anyway: "
                        "xy_err=%.2f z_err=%.2f speed=%.2f",
                        xy_err, z_err, speed,
                    )
                    self.enter_state("DROP_HOVER")
                else:
                    self.request_emergency_land("takeoff timeout")
            return

        # 任务二仍要求起飞状态连续稳定 stable_time_s 后才开始截获。
        ready = self.update_stable_timer(
            stable,
            "takeoff_stable_since",
            safe_float(self.takeoff_cfg.get("stable_time_s", 0.45), 0.45),
        )
        if ready:
            self.enter_state("INTERCEPT")
        elif self.state_elapsed() > safe_float(self.takeoff_cfg.get("max_wait_s", 8.0), 8.0):
            if z_err < safe_float(
                self.takeoff_cfg.get("soft_timeout_z_error_m", 0.22), 0.22
            ):
                rospy.logwarn("Task2 takeoff soft timeout accepted: z_err=%.2f", z_err)
                self.enter_state("INTERCEPT")
            else:
                self.request_emergency_land("takeoff timeout")

    def handle_intercept(self, dt):
        # 截获阶段不追逐小车当前点，而是追逐“当前估计位置 + 速度×预测时间”。
        # update_car_estimator() 返回的 x/y 已经是 prediction_s 秒后的预测坐标。
        prediction_s = safe_float(
            self.follow_cfg.get("intercept_prediction_s", 0.60), 0.60
        )
        car = self.update_car_estimator(prediction_s)
        if car is None:
            self.publish_position_velocity_yaw(
                self.home_x, self.home_y, self.home_z + self.cruise_height,
                0.0, 0.0, 0.0, self.commanded_yaw,
            )
            return
        metrics = self.publish_follow_control(
            car, self.home_z + self.cruise_height, dt, speed_scale=1.0
        )
        if self.follow_is_stable(metrics, strict=False):
            self.enter_state("FOLLOW_DROP" if self.mission_type == "drop" else "FOLLOW_CD")

    def handle_follow_drop(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is None:
            return
        metrics = self.publish_follow_control(car, self.home_z + self.cruise_height, dt)
        segment = self.drop_cfg.get("segment_name", "CD")
        start = safe_float(self.drop_cfg.get("window_start", 0.20), 0.20)
        end = safe_float(self.drop_cfg.get("window_end", 0.78), 0.78)
        if self.in_segment_window(car, segment, start, end) and self.follow_is_stable(metrics, strict=True):
            self.enter_state("DROP_ALIGN")
        elif (
            str(car.get("segment", "")).upper() == str(segment).upper() and
            safe_float(car.get("segment_progress", -1.0)) > end
        ):
            self.enter_return_home("DROP_WINDOW_MISSED")

    def handle_drop_align(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is None:
            return
        drop_offset = self.drop_uav_center_offset(car)
        metrics = self.publish_follow_control(
            car, self.home_z + self.cruise_height, dt,
            target_offset_xy=drop_offset,
        )
        segment = self.drop_cfg.get("segment_name", "CD")
        end = safe_float(self.drop_cfg.get("window_end", 0.78), 0.78)
        if (
            str(car.get("segment", "")).upper() != str(segment).upper() or
            safe_float(car.get("segment_progress", -1.0)) > end
        ):
            self.enter_return_home("DROP_ALIGN_WINDOW_LOST")
            return

        lead = safe_float(self.drop_cfg.get("fall_time_s", 0.58), 0.58)
        predicted_ex = metrics["ex"] + metrics["evx"] * lead
        predicted_ey = metrics["ey"] + metrics["evy"] * lead
        predicted_error = norm2(predicted_ex, predicted_ey)
        vision_ok = self.vision_valid() or not bool(self.drop_cfg.get("vision_required", True))
        condition = (
            vision_ok and
            metrics["position_error"] <= safe_float(
                self.drop_cfg.get("align_position_error_m", 0.075), 0.075
            ) and
            metrics["relative_speed"] <= safe_float(
                self.drop_cfg.get("align_relative_speed_mps", 0.075), 0.075
            ) and
            predicted_error <= safe_float(
                self.drop_cfg.get("predicted_error_limit_m", 0.085), 0.085
            )
        )
        ready = self.update_stable_timer(
            condition,
            "drop_stable_since",
            safe_float(self.drop_cfg.get("align_stable_time_s", 0.45), 0.45),
        )
        if ready:
            self.drop_action_id += 1
            self.drop_retry_count = 0
            self.drop_ack_success = False
            self.drop_ack_text = ""
            self.send_drop_command()
            self.enter_state("DROP_WAIT_ACK")

    def handle_drop_wait_ack(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is not None:
            self.publish_follow_control(
                car, self.home_z + self.cruise_height, dt,
                target_offset_xy=self.drop_uav_center_offset(car),
            )
        if self.drop_ack_success:
            self.drop_done_time = rospy.Time.now()
            self.mission_result = "DROP_DONE"
            self.publish_event_once("UAV_DROP_DONE")
            self.enter_state("POST_DROP_FOLLOW")
            return
        if self.drop_send_time is None:
            self.send_drop_command()
            return
        elapsed = (rospy.Time.now() - self.drop_send_time).to_sec()
        retry_interval = safe_float(self.drop_cfg.get("retry_interval_s", 0.55), 0.55)
        ack_timeout = safe_float(self.drop_cfg.get("ack_timeout_s", 1.80), 1.80)
        max_retries = safe_int(self.drop_cfg.get("max_retries", 3), 3)
        if elapsed >= retry_interval and self.drop_retry_count < max_retries:
            self.send_drop_command()
        elif elapsed >= ack_timeout and self.drop_retry_count >= max_retries:
            self.enter_return_home("DROP_ACK_FAILED")

    def handle_post_drop_follow(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is not None:
            self.publish_follow_control(car, self.home_z + self.cruise_height, dt)
        hold = safe_float(self.drop_cfg.get("post_drop_follow_s", 0.50), 0.50)
        if self.drop_done_time is not None and (rospy.Time.now() - self.drop_done_time).to_sec() >= hold:
            self.enter_return_home("DROP_TASK_COMPLETE")

    def handle_follow_cd(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is None:
            return
        metrics = self.publish_follow_control(car, self.home_z + self.cruise_height, dt)
        segment = str(self.land_cfg.get("segment_name", "CD")).upper()
        start = safe_float(self.land_cfg.get("window_start", 0.05), 0.05)
        latest = safe_float(self.land_cfg.get("latest_start", 0.42), 0.42)
        progress = safe_float(car.get("segment_progress", -1.0))
        attempts_max = safe_int(self.land_cfg.get("max_attempts_in_cd", 2), 2)
        entry_condition = (
            self.in_segment_window(car, segment, start, latest) and
            self.vision_valid() and
            metrics is not None and
            metrics["position_error"] <= safe_float(
                self.land_cfg.get("entry_position_error_m", 0.12), 0.12
            ) and
            metrics["relative_speed"] <= safe_float(
                self.land_cfg.get("entry_relative_speed_mps", 0.10), 0.10
            ) and
            self.landing_attempts < attempts_max
        )
        ready = self.update_stable_timer(
            entry_condition,
            "follow_stable_since",
            safe_float(self.land_cfg.get("entry_stable_time_s", 0.45), 0.45),
        )
        if ready:
            self.landing_attempts += 1
            self.landing_target_z = min(
                self.current_pose.pose.position.z,
                self.home_z + self.cruise_height,
            )
            self.enter_state("DYNAMIC_DESCENT")
            return
        if str(car.get("segment", "")).upper() == segment and progress > safe_float(
            self.land_cfg.get("abort_progress", 0.88), 0.88
        ):
            self.enter_return_home("DYNAMIC_LANDING_WINDOW_MISSED")

    def touchdown_condition(self, platform_z):
        use_extended = bool(self.land_cfg.get("use_extended_landed_state", True))
        extended_ok = (
            use_extended and self.extended_state_received and
            self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        )
        z_tol = safe_float(self.land_cfg.get("touchdown_z_tolerance_m", 0.13), 0.13)
        vz_limit = safe_float(
            self.land_cfg.get("touchdown_vertical_speed_mps", 0.10), 0.10
        )
        kinematic_ok = (
            self.current_pose is not None and
            self.current_pose.pose.position.z <= platform_z + z_tol and
            abs(self.current_vel[2]) <= vz_limit and
            self.landing_target_z is not None and
            self.landing_target_z <= platform_z +
            safe_float(self.land_cfg.get("min_target_height_above_platform_m", 0.03), 0.03) + 0.05
        )
        use_range = bool(self.land_cfg.get("use_rangefinder", True))
        range_required = bool(self.land_cfg.get("range_required_for_touchdown", False))
        range_available = self.range_m is not None and self.range_age() <= 0.35
        if not use_range:
            range_ok = True
        elif range_available:
            range_ok = self.range_m <= safe_float(
                self.land_cfg.get("touchdown_range_max_m", 0.14), 0.14
            )
        else:
            range_ok = not range_required
        return extended_ok or (kinematic_ok and range_ok)

    def handle_dynamic_descent(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is None:
            self.enter_return_home("CAR_ESTIMATE_LOST_DURING_DESCENT")
            return
        segment = str(self.land_cfg.get("segment_name", "CD")).upper()
        progress = safe_float(car.get("segment_progress", -1.0))
        abort_progress = safe_float(self.land_cfg.get("abort_progress", 0.88), 0.88)
        if str(car.get("segment", "")).upper() != segment or progress > abort_progress:
            self.enter_return_home("LEFT_CD_BEFORE_TOUCHDOWN")
            return

        platform_z = self.home_z + safe_float(
            self.land_cfg.get("platform_height_m", 0.34), 0.34
        )
        if self.landing_target_z is None:
            self.landing_target_z = self.current_pose.pose.position.z

        metrics_preview = self.follow_metrics(car)
        vision_age = self.vision_age()
        vision_ok = self.vision_valid()
        if not vision_ok and vision_age >= safe_float(
            self.land_cfg.get("vision_abort_timeout_s", 0.60), 0.60
        ):
            rospy.logerr("Landing vision lost %.2fs; abort current C-D descent attempt.", vision_age)
            self.landing_target_z = min(
                self.home_z + self.cruise_height,
                self.current_pose.pose.position.z + safe_float(
                    self.land_cfg.get("reacquire_climb_m", 0.28), 0.28
                ),
            )
            self.enter_state("FOLLOW_CD")
            return

        can_descend = (
            vision_ok and metrics_preview is not None and
            metrics_preview["position_error"] <= safe_float(
                self.land_cfg.get("pause_position_error_m", 0.14), 0.14
            ) and
            metrics_preview["relative_speed"] <= safe_float(
                self.land_cfg.get("pause_relative_speed_mps", 0.12), 0.12
            )
        )
        min_target = platform_z + safe_float(
            self.land_cfg.get("min_target_height_above_platform_m", 0.03), 0.03
        )
        if can_descend:
            height_above = max(0.0, self.landing_target_z - platform_z)
            slow_height = safe_float(
                self.land_cfg.get("slow_height_above_platform_m", 0.42), 0.42
            )
            rate = safe_float(
                self.land_cfg.get(
                    "touchdown_rate_mps" if height_above <= slow_height else "descent_rate_mps",
                    0.09 if height_above <= slow_height else 0.20,
                )
            )
            self.landing_target_z = max(min_target, self.landing_target_z - rate * dt)

        self.publish_follow_control(car, self.landing_target_z, dt, speed_scale=0.75)

        touched = self.touchdown_condition(platform_z)
        confirmed = self.update_stable_timer(
            touched,
            "touchdown_since",
            safe_float(self.land_cfg.get("touchdown_confirm_time_s", 0.45), 0.45),
        )
        if confirmed:
            self.expected_disarm = True
            self.enter_state("PLATFORM_DISARM")

    def follow_metrics(self, car):
        if self.current_pose is None or car is None:
            return None
        tx, ty = self.car_target_xy(car)
        p = self.current_pose.pose.position
        return {
            "position_error": norm2(tx - p.x, ty - p.y),
            "relative_speed": norm2(
                car["vx"] - self.current_vel[0], car["vy"] - self.current_vel[1]
            ),
            "ex": tx - p.x,
            "ey": ty - p.y,
            "evx": car["vx"] - self.current_vel[0],
            "evy": car["vy"] - self.current_vel[1],
        }

    def handle_platform_disarm(self, dt):
        car = self.update_car_estimator(0.0)
        platform_z = self.home_z + safe_float(
            self.land_cfg.get("platform_height_m", 0.34), 0.34
        )
        if car is not None:
            self.publish_follow_control(car, platform_z, dt, speed_scale=0.55)
        else:
            self.publish_neutral()
        if self.current_state.armed:
            self.request_arm(False)
            return
        self.dwell_start = rospy.Time.now()
        self.expected_disarm = False
        self.enter_state("PLATFORM_DWELL")
        self.mission_result = "LANDED_ON_CAR"

    def handle_platform_dwell(self, dt):
        car = self.update_car_estimator(0.0)
        platform_z = self.home_z + safe_float(
            self.land_cfg.get("platform_height_m", 0.34), 0.34
        )
        if car is not None:
            yaw = self.desired_yaw(car, dt)
            tx, ty = self.car_target_xy(car)
            self.publish_position_velocity_yaw(
                tx, ty, platform_z, car["vx"], car["vy"], 0.0, yaw
            )
        if self.dwell_start is None:
            self.dwell_start = rospy.Time.now()
        dwell = (rospy.Time.now() - self.dwell_start).to_sec()
        if dwell < safe_float(self.land_cfg.get("dwell_time_s", 5.0), 5.0):
            return
        if not self.telemetry_valid():
            rospy.logwarn_throttle(1.0, "Dwell complete but car telemetry is not valid; remain disarmed.")
            return
        self.platform_takeoff_prestream_start = rospy.Time.now()
        self.cmd_xy = [car["vx"], car["vy"]] if car is not None else [0.0, 0.0]
        self.cmd_vz = 0.0
        self.enter_state("PLATFORM_TAKEOFF")

    def handle_platform_takeoff(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is None or not self.telemetry_valid():
            if not self.current_state.armed:
                rospy.logerr_throttle(1.0, "Platform takeoff inhibited: car telemetry unavailable.")
                return
            self.enter_return_home("CAR_LOST_AFTER_PLATFORM_ARM")
            return
        target_z = self.home_z + self.cruise_height
        self.publish_follow_control(car, target_z, dt, speed_scale=0.85)

        if self.platform_takeoff_prestream_start is None:
            self.platform_takeoff_prestream_start = rospy.Time.now()
        prestream = (rospy.Time.now() - self.platform_takeoff_prestream_start).to_sec()
        if prestream >= safe_float(self.land_cfg.get("takeoff_prestream_s", 1.0), 1.0):
            self.try_offboard_and_arm(allow_arm=True)

        if self.current_state.armed:
            current_height = self.current_pose.pose.position.z - self.home_z
            platform_height = safe_float(self.land_cfg.get("platform_height_m", 0.34), 0.34)
            clearance = current_height - platform_height
            if (
                clearance >= safe_float(self.land_cfg.get("takeoff_clearance_m", 0.42), 0.42) and
                current_height >= safe_float(
                    self.land_cfg.get("takeoff_reached_height_m", 1.20), 1.20
                )
            ):
                self.mission_result = "PLATFORM_TAKEOFF_DONE"
                self.publish_event_once("UAV_TAKEOFF_FROM_CAR")
                self.enter_return_home("DYNAMIC_LAND_TASK_COMPLETE")
                return
        if self.state_elapsed() > safe_float(self.land_cfg.get("takeoff_max_wait_s", 8.0), 8.0):
            self.request_emergency_land("platform takeoff timeout")

    def handle_return_home(self, dt):
        target_z = self.home_z + safe_float(self.return_cfg.get("height_m", 1.50), 1.50)
        metrics = self.publish_point_control(
            self.home_x,
            self.home_y,
            target_z,
            dt,
            safe_float(self.return_cfg.get("max_horizontal_speed_mps", 1.0), 1.0),
            safe_float(self.return_cfg.get("max_horizontal_accel_mps2", 0.7), 0.7),
            safe_float(self.return_cfg.get("position_kp", 0.8), 0.8),
        )
        self.try_offboard_and_arm(allow_arm=True)
        if metrics is None:
            return
        xy_err, z_err, speed = metrics
        ready = self.update_stable_timer(
            xy_err <= safe_float(self.return_cfg.get("position_tolerance_m", 0.16), 0.16) and
            z_err <= 0.15 and
            speed <= safe_float(self.return_cfg.get("speed_tolerance_mps", 0.18), 0.18),
            "home_stable_since",
            safe_float(self.return_cfg.get("stable_time_s", 0.45), 0.45),
        )
        if ready:
            self.expected_disarm = True
            self.auto_land_prepare_start = rospy.Time.now()
            self.auto_land_start = None
            self.enter_state("HOME_LAND")

    def handle_auto_land(self, emergency=False):
        if self.current_pose is not None:
            p = self.current_pose.pose.position
            self.publish_position_velocity_yaw(
                p.x, p.y, p.z, 0.0, 0.0, 0.0, self.current_yaw
            )
        prepare = 0.20 if emergency else safe_float(
            self.return_cfg.get("auto_land_prepare_s", 0.60), 0.60
        )
        if self.auto_land_prepare_start is None:
            self.auto_land_prepare_start = rospy.Time.now()
        if (rospy.Time.now() - self.auto_land_prepare_start).to_sec() >= prepare:
            if self.current_state.mode != "AUTO.LAND":
                self.request_mode("AUTO.LAND")
            elif self.auto_land_start is None:
                self.auto_land_start = rospy.Time.now()

        if self.is_landed():
            if self.current_state.armed:
                self.request_arm(False)
                return
            self.expected_disarm = False
            self.start_requested = False
            self.reset_required = True
            if emergency:
                self.mission_result = "ABORTED_LANDED"
            elif self.mission_result in {"RUNNING", "DROP_DONE", "LANDED_ON_CAR", "PLATFORM_TAKEOFF_DONE", "DROP_TASK_COMPLETE", "DYNAMIC_LAND_TASK_COMPLETE"}:
                self.mission_result = "MISSION_COMPLETE"
            self.publish_event_once("UAV_LANDED")
            if not emergency and self.mission_result == "MISSION_COMPLETE":
                self.publish_event_once("MISSION_DONE")
            self.enter_state("WAIT_RESET")
            return

        timeout = safe_float(self.return_cfg.get("auto_land_timeout_s", 15.0), 15.0)
        if self.auto_land_start is not None and (
            rospy.Time.now() - self.auto_land_start
        ).to_sec() > timeout:
            rospy.logerr_throttle(1.0, "AUTO.LAND timeout; keep requesting mode and waiting for landing.")
            self.request_mode("AUTO.LAND")

    def safety_checks(self):
        if self.fsm_state in {"WAIT_START", "WAIT_FCU", "WAIT_RESET", "PLATFORM_DWELL"}:
            return
        if self.current_state.armed and self.pose_age() > safe_float(
            self.safety_cfg.get("odometry_loss_auto_land_s", 0.70), 0.70
        ):
            self.request_emergency_land("local odometry timeout")
            return

        car_dependent = self.fsm_state in {
            "INTERCEPT", "FOLLOW_DROP", "DROP_ALIGN", "DROP_WAIT_ACK",
            "POST_DROP_FOLLOW", "FOLLOW_CD", "DYNAMIC_DESCENT", "PLATFORM_TAKEOFF"
        }
        if car_dependent and self.car_age() > safe_float(
            self.safety_cfg.get("car_loss_return_s", 1.20), 1.20
        ):
            if self.fsm_state == "DYNAMIC_DESCENT":
                self.enter_return_home("CAR_COMM_LOST_DURING_DESCENT")
            elif self.fsm_state != "PLATFORM_TAKEOFF" or self.current_state.armed:
                self.enter_return_home("CAR_COMM_LOST")

    def publish_status(self):
        now = rospy.Time.now()
        if (now - self.last_status_publish).to_sec() < 1.0 / max(self.status_rate_hz, 1.0):
            return
        self.last_status_publish = now
        p = self.current_pose.pose.position if self.current_pose is not None else None
        car = self.car_estimate or {}
        follow = self.last_follow_metrics or {}
        follow_age = (
            (now - self.last_follow_metrics_time).to_sec()
            if self.last_follow_metrics_time is not None else float("inf")
        )
        state_cn = {
            "WAIT_START": "等待启动",
            "WAIT_FCU": "飞控准备",
            "TAKEOFF": "起飞",
            "DROP_HOVER": "起飞点悬停",
            "INTERCEPT": "预测截获小车",
            "FOLLOW_DROP": "伴飞等待投放",
            "DROP_ALIGN": "投放对准",
            "DROP_WAIT_ACK": "投放执行",
            "POST_DROP_FOLLOW": "投放后伴飞",
            "FOLLOW_CD": "伴飞等待CD段",
            "DYNAMIC_DESCENT": "移动平台下降",
            "PLATFORM_DISARM": "平台着陆确认",
            "PLATFORM_DWELL": "平台停留",
            "PLATFORM_TAKEOFF": "随车起飞",
            "RETURN_HOME": "返航",
            "HOME_LAND": "H点降落",
            "EMERGENCY_LAND": "应急降落",
            "WAIT_RESET": "任务结束等待复位",
        }.get(self.fsm_state, self.fsm_state)
        payload = {
            "stamp": now.to_sec(),
            "mission_id": self.mission_id,
            "run_id": self.run_id,
            "mission_time_s": (
                (now - self.mission_start_time).to_sec()
                if self.mission_start_time is not None else 0.0
            ),
            "mission_type": self.mission_type,
            "fsm_state": self.fsm_state,
            "fsm_state_cn": state_cn,
            "safety_state": self.safety_state,
            "mission_result": self.mission_result,
            "abort_reason": self.abort_reason,
            "mavros": {
                "connected": bool(self.current_state.connected),
                "armed": bool(self.current_state.armed),
                "mode": str(self.current_state.mode),
            },
            "home": {
                "x": self.home_x if self.home_ready else None,
                "y": self.home_y if self.home_ready else None,
                "z": self.home_z if self.home_ready else None,
                "yaw": self.home_yaw if self.home_ready else None,
            },
            "uav": {
                "x": p.x if p is not None else None,
                "y": p.y if p is not None else None,
                "z": p.z if p is not None else None,
                "x_rel_home": (p.x - self.home_x) if p is not None and self.home_ready else None,
                "y_rel_home": (p.y - self.home_y) if p is not None and self.home_ready else None,
                "z_rel_home": (p.z - self.home_z) if p is not None and self.home_ready else None,
                "vx": self.current_vel[0],
                "vy": self.current_vel[1],
                "vz": self.current_vel[2],
                "yaw": self.current_yaw,
            },
            "car": {
                "valid": self.telemetry_valid(),
                "age": self.car_age(),
                "x": car.get("x"),
                "y": car.get("y"),
                "vx": car.get("vx"),
                "vy": car.get("vy"),
                "yaw": car.get("yaw"),
                "running": bool(self.car_data.get("running", True)),
                "path_s": self.car_data.get("path_s"),
                "path_s_cm": self.car_data.get("path_s_cm"),
                "segment_reported": self.car_data.get("segment_reported"),
                "segment_from_path": self.car_data.get("segment_from_path"),
                "segment_consistent": self.car_data.get("segment_consistent"),
                "lap_progress": self.car_data.get("lap_progress"),
                "segment": car.get("segment", self.car_data.get("segment", "UNKNOWN")),
                "segment_progress": car.get(
                    "segment_progress", self.car_data.get("segment_progress", -1.0)
                ),
            },
            "tracking": {
                "valid": follow_age <= max(self.telemetry_timeout, self.vision_timeout, 0.5),
                "age": follow_age,
                "target_x": follow.get("target_x"),
                "target_y": follow.get("target_y"),
                "position_error_m": follow.get("position_error"),
                "relative_speed_mps": follow.get("relative_speed"),
                "error_x_m": follow.get("ex"),
                "error_y_m": follow.get("ey"),
                "relative_vx_mps": follow.get("evx"),
                "relative_vy_mps": follow.get("evy"),
                "target_offset_x_m": follow.get("target_offset_x", 0.0),
                "target_offset_y_m": follow.get("target_offset_y", 0.0),
                "intercept_prediction_s": safe_float(
                    self.follow_cfg.get("intercept_prediction_s", 0.60), 0.60
                ),
            },
            "vision": {
                "valid": self.vision_valid(),
                "age": self.vision_age(),
                "confidence": self.vision_data.get("confidence", 0.0),
                "stable_count": self.vision_data.get("stable_count", 0),
                "forward_m": self.vision_data.get("forward_m"),
                "left_m": self.vision_data.get("left_m"),
            },
            "landing": {
                "attempts": self.landing_attempts,
                "target_z": self.landing_target_z,
                "range_m": self.range_m,
                "dwell_s": (
                    (now - self.dwell_start).to_sec() if self.dwell_start is not None else 0.0
                ),
            },
            "takeoff": {
                "hover_required_s": safe_float(
                    self.takeoff_cfg.get("drop_hover_time_s", 3.0), 3.0
                ),
                "hover_elapsed_s": self.state_elapsed()
                    if self.fsm_state == "DROP_HOVER" else 0.0,
                "hover_started_by_stable": self.drop_hover_started_by_stable,
            },
            "drop": {
                "action_id": self.drop_action_id,
                "retry_count": self.drop_retry_count,
                "ack": self.drop_ack_success,
                "ack_text": self.drop_ack_text,
                "release_offset_frame": str(
                    self.drop_cfg.get("release_offset_frame", "body")
                ),
                "release_offset_x_m": safe_float(
                    self.drop_cfg.get("release_offset_x_m", 0.0), 0.0
                ),
                "release_offset_y_m": safe_float(
                    self.drop_cfg.get("release_offset_y_m", 0.0), 0.0
                ),
            },
        }
        text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.fsm_state_pub.publish(String(data=self.fsm_state))
        self.safety_state_pub.publish(String(data=self.safety_state))
        self.mission_status_pub.publish(String(data=text))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = clamp((now - last).to_sec(), 0.01, 0.10)
            last = now

            self.platform_scan_pub.publish(Bool(data=self.fsm_state in self.VISION_STATES))
            self.publish_status()
            self.safety_checks()

            if self.current_pose is None or not self.home_ready:
                rate.sleep()
                continue

            if self.fsm_state == "WAIT_START":
                self.publish_neutral()

            elif self.fsm_state == "WAIT_FCU":
                target_z = self.home_z + self.cruise_height
                self.publish_position_velocity_yaw(
                    self.home_x, self.home_y, target_z,
                    0.0, 0.0, 0.0, self.takeoff_yaw,
                )
                if self.wait_fcu_prestream_start is None:
                    self.wait_fcu_prestream_start = now
                prestream = (now - self.wait_fcu_prestream_start).to_sec()
                if prestream >= self.offboard_prestream_s:
                    self.try_offboard_and_arm(allow_arm=True)
                if self.current_state.connected and self.current_state.mode == "OFFBOARD" and self.current_state.armed:
                    self.enter_state("TAKEOFF")

            elif self.fsm_state == "TAKEOFF":
                self.handle_takeoff(dt)

            elif self.fsm_state == "DROP_HOVER":
                self.publish_point_control(
                    self.home_x, self.home_y, self.home_z + self.cruise_height, dt,
                    0.5, 0.5, 0.8,
                )
                if self.state_elapsed() >= safe_float(
                    self.takeoff_cfg.get("drop_hover_time_s", 3.0), 3.0
                ):
                    self.enter_state("INTERCEPT")

            elif self.fsm_state == "INTERCEPT":
                self.handle_intercept(dt)

            elif self.fsm_state == "FOLLOW_DROP":
                self.handle_follow_drop(dt)

            elif self.fsm_state == "DROP_ALIGN":
                self.handle_drop_align(dt)

            elif self.fsm_state == "DROP_WAIT_ACK":
                self.handle_drop_wait_ack(dt)

            elif self.fsm_state == "POST_DROP_FOLLOW":
                self.handle_post_drop_follow(dt)

            elif self.fsm_state == "FOLLOW_CD":
                self.handle_follow_cd(dt)

            elif self.fsm_state == "DYNAMIC_DESCENT":
                self.handle_dynamic_descent(dt)

            elif self.fsm_state == "PLATFORM_DISARM":
                self.handle_platform_disarm(dt)

            elif self.fsm_state == "PLATFORM_DWELL":
                self.handle_platform_dwell(dt)

            elif self.fsm_state == "PLATFORM_TAKEOFF":
                self.handle_platform_takeoff(dt)

            elif self.fsm_state == "RETURN_HOME":
                self.handle_return_home(dt)

            elif self.fsm_state == "HOME_LAND":
                self.handle_auto_land(emergency=False)

            elif self.fsm_state == "EMERGENCY_LAND":
                self.handle_auto_land(emergency=True)

            elif self.fsm_state == "WAIT_RESET":
                self.platform_scan_pub.publish(Bool(data=False))
                if self.current_state.armed and self.is_landed():
                    self.request_arm(False)
                else:
                    self.publish_neutral()

            else:
                rospy.logerr_throttle(1.0, "Unknown FSM state: %s", self.fsm_state)
                self.request_emergency_land("unknown FSM state")

            rate.sleep()


if __name__ == "__main__":
    try:
        AirGroundMissionFSM().spin()
    except rospy.ROSInterruptException:
        pass
