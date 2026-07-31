#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
2026 D题陆空协同无人机任务状态机。

支持两种任务：
1. drop：起飞、悬停3秒、截获小车、伴飞、C-D段边跟车边下降、低空投放、返回H点降落；
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


def calculate_empirical_release_lead(vx, vy, configured_distance_m):
    """Return a bounded fixed-distance offset along the car velocity direction."""
    vx = safe_float(vx, 0.0)
    vy = safe_float(vy, 0.0)
    distance = clamp(
        safe_float(configured_distance_m, 0.0),
        -0.30,
        0.30,
    )
    speed = norm2(vx, vy)
    direction_valid = speed >= 0.05
    if direction_valid:
        direction_x = vx / speed
        direction_y = vy / speed
        lead_x = direction_x * distance
        lead_y = direction_y * distance
    else:
        direction_x = 0.0
        direction_y = 0.0
        lead_x = 0.0
        lead_y = 0.0
    return {
        "configured_distance_m": distance,
        "speed_mps": speed,
        "direction_valid": direction_valid,
        "direction_x": direction_x,
        "direction_y": direction_y,
        "lead_x": lead_x,
        "lead_y": lead_y,
    }


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


FOLLOW_VELOCITY_PROFILE_NAMES = (
    "intercept",
    "follow",
    "drop_descent",
    "drop_align",
)

FOLLOW_VELOCITY_PROFILE_FIELDS = (
    "velocity_feedforward_gain",
    "position_kp",
    "relative_velocity_kd",
    "max_horizontal_speed_mps",
    "max_correction_speed_mps",
    "max_horizontal_accel_mps2",
    "max_horizontal_jerk_mps3",
)


def load_follow_velocity_profiles(follow_cfg):
    if not isinstance(follow_cfg, dict):
        raise ValueError("mission.follow must be a mapping")
    profiles_cfg = follow_cfg.get("velocity_profiles")
    if not isinstance(profiles_cfg, dict):
        raise ValueError("mission.follow.velocity_profiles must be a mapping")

    profiles = {}
    for profile_name in FOLLOW_VELOCITY_PROFILE_NAMES:
        source = profiles_cfg.get(profile_name)
        if not isinstance(source, dict):
            raise ValueError(
                "mission.follow.velocity_profiles.%s must be a mapping"
                % profile_name
            )

        profile = {}
        for field_name in FOLLOW_VELOCITY_PROFILE_FIELDS:
            if field_name not in source:
                raise ValueError(
                    "missing mission.follow.velocity_profiles.%s.%s"
                    % (profile_name, field_name)
                )
            value = source[field_name]
            if isinstance(value, bool):
                raise ValueError(
                    "%s.%s must be a finite number"
                    % (profile_name, field_name)
                )
            try:
                number = float(value)
            except (TypeError, ValueError):
                raise ValueError(
                    "%s.%s must be a finite number"
                    % (profile_name, field_name)
                )
            if not math.isfinite(number):
                raise ValueError(
                    "%s.%s must be a finite number"
                    % (profile_name, field_name)
                )
            profile[field_name] = number

        for field_name in (
            "velocity_feedforward_gain",
            "position_kp",
            "relative_velocity_kd",
        ):
            if profile[field_name] < 0.0:
                raise ValueError(
                    "%s.%s must be non-negative"
                    % (profile_name, field_name)
                )

        for field_name in (
            "max_horizontal_speed_mps",
            "max_correction_speed_mps",
            "max_horizontal_accel_mps2",
            "max_horizontal_jerk_mps3",
        ):
            if profile[field_name] <= 0.0:
                raise ValueError(
                    "%s.%s must be greater than zero"
                    % (profile_name, field_name)
                )

        if (
            profile["max_correction_speed_mps"]
            > profile["max_horizontal_speed_mps"]
        ):
            raise ValueError(
                "%s.max_correction_speed_mps cannot exceed "
                "max_horizontal_speed_mps" % profile_name
            )
        profiles[profile_name] = profile

    return profiles


def calculate_follow_xy_velocity(
    profile, car_vx, car_vy, ex, ey, evx, evy
):
    ff_gain = profile["velocity_feedforward_gain"]
    kp = profile["position_kp"]
    kd = profile["relative_velocity_kd"]

    raw_correction_vx = kp * ex + kd * evx
    raw_correction_vy = kp * ey + kd * evy
    correction_vx, correction_vy = limit_xy(
        raw_correction_vx,
        raw_correction_vy,
        profile["max_correction_speed_mps"],
    )

    prelimit_vx = ff_gain * car_vx + correction_vx
    prelimit_vy = ff_gain * car_vy + correction_vy
    desired_vx, desired_vy = limit_xy(
        prelimit_vx,
        prelimit_vy,
        profile["max_horizontal_speed_mps"],
    )
    return {
        "raw_correction_vx": raw_correction_vx,
        "raw_correction_vy": raw_correction_vy,
        "correction_vx": correction_vx,
        "correction_vy": correction_vy,
        "prelimit_vx": prelimit_vx,
        "prelimit_vy": prelimit_vy,
        "desired_vx": desired_vx,
        "desired_vy": desired_vy,
    }


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
        "TAKEOFF", "DROP_HOVER", "INTERCEPT", "FOLLOW_DROP", "DROP_DESCENT", "DROP_ALIGN",
        "DROP_WAIT_ACK", "POST_DROP_FOLLOW", "FOLLOW_CD", "DYNAMIC_DESCENT",
        "PLATFORM_DISARM", "PLATFORM_TAKEOFF", "RETURN_HOME", "HOME_LAND"
    }

    VISION_STATES = {
        "INTERCEPT", "FOLLOW_DROP", "DROP_DESCENT", "DROP_ALIGN", "DROP_WAIT_ACK",
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
        try:
            self.follow_velocity_profiles = load_follow_velocity_profiles(
                self.follow_cfg
            )
        except ValueError as exc:
            rospy.logfatal("Invalid follow velocity profiles: %s", str(exc))
            raise
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
        self.fixed_yaw = math.radians(safe_float(self.takeoff_cfg.get("yaw_deg", 0.0), 0.0))

        self.telemetry_timeout = safe_float(self.est_cfg.get("telemetry_timeout_s", 0.50), 0.50)
        self.vision_timeout = safe_float(self.est_cfg.get("vision_timeout_s", 0.35), 0.35)
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
        self.fuse_synthetic_vision = bool(
            self.est_cfg.get("fuse_synthetic_vision", False)
        )
        self.curve_prediction_enabled = bool(
            self.est_cfg.get("curve_prediction_enabled", True)
        )
        self.curve_prediction_radius = max(
            0.05,
            safe_float(self.est_cfg.get("curve_prediction_radius_m", 0.75), 0.75),
        )
        self.curve_turn_direction = 1.0 if safe_float(
            self.est_cfg.get("curve_turn_direction", 1.0), 1.0
        ) >= 0.0 else -1.0

        self.current_state = State()
        self.extended_state = ExtendedState()
        self.extended_state_received = False
        self.current_pose = None
        self.current_vel = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        self.last_pose_rx = None
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
        self.reset_required = False

        self.fsm_state = "WAIT_START"
        self.state_enter_time = rospy.Time.now()
        self.follow_stable_since = None
        self.drop_stable_since = None
        self.drop_descent_stable_since = None
        self.touchdown_since = None
        self.home_stable_since = None
        self.post_drop_recover_since = None
        self.home_land_stable_since = None
        self.takeoff_stable_since = None
        # 任务一：首次满足起飞位置/速度判稳条件时，立即进入3秒悬停计时。
        # 进入 DROP_HOVER 后不再要求稳定条件持续成立，3秒到点即开始截获。
        self.drop_hover_started_by_stable = False

        # 最近一次移动目标控制指标，供地面站显示相对位置/相对速度误差。
        self.last_follow_metrics = None
        self.last_follow_metrics_time = None
        # 最近一次投放目标偏置分解，供状态遥测和投放触发日志使用。
        self.last_drop_lead = None

        # 截获阶段自适应预测时间状态。预测距离使用未外推的小车当前位置计算，
        # 预测时间则在远/近距离参数之间连续变化，并经过变化率限制。
        self.intercept_prediction_filtered_s = None
        self.intercept_prediction_raw_s = None
        self.intercept_current_distance_m = None

        self.last_mode_request = rospy.Time(0)
        self.last_arm_request = rospy.Time(0)
        self.wait_fcu_prestream_start = None
        self.auto_land_start = None
        self.auto_land_prepare_start = None

        self.cmd_xy = [0.0, 0.0]
        self.cmd_acc_xy = [0.0, 0.0]
        self.cmd_vz = 0.0

        self.landing_target_z = None
        self.home_land_target_z = None
        self.drop_target_z = None
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
        self.mission_start_time = rospy.Time.now()
        self.mission_result = "RUNNING"
        self.safety_state = "NORMAL"
        self.wait_fcu_prestream_start = rospy.Time.now()
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
        self.cmd_acc_xy = [0.0, 0.0]
        self.cmd_vz = 0.0
        self.follow_stable_since = None
        self.drop_stable_since = None
        self.drop_descent_stable_since = None
        self.touchdown_since = None
        self.home_stable_since = None
        self.home_land_stable_since = None
        self.takeoff_stable_since = None
        self.drop_hover_started_by_stable = False
        self.last_follow_metrics = None
        self.last_follow_metrics_time = None
        self.last_drop_lead = None
        self.intercept_prediction_filtered_s = None
        self.intercept_prediction_raw_s = None
        self.intercept_current_distance_m = None
        self.landing_target_z = None
        self.home_land_target_z = None
        self.drop_target_z = None
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
        # 每次切换任务状态都重新累计伴飞稳定时间，避免上一状态的计时直接穿透。
        self.follow_stable_since = None
        if new_state != "DROP_ALIGN":
            self.drop_stable_since = None
        if new_state != "DROP_DESCENT":
            self.drop_descent_stable_since = None
            self.drop_target_z = None
        if new_state != "DYNAMIC_DESCENT":
            self.touchdown_since = None
        if new_state == "INTERCEPT":
            # 每次重新进入截获阶段时，让预测时间从当前距离对应值开始，
            # 避免沿用上一次任务或上一次截获的滤波状态。
            self.intercept_prediction_filtered_s = None
            self.intercept_prediction_raw_s = None
            self.intercept_current_distance_m = None
        if new_state in {"FOLLOW_DROP", "FOLLOW_CD"}:
            self.publish_event_once("UAV_FOLLOW_ESTABLISHED")
        elif new_state == "PLATFORM_DWELL":
            self.publish_event_once("UAV_LAND_ON_CAR")
        elif new_state == "HOME_LAND":
            self.home_land_stable_since = None
            self.auto_land_prepare_start = None
            self.auto_land_start = None
            self.home_land_target_z = (
                self.current_pose.pose.position.z
                if self.current_pose is not None
                else self.home_z + safe_float(self.return_cfg.get("height_m", 1.20), 1.20)
            )
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

    def publish_position_xy_velocity_xyz_yaw(self, x, y, vx, vy, vz, yaw):
        """XY由PX4位置环+速度前馈控制，Z只使用本节点限速后的速度闭环。"""
        msg = self.make_target_msg()
        msg.type_mask = (
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE
        )
        msg.position.x = x
        msg.position.y = y
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz
        msg.yaw = yaw
        self.raw_pub.publish(msg)

    def update_xy_command(self, desired_vx, desired_vy, dt, max_accel, max_jerk):
        """Jerk受限的二维速度命令更新，防止半圆上加速度方向逐帧突变。"""
        dt_eff = max(float(dt), 0.01)
        desired_ax = (float(desired_vx) - self.cmd_xy[0]) / dt_eff
        desired_ay = (float(desired_vy) - self.cmd_xy[1]) / dt_eff
        desired_ax, desired_ay = limit_xy(desired_ax, desired_ay, max_accel)
        self.cmd_acc_xy = limit_xy_change(
            self.cmd_acc_xy,
            [desired_ax, desired_ay],
            max_jerk * dt_eff,
        )
        ax, ay = limit_xy(self.cmd_acc_xy[0], self.cmd_acc_xy[1], max_accel)
        self.cmd_acc_xy = [ax, ay]

        error_x = float(desired_vx) - self.cmd_xy[0]
        error_y = float(desired_vy) - self.cmd_xy[1]
        delta_x = ax * dt_eff
        delta_y = ay * dt_eff
        error_norm = norm2(error_x, error_y)
        delta_norm = norm2(delta_x, delta_y)
        if delta_norm > error_norm > 1e-9:
            scale = error_norm / delta_norm
            delta_x *= scale
            delta_y *= scale
            self.cmd_acc_xy = [delta_x / dt_eff, delta_y / dt_eff]

        self.cmd_xy[0] += delta_x
        self.cmd_xy[1] += delta_y
        return self.cmd_xy

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
            "source": str(self.car_data.get("source", "")),
            "path_s": self.car_data.get("path_s"),
        }

    def vision_abs_measurement(self):
        if not self.vision_valid():
            return None

        # 融合视觉节点已经按图像采集时刻匹配 UAV 位姿，并给出 MAVROS local
        # 绝对坐标。优先使用该结果，避免推理延迟期间无人机移动造成二次换算误差。
        if bool(self.vision_data.get("local_position_valid", False)):
            target_x = self.vision_data.get("target_x_local")
            target_y = self.vision_data.get("target_y_local")
            if target_x is not None and target_y is not None:
                return {
                    "x": safe_float(target_x, 0.0),
                    "y": safe_float(target_y, 0.0),
                    "confidence": safe_float(self.vision_data.get("confidence", 0.0)),
                    "stable_count": safe_int(self.vision_data.get("stable_count", 0)),
                }

        # 兼容旧视觉消息：仅有机体系 forward/left 时，使用当前位姿换算。
        if self.current_pose is None:
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

    def predict_car_motion(self, base, telemetry, prediction_s):
        result = dict(base)
        p = max(0.0, float(prediction_s))
        segment = str(telemetry.get("segment", "UNKNOWN")).upper() if telemetry else "UNKNOWN"
        if (
            self.curve_prediction_enabled and telemetry is not None and
            segment in {"BC", "DA"} and p > 1e-6
        ):
            speed = norm2(base["vx"], base["vy"])
            yaw = safe_float(telemetry.get("yaw", 0.0), 0.0)
            omega = self.curve_turn_direction * speed / self.curve_prediction_radius
            if speed > 1e-4 and abs(omega) > 1e-6:
                next_yaw = yaw + omega * p
                result["x"] = base["x"] + speed / omega * (
                    math.sin(next_yaw) - math.sin(yaw)
                )
                result["y"] = base["y"] - speed / omega * (
                    math.cos(next_yaw) - math.cos(yaw)
                )
                result["vx"] = speed * math.cos(next_yaw)
                result["vy"] = speed * math.sin(next_yaw)
                result["prediction_model"] = "constant_curvature"
                return result
        result["x"] = base["x"] + base["vx"] * p
        result["y"] = base["y"] + base["vy"] * p
        result["prediction_model"] = "constant_velocity"
        return result

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
        vision_source = str(self.vision_data.get("source", "")).lower()
        synthetic_duplicate = (
            vision is not None and telemetry is not None
            and "synthetic" in vision_source and not self.fuse_synthetic_vision
        )
        if vision is not None and self.vision_version != self.last_processed_vision_version:
            if synthetic_duplicate:
                # 假小车里程和合成视觉来自同一几何真值。视觉仍用于“已锁定/可下降”门槛，
                # 但不再第二次修正 Alpha-Beta 跟踪器，避免速度估计被重复测量扰动。
                self.last_processed_vision_version = self.vision_version
            elif not self.tracker.initialized:
                self.tracker.initialize(vision["x"], vision["y"], 0.0, 0.0, now_sec)
                self.last_processed_vision_version = self.vision_version
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
        base = self.tracker.snapshot(0.0)
        result = self.predict_car_motion(base, telemetry, prediction)
        result["prediction_s"] = prediction
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
                if norm2(result["vx"], result["vy"]) > 0.03 else self.fixed_yaw,
                "segment": str(self.car_data.get("segment", "UNKNOWN")).upper(),
                "segment_progress": safe_float(self.car_data.get("segment_progress", -1.0)),
                "running": bool(self.car_data.get("running", True)),
            })
        self.car_estimate = result
        return deepcopy(result)

    def drop_uav_center_offset(self):
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
            yaw = self.fixed_yaw
            release_local_x = math.cos(yaw) * release_x - math.sin(yaw) * release_y
            release_local_y = math.sin(yaw) * release_x + math.cos(yaw) * release_y

        return -release_local_x, -release_local_y

    def drop_target_offset(self, car):
        """Combine physical outlet compensation with signed empirical lead."""
        physical_x, physical_y = self.drop_uav_center_offset()
        lead = calculate_empirical_release_lead(
            car.get("vx", 0.0),
            car.get("vy", 0.0),
            self.drop_cfg.get("empirical_release_lead_distance_m", 0.0),
        )
        combined_x = physical_x + lead["lead_x"]
        combined_y = physical_y + lead["lead_y"]
        self.last_drop_lead = dict(lead)
        self.last_drop_lead.update({
            "physical_offset_x": physical_x,
            "physical_offset_y": physical_y,
            "combined_offset_x": combined_x,
            "combined_offset_y": combined_y,
        })
        if (
            not lead["direction_valid"]
            and abs(lead["configured_distance_m"]) > 1e-6
        ):
            rospy.logwarn_throttle(
                1.0,
                "Empirical drop lead disabled: car speed %.3fm/s is below 0.05m/s.",
                lead["speed_mps"],
            )
        return combined_x, combined_y

    def drop_platform_z(self):
        """返回投放平台表面在 MAVROS local Z 中的高度。"""
        platform_height = safe_float(
            self.drop_cfg.get(
                "platform_height_m",
                self.land_cfg.get("platform_height_m", 0.34),
            ),
            0.34,
        )
        return self.home_z + platform_height

    def drop_final_target_z(self):
        """
        返回任务一投放前的最终目标高度。

        target_height_above_platform_m 表示无人机控制点相对小车平台表面的
        垂直间距，而不是相对 H 点地面的绝对高度。这样平台本身有高度时，
        不会把 0.35m 错当成距地面 0.35m 而贴到平台表面。
        """
        clearance = safe_float(
            self.drop_cfg.get("target_height_above_platform_m", 0.35),
            0.35,
        )
        return self.drop_platform_z() + max(0.0, clearance)

    def car_target_xy(self, car):
        # 跟随偏置始终按任务固定航向解释，小车 yaw 不参与目标位置计算。
        yaw = self.fixed_yaw
        forward_offset = safe_float(self.follow_cfg.get("offset_forward_m", 0.0), 0.0)
        left_offset = safe_float(self.follow_cfg.get("offset_left_m", 0.0), 0.0)
        tx = (
            car["x"] + math.cos(yaw) * forward_offset - math.sin(yaw) * left_offset
        )
        ty = (
            car["y"] + math.sin(yaw) * forward_offset + math.cos(yaw) * left_offset
        )
        return tx, ty

    def publish_dynamic_follow_control(
        self, car, target_z, dt, speed_scale=1.0, target_offset_xy=None,
        max_vz_override=None, max_az_override=None
    ):
        """动态降落专用：保持PX4位置环+小车速度前馈的原控制行为。"""
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
        desired_vx = ff * car["vx"]
        desired_vy = ff * car["vy"]
        max_speed = safe_float(
            self.follow_cfg.get("max_horizontal_speed_mps", 1.20), 1.20
        ) * speed_scale
        desired_vx, desired_vy = limit_xy(desired_vx, desired_vy, max_speed)
        max_acc = safe_float(
            self.follow_cfg.get("max_horizontal_accel_mps2", 0.55), 0.55
        )
        max_jerk = safe_float(
            self.follow_cfg.get("max_horizontal_jerk_mps3", 0.90), 0.90
        )
        self.update_xy_command(desired_vx, desired_vy, dt, max_acc, max_jerk)

        z_error = target_z - p.z
        z_kp = safe_float(self.follow_cfg.get("vertical_kp", 0.90), 0.90)
        max_vz = (
            safe_float(max_vz_override, 0.45)
            if max_vz_override is not None
            else safe_float(self.follow_cfg.get("max_vertical_speed_mps", 0.45), 0.45)
        )
        desired_vz = clamp(z_kp * z_error, -max_vz, max_vz)
        max_az = (
            safe_float(max_az_override, 0.60)
            if max_az_override is not None
            else safe_float(self.follow_cfg.get("max_vertical_accel_mps2", 0.60), 0.60)
        )
        self.cmd_vz += clamp(desired_vz - self.cmd_vz, -max_az * dt, max_az * dt)

        yaw = self.fixed_yaw
        self.publish_position_xy_velocity_xyz_yaw(
            tx, ty,
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
            "command_vx": self.cmd_xy[0],
            "command_vy": self.cmd_xy[1],
            "command_ax": self.cmd_acc_xy[0],
            "command_ay": self.cmd_acc_xy[1],
            "prediction_model": car.get("prediction_model", "unknown"),
            "prediction_s": car.get("prediction_s", 0.0),
        }
        self.last_follow_metrics = deepcopy(metrics)
        self.last_follow_metrics_time = rospy.Time.now()
        return metrics

    def publish_drop_follow_control(
        self, car, target_z, dt, profile_name, target_offset_xy=None,
        max_vz_override=None, max_az_override=None
    ):
        """投掷任务专用：ROS前馈+PD外环计算并发布完整XY速度。"""
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

        profile = self.follow_velocity_profiles[profile_name]
        control = calculate_follow_xy_velocity(
            profile,
            car["vx"],
            car["vy"],
            ex,
            ey,
            evx,
            evy,
        )

        previous_cmd_xy = list(self.cmd_xy)
        self.update_xy_command(
            control["desired_vx"],
            control["desired_vy"],
            dt,
            profile["max_horizontal_accel_mps2"],
            profile["max_horizontal_jerk_mps3"],
        )
        limited_vx, limited_vy = limit_xy(
            self.cmd_xy[0],
            self.cmd_xy[1],
            profile["max_horizontal_speed_mps"],
        )
        self.cmd_xy = [limited_vx, limited_vy]
        dt_eff = max(float(dt), 0.01)
        self.cmd_acc_xy = [
            (self.cmd_xy[0] - previous_cmd_xy[0]) / dt_eff,
            (self.cmd_xy[1] - previous_cmd_xy[1]) / dt_eff,
        ]

        z_error = target_z - p.z
        z_kp = safe_float(self.follow_cfg.get("vertical_kp", 0.90), 0.90)
        max_vz = (
            safe_float(max_vz_override, 0.45)
            if max_vz_override is not None
            else safe_float(self.follow_cfg.get("max_vertical_speed_mps", 0.45), 0.45)
        )
        desired_vz = clamp(z_kp * z_error, -max_vz, max_vz)
        max_az = (
            safe_float(max_az_override, 0.60)
            if max_az_override is not None
            else safe_float(self.follow_cfg.get("max_vertical_accel_mps2", 0.60), 0.60)
        )
        self.cmd_vz += clamp(desired_vz - self.cmd_vz, -max_az * dt, max_az * dt)

        self.publish_velocity_yaw(
            self.cmd_xy[0],
            self.cmd_xy[1],
            self.cmd_vz,
            self.fixed_yaw,
        )

        measured_speed = norm2(self.current_vel[0], self.current_vel[1])
        if measured_speed > profile["max_horizontal_speed_mps"] + 0.05:
            rospy.logwarn_throttle(
                1.0,
                "Measured XY speed %.3f exceeds %s command limit %.3f",
                measured_speed,
                profile_name,
                profile["max_horizontal_speed_mps"],
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
            "command_vx": self.cmd_xy[0],
            "command_vy": self.cmd_xy[1],
            "command_ax": self.cmd_acc_xy[0],
            "command_ay": self.cmd_acc_xy[1],
            "prediction_model": car.get("prediction_model", "unknown"),
            "prediction_s": car.get("prediction_s", 0.0),
            "control_profile": profile_name,
            "raw_correction_vx": control["raw_correction_vx"],
            "raw_correction_vy": control["raw_correction_vy"],
            "correction_vx": control["correction_vx"],
            "correction_vy": control["correction_vy"],
            "prelimit_total_vx": control["prelimit_vx"],
            "prelimit_total_vy": control["prelimit_vy"],
            "desired_vx": control["desired_vx"],
            "desired_vy": control["desired_vy"],
            "measured_horizontal_speed": measured_speed,
            "command_speed_limit": profile["max_horizontal_speed_mps"],
        }
        self.last_follow_metrics = deepcopy(metrics)
        self.last_follow_metrics_time = rospy.Time.now()
        return metrics

    def publish_point_control(
        self, tx, ty, tz, dt, max_speed, max_accel, kp,
        max_jerk=None, max_vz=None, max_az=None, vertical_kp=None
    ):
        if self.current_pose is None:
            return None
        p = self.current_pose.pose.position
        ex = tx - p.x
        ey = ty - p.y
        desired_vx, desired_vy = limit_xy(kp * ex, kp * ey, max_speed)
        jerk_limit = safe_float(
            max_jerk,
            safe_float(self.follow_cfg.get("max_horizontal_jerk_mps3", 0.90), 0.90),
        )
        self.update_xy_command(
            desired_vx, desired_vy, dt, max_accel, jerk_limit
        )
        z_kp = (
            safe_float(vertical_kp, 0.90)
            if vertical_kp is not None
            else safe_float(self.follow_cfg.get("vertical_kp", 0.90), 0.90)
        )
        vz_limit = safe_float(
            max_vz,
            safe_float(self.follow_cfg.get("max_vertical_speed_mps", 0.45), 0.45),
        )
        desired_vz = clamp(z_kp * (tz - p.z), -vz_limit, vz_limit)
        az_limit = safe_float(
            max_az,
            safe_float(self.follow_cfg.get("max_vertical_accel_mps2", 0.60), 0.60),
        )
        self.cmd_vz += clamp(
            desired_vz - self.cmd_vz,
            -az_limit * max(dt, 0.01),
            az_limit * max(dt, 0.01),
        )
        # 固定点阶段使用纯速度外环，避免“位置目标+由位置误差算出的速度”重复闭环。
        self.publish_velocity_yaw(
            self.cmd_xy[0], self.cmd_xy[1], self.cmd_vz, self.fixed_yaw
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

    def start_drop_action(self, trigger_reason):
        """统一启动一次投放动作，避免正常瞄准与超时强投逻辑重复。"""
        self.drop_action_id += 1
        self.drop_retry_count = 0
        self.drop_ack_success = False
        self.drop_ack_text = ""
        lead = self.last_drop_lead or {}
        rospy.logwarn(
            "Drop action triggered: reason=%s action_id=%d "
            "distance=%.3f speed=%.3f direction=(%.3f,%.3f) "
            "lead=(%.3f,%.3f) combined=(%.3f,%.3f)",
            str(trigger_reason),
            self.drop_action_id,
            safe_float(lead.get("configured_distance_m", 0.0), 0.0),
            safe_float(lead.get("speed_mps", 0.0), 0.0),
            safe_float(lead.get("direction_x", 0.0), 0.0),
            safe_float(lead.get("direction_y", 0.0), 0.0),
            safe_float(lead.get("lead_x", 0.0), 0.0),
            safe_float(lead.get("lead_y", 0.0), 0.0),
            safe_float(lead.get("combined_offset_x", 0.0), 0.0),
            safe_float(lead.get("combined_offset_y", 0.0), 0.0),
        )
        self.send_drop_command()
        self.enter_state("DROP_WAIT_ACK")

    def send_drop_command(self):
        payload = {
            "cmd": str(self.drop_cfg.get("command", "A90")),
            "mission_id": self.mission_id,
            "source": "fsm",
            "manual": False,
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
        self.cmd_xy = [self.current_vel[0], self.current_vel[1]]
        self.cmd_acc_xy = [0.0, 0.0]
        self.cmd_vz = self.current_vel[2]
        self.home_stable_since = None
        self.enter_state("RETURN_HOME")

    def publish_takeoff_point_control(self, dt):
        target_z = self.home_z + self.cruise_height
        return self.publish_point_control(
            self.home_x,
            self.home_y,
            target_z,
            dt,
            safe_float(
                self.takeoff_cfg.get("max_horizontal_speed_mps", 0.35),
                0.35,
            ),
            safe_float(
                self.takeoff_cfg.get("max_horizontal_accel_mps2", 0.35),
                0.35,
            ),
            safe_float(
                self.takeoff_cfg.get("horizontal_position_kp", 0.80),
                0.80,
            ),
            max_jerk=safe_float(
                self.takeoff_cfg.get("max_horizontal_jerk_mps3", 0.60),
                0.60,
            ),
            max_vz=safe_float(
                self.takeoff_cfg.get("max_vertical_speed_mps", 0.55),
                0.55,
            ),
            max_az=safe_float(
                self.takeoff_cfg.get("max_vertical_accel_mps2", 0.75),
                0.75,
            ),
            vertical_kp=safe_float(
                self.takeoff_cfg.get("vertical_position_kp", 0.90),
                0.90,
            ),
        )

    def handle_takeoff(self, dt):
        metrics = self.publish_takeoff_point_control(dt)
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
        # 3秒后若宽松安全门槛未满足则继续悬停；门槛恢复后立即截获，不重新计时。
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

    def handle_drop_hover(self, dt):
        metrics = self.publish_takeoff_point_control(dt)
        if metrics is None:
            return
        required_hover = safe_float(
            self.takeoff_cfg.get("drop_hover_time_s", 3.0), 3.0
        )
        if self.state_elapsed() < required_hover:
            return

        xy_err, z_err, speed = metrics
        exit_safe = (
            xy_err <= safe_float(
                self.takeoff_cfg.get(
                    "hover_exit_xy_tolerance_m", 0.30
                ),
                0.30,
            )
            and z_err <= safe_float(
                self.takeoff_cfg.get(
                    "hover_exit_z_tolerance_m", 0.18
                ),
                0.18,
            )
            and speed <= safe_float(
                self.takeoff_cfg.get(
                    "hover_exit_speed_tolerance_mps", 0.35
                ),
                0.35,
            )
        )
        if exit_safe:
            self.enter_state("INTERCEPT")
            return

        rospy.logwarn_throttle(
            1.0,
            "Task1 hover timer done; wait for safe intercept exit: "
            "xy_err=%.2f z_err=%.2f speed=%.2f",
            xy_err,
            z_err,
            speed,
        )

    def adaptive_intercept_prediction_s(self, distance_m, dt):
        """根据无人机到小车当前估计位置的距离连续调整截获预测时间。"""
        far_prediction = safe_float(
            self.follow_cfg.get("intercept_prediction_far_s", 0.20),
            0.20,
        )
        near_prediction = safe_float(
            self.follow_cfg.get("intercept_prediction_near_s", 0.05),
            0.05,
        )
        far_distance = safe_float(
            self.follow_cfg.get("intercept_prediction_far_distance_m", 0.80),
            0.80,
        )
        near_distance = safe_float(
            self.follow_cfg.get("intercept_prediction_near_distance_m", 0.50),
            0.50,
        )

        # 配置异常时退回原固定预测时间，避免除零或反向插值。
        if far_distance <= near_distance:
            raw_prediction = safe_float(
                self.follow_cfg.get("intercept_prediction_s", far_prediction),
                far_prediction,
            )
            rospy.logwarn_throttle(
                2.0,
                "Invalid adaptive intercept distances: far=%.3f near=%.3f; "
                "fallback prediction=%.3fs",
                far_distance,
                near_distance,
                raw_prediction,
            )
        else:
            ratio = clamp(
                (safe_float(distance_m, far_distance) - near_distance)
                / (far_distance - near_distance),
                0.0,
                1.0,
            )
            raw_prediction = (
                near_prediction
                + ratio * (far_prediction - near_prediction)
            )

        prediction_low = min(near_prediction, far_prediction)
        prediction_high = max(near_prediction, far_prediction)
        raw_prediction = clamp(raw_prediction, prediction_low, prediction_high)
        self.intercept_prediction_raw_s = raw_prediction

        slew_rate = max(
            0.0,
            safe_float(
                self.follow_cfg.get(
                    "intercept_prediction_slew_rate_s_per_s",
                    0.50,
                ),
                0.50,
            ),
        )
        if self.intercept_prediction_filtered_s is None:
            self.intercept_prediction_filtered_s = raw_prediction
        elif slew_rate <= 1e-9:
            self.intercept_prediction_filtered_s = raw_prediction
        else:
            max_change = slew_rate * max(float(dt), 0.01)
            delta = clamp(
                raw_prediction - self.intercept_prediction_filtered_s,
                -max_change,
                max_change,
            )
            self.intercept_prediction_filtered_s += delta

        self.intercept_prediction_filtered_s = clamp(
            self.intercept_prediction_filtered_s,
            prediction_low,
            prediction_high,
        )
        return self.intercept_prediction_filtered_s

    def handle_intercept(self, dt):
        # 先更新并取得“未向前外推”的小车当前融合位置。自适应预测所用距离
        # 必须基于当前点，不能使用已经预测后的目标误差，否则会形成反馈耦合。
        car_now = self.update_car_estimator(0.0)
        if car_now is None:
            self.intercept_current_distance_m = None
            self.publish_position_velocity_yaw(
                self.home_x, self.home_y, self.home_z + self.cruise_height,
                0.0, 0.0, 0.0, self.fixed_yaw,
            )
            return

        if self.current_pose is None:
            return
        p = self.current_pose.pose.position
        current_distance = norm2(car_now["x"] - p.x, car_now["y"] - p.y)
        self.intercept_current_distance_m = current_distance

        prediction_s = self.adaptive_intercept_prediction_s(
            current_distance,
            dt,
        )
        telemetry = self.car_abs_measurement()
        car = self.predict_car_motion(car_now, telemetry, prediction_s)
        car["prediction_s"] = prediction_s
        # predict_car_motion() 从 car_now 复制赛段、航向等字段；这里只覆盖
        # 当前截获使用的预测结果，供控制与 /uav/mission_status 同步查看。
        self.car_estimate = deepcopy(car)

        metrics = self.publish_drop_follow_control(
            car, self.home_z + self.cruise_height, dt, "intercept"
        )
        rospy.loginfo_throttle(
            0.5,
            "Adaptive intercept: current_distance=%.3fm "
            "prediction_raw=%.3fs prediction_used=%.3fs "
            "target_error=%.3fm relative_speed=%.3fm/s",
            current_distance,
            safe_float(self.intercept_prediction_raw_s, prediction_s),
            prediction_s,
            metrics["position_error"] if metrics is not None else -1.0,
            metrics["relative_speed"] if metrics is not None else -1.0,
        )
        if self.follow_is_stable(metrics, strict=False):
            self.enter_state("FOLLOW_DROP" if self.mission_type == "drop" else "FOLLOW_CD")

    def handle_follow_drop(self, dt):
        """
        任务一高空伴飞等待投放。

        无人机可以在 AB、BC 甚至进入 CD 前就完成截获并保持稳定，但这些
        赛段绝不允许预累计“进入投放下降”的稳定时间。只有进入配置的 CD
        下降窗口后，才从零开始重新判断严格伴飞条件；连续满足后才进入
        DROP_DESCENT。
        """
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is None:
            return

        metrics = self.publish_drop_follow_control(
            car, self.home_z + self.cruise_height, dt, "follow"
        )

        required_segment = str(
            self.drop_cfg.get("segment_name", "CD")
        ).upper()
        descent_start = safe_float(
            self.drop_cfg.get("descent_window_start", 0.05), 0.05
        )
        window_end = safe_float(
            self.drop_cfg.get("window_end", 0.78), 0.78
        )
        car_segment = str(car.get("segment", "UNKNOWN")).upper()
        progress = safe_float(car.get("segment_progress", -1.0), -1.0)

        segment_ok = car_segment == required_segment
        entry_window_ok = (
            segment_ok and descent_start <= progress <= window_end
        )

        # 将门控状态写入任务状态，实飞时可直接查看为什么尚未进入下降。
        if metrics is not None:
            metrics.update({
                "drop_entry_required_segment": required_segment,
                "drop_entry_car_segment": car_segment,
                "drop_entry_segment_progress": progress,
                "drop_entry_segment_ok": segment_ok,
                "drop_entry_window_ok": entry_window_ok,
                "drop_entry_waiting_for_cd": not entry_window_ok,
            })
            self.last_follow_metrics = deepcopy(metrics)
            self.last_follow_metrics_time = rospy.Time.now()

        if not entry_window_ok:
            # 核心保证：AB/BC 中即使位置、速度和视觉全部满足，也不能把
            # 稳定计时带入 CD。进入 CD 后必须重新连续满足 strict 条件。
            self.follow_stable_since = None

            if segment_ok and progress > window_end:
                self.enter_return_home("DROP_WINDOW_MISSED")
                return

            rospy.loginfo_throttle(
                0.5,
                "Drop entry gated: waiting for %s window %.2f..%.2f; "
                "car_segment=%s progress=%.3f. Keep following at cruise height.",
                required_segment,
                descent_start,
                window_end,
                car_segment,
                progress,
            )
            return

        # 进入 CD 指定窗口后才开始重新累计严格伴飞稳定时间。
        stable_in_cd = self.follow_is_stable(metrics, strict=True)
        rospy.loginfo_throttle(
            0.5,
            "Drop entry in %s: progress=%.3f pos_err=%.3f rel_speed=%.3f "
            "vision=%s stable=%s",
            required_segment,
            progress,
            metrics["position_error"] if metrics is not None else -1.0,
            metrics["relative_speed"] if metrics is not None else -1.0,
            self.vision_valid(),
            stable_in_cd,
        )
        if not stable_in_cd:
            return

        self.drop_target_z = max(
            self.drop_final_target_z(),
            min(
                self.current_pose.pose.position.z,
                self.home_z + self.cruise_height,
            ),
        )
        self.enter_state("DROP_DESCENT")

    def handle_drop_descent(self, dt):
        """任务一：保持 XY 跟车，同时逐步降低 Z 目标到投放高度。"""
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is None:
            return

        segment = str(self.drop_cfg.get("segment_name", "CD")).upper()
        window_start = safe_float(self.drop_cfg.get("window_start", 0.20), 0.20)
        window_end = safe_float(self.drop_cfg.get("window_end", 0.78), 0.78)
        car_segment = str(car.get("segment", "")).upper()
        progress = safe_float(car.get("segment_progress", -1.0))

        if car_segment != segment:
            self.enter_return_home("DROP_DESCENT_SEGMENT_LOST")
            return

        # 保留之前确定的投放兜底：若下降阶段已经耗尽 C-D 投放窗口，
        # 仍在 C-D 内时按当前位置直接投放，避免整轮任务完全错过。
        if progress > window_end:
            rospy.logwarn(
                "Drop descent reached window deadline at progress=%.3f; "
                "force drop at current height z=%.3f.",
                progress,
                self.current_pose.pose.position.z,
            )
            self.start_drop_action("DESCENT_WINDOW_DEADLINE_FORCE_DROP")
            return

        final_z = self.drop_final_target_z()
        if self.drop_target_z is None:
            self.drop_target_z = max(
                final_z,
                min(
                    self.current_pose.pose.position.z,
                    self.home_z + self.cruise_height,
                ),
            )

        drop_offset = self.drop_target_offset(car)
        metrics_preview = self.follow_metrics(car, target_offset_xy=drop_offset)
        vision_ok = self.vision_valid()
        vision_age = self.vision_age()

        if not vision_ok and vision_age >= safe_float(
            self.drop_cfg.get("descent_vision_abort_timeout_s", 0.60), 0.60
        ):
            rospy.logerr(
                "Drop descent vision lost %.2fs; return to high-altitude follow.",
                vision_age,
            )
            self.enter_state("FOLLOW_DROP")
            return

        can_descend = (
            vision_ok and
            metrics_preview is not None and
            metrics_preview["position_error"] <= safe_float(
                self.drop_cfg.get("descent_pause_position_error_m", 0.14), 0.14
            ) and
            metrics_preview["relative_speed"] <= safe_float(
                self.drop_cfg.get("descent_pause_relative_speed_mps", 0.12), 0.12
            )
        )

        if can_descend:
            remaining = max(0.0, self.drop_target_z - final_z)
            slow_band = safe_float(
                self.drop_cfg.get("descent_slow_height_m", 0.25), 0.25
            )
            rate = safe_float(
                self.drop_cfg.get(
                    "descent_final_rate_mps" if remaining <= slow_band
                    else "descent_rate_mps",
                    0.10 if remaining <= slow_band else 0.20,
                ),
                0.10 if remaining <= slow_band else 0.20,
            )
            self.drop_target_z = max(final_z, self.drop_target_z - rate * dt)

        metrics = self.publish_drop_follow_control(
            car,
            self.drop_target_z,
            dt,
            "drop_descent",
            target_offset_xy=drop_offset,
        )

        z_reached = (
            abs(self.current_pose.pose.position.z - final_z) <= safe_float(
                self.drop_cfg.get("descent_z_tolerance_m", 0.05), 0.05
            ) and
            abs(self.current_vel[2]) <= safe_float(
                self.drop_cfg.get("descent_vertical_speed_mps", 0.12), 0.12
            )
        )
        tracking_ok = (
            vision_ok and metrics is not None and
            metrics["position_error"] <= safe_float(
                self.drop_cfg.get("descent_pause_position_error_m", 0.14), 0.14
            ) and
            metrics["relative_speed"] <= safe_float(
                self.drop_cfg.get("descent_pause_relative_speed_mps", 0.12), 0.12
            )
        )
        ready = self.update_stable_timer(
            z_reached and tracking_ok and progress >= window_start,
            "drop_descent_stable_since",
            safe_float(
                self.drop_cfg.get("descent_reached_stable_time_s", 0.30), 0.30
            ),
        )
        if ready:
            self.enter_state("DROP_ALIGN")

    def handle_drop_align(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is None:
            return
        drop_offset = self.drop_target_offset(car)
        metrics = self.publish_drop_follow_control(
            car, self.drop_final_target_z(), dt, "drop_align",
            target_offset_xy=drop_offset,
        )
        segment = self.drop_cfg.get("segment_name", "CD")
        end = safe_float(self.drop_cfg.get("window_end", 0.78), 0.78)
        car_segment = str(car.get("segment", "")).upper()
        segment_progress = safe_float(car.get("segment_progress", -1.0))

        # 离开 C-D 后禁止投放；仍在 C-D 但到达窗口末端时直接投放，
        # 保留当前任务的最后兜底，不再通过额外配置开关控制。
        if car_segment != str(segment).upper():
            self.enter_return_home("DROP_ALIGN_SEGMENT_LOST")
            return
        if segment_progress > end:
            rospy.logwarn(
                "Drop alignment window deadline reached at progress=%.3f: force drop.",
                segment_progress,
            )
            self.start_drop_action("ALIGN_WINDOW_DEADLINE_FORCE_DROP")
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
            self.start_drop_action("ALIGN_READY")
            return

        # 精确瞄准超时后直接投放。上方已确认仍处于 C-D 且未超过窗口末端。
        align_timeout = safe_float(self.drop_cfg.get("align_timeout_s", 1.00), 1.00)
        if align_timeout > 0.0 and self.state_elapsed() >= align_timeout:
            rospy.logwarn(
                "Drop alignment timeout %.2fs: force drop in valid window; "
                "pos_err=%.3f rel_speed=%.3f predicted_err=%.3f vision=%s",
                align_timeout,
                metrics["position_error"],
                metrics["relative_speed"],
                predicted_error,
                vision_ok,
            )
            self.start_drop_action("ALIGN_TIMEOUT_FORCE_DROP")

    def handle_drop_wait_ack(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is not None:
            self.publish_drop_follow_control(
                car, self.drop_final_target_z(), dt, "drop_align",
                target_offset_xy=self.drop_target_offset(car),
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
        """
        投放完成后的恢复阶段：
        1. 不再继续伴飞；
        2. XY方向使用jerk/加速度限制平滑减速到0；
        3. Z轴匀速抬升到指定恢复高度；
        4. XY速度稳定0.4s后才进入返航。
        """
        # 1) XY平滑刹停
        self.update_xy_command(
            0.0,
            0.0,
            dt,
            safe_float(
                self.drop_cfg.get(
                    "post_drop_xy_brake_accel_mps2", 0.35
                ),
                0.35,
            ),
            safe_float(
                self.drop_cfg.get(
                    "post_drop_xy_brake_jerk_mps3", 0.80
                ),
                0.80,
            ),
        )

        # 2) Z抬升到恢复高度
        recover_z = self.home_z + safe_float(
            self.drop_cfg.get(
                "post_drop_recover_height_m",
                self.cruise_height,
            ),
            self.cruise_height,
        )

        z_error = recover_z - self.current_pose.pose.position.z
        max_vz = safe_float(
            self.drop_cfg.get(
                "post_drop_climb_speed_mps",
                0.25,
            ),
            0.25,
        )
        max_az = safe_float(
            self.drop_cfg.get(
                "post_drop_climb_accel_mps2",
                0.30,
            ),
            0.30,
        )

        desired_vz = max(-max_vz, min(max_vz, 0.9 * z_error))
        self.cmd_vz += max(
            -max_az * dt,
            min(max_az * dt, desired_vz - self.cmd_vz),
        )

        self.publish_position_xy_velocity_xyz_yaw(
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.cmd_xy[0],
            self.cmd_xy[1],
            self.cmd_vz,
            self.current_yaw,
        )

        # 3) 判断恢复完成：
        # 高度达到目标 + XY已经刹停 + 连续稳定0.4s
        xy_speed = norm2(self.cmd_xy[0], self.cmd_xy[1])
        height_ok = abs(z_error) <= safe_float(
            self.drop_cfg.get(
                "post_drop_height_tolerance_m",
                0.08,
            ),
            0.08,
        )
        speed_ok = xy_speed <= safe_float(
            self.drop_cfg.get(
                "post_drop_xy_stable_speed_mps",
                0.05,
            ),
            0.05,
        )

        ready = height_ok and speed_ok

        if ready:
            if self.post_drop_recover_since is None:
                self.post_drop_recover_since = rospy.Time.now()

            stable_time = (
                rospy.Time.now() -
                self.post_drop_recover_since
            ).to_sec()

            if stable_time >= safe_float(
                self.drop_cfg.get(
                    "post_drop_recover_stable_time_s",
                    0.40,
                ),
                0.40,
            ):
                self.enter_return_home(
                    "DROP_RECOVER_COMPLETE"
                )
        else:
            self.post_drop_recover_since = None


    def handle_follow_cd(self, dt):
        car = self.update_car_estimator(
            safe_float(self.follow_cfg.get("follow_prediction_s", 0.18), 0.18)
        )
        if car is None:
            return
        metrics = self.publish_dynamic_follow_control(
            car, self.home_z + self.cruise_height, dt
        )
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

        self.publish_dynamic_follow_control(
            car, self.landing_target_z, dt, speed_scale=0.75
        )

        touched = self.touchdown_condition(platform_z)
        confirmed = self.update_stable_timer(
            touched,
            "touchdown_since",
            safe_float(self.land_cfg.get("touchdown_confirm_time_s", 0.45), 0.45),
        )
        if confirmed:
            self.expected_disarm = True
            self.enter_state("PLATFORM_DISARM")

    def follow_metrics(self, car, target_offset_xy=None):
        if self.current_pose is None or car is None:
            return None
        tx, ty = self.car_target_xy(car)
        if target_offset_xy is not None:
            tx += safe_float(target_offset_xy[0], 0.0)
            ty += safe_float(target_offset_xy[1], 0.0)
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
            self.publish_dynamic_follow_control(
                car, platform_z, dt, speed_scale=0.55
            )
        else:
            self.publish_neutral()
        if self.current_state.armed:
            self.request_arm(False)
            return
        self.dwell_start = rospy.Time.now()
        self.expected_disarm = False
        self.enter_state("PLATFORM_DWELL")
        self.mission_result = "LANDED_ON_CAR"

    def handle_platform_dwell(self):
        car = self.update_car_estimator(0.0)
        platform_z = self.home_z + safe_float(
            self.land_cfg.get("platform_height_m", 0.34), 0.34
        )
        if car is not None:
            yaw = self.fixed_yaw
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
        self.publish_dynamic_follow_control(
            car, target_z, dt, speed_scale=0.85
        )

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
            safe_float(self.return_cfg.get("max_horizontal_speed_mps", 0.45), 0.45),
            safe_float(self.return_cfg.get("max_horizontal_accel_mps2", 0.30), 0.30),
            safe_float(self.return_cfg.get("position_kp", 0.55), 0.55),
            max_jerk=safe_float(
                self.return_cfg.get("max_horizontal_jerk_mps3", 0.70), 0.70
            ),
            max_vz=safe_float(
                self.return_cfg.get("max_vertical_speed_mps", 0.22), 0.22
            ),
            max_az=safe_float(
                self.return_cfg.get("max_vertical_accel_mps2", 0.30), 0.30
            ),
        )
        self.try_offboard_and_arm(allow_arm=True)
        if metrics is None:
            return
        xy_err, z_err, speed = metrics

        if self.mission_type == "drop":
            # 任务1：只要已经飞到H点上方的宽松到达区域，就进入定点判稳阶段。
            # HOME_LAND 中保持当前返航高度，不再执行OFFBOARD受控下降；
            # 连续稳定0.7s或等待3s超时后，直接切换PX4 AUTO.LAND。
            arrived = (
                xy_err <= safe_float(
                    self.return_cfg.get("task1_arrival_position_tolerance_m", 0.18),
                    0.18,
                ) and
                z_err <= safe_float(
                    self.return_cfg.get("task1_arrival_z_tolerance_m", 0.18),
                    0.18,
                )
            )
            if arrived:
                self.expected_disarm = True
                self.auto_land_prepare_start = None
                self.auto_land_start = None
                self.enter_state("HOME_LAND")
            return

        # 任务2保持原返航到位判定与低空AUTO.LAND交接流程。
        ready = self.update_stable_timer(
            xy_err <= safe_float(self.return_cfg.get("position_tolerance_m", 0.16), 0.16) and
            z_err <= safe_float(self.return_cfg.get("z_tolerance_m", 0.10), 0.10) and
            speed <= safe_float(self.return_cfg.get("speed_tolerance_mps", 0.18), 0.18),
            "home_stable_since",
            safe_float(self.return_cfg.get("stable_time_s", 0.45), 0.45),
        )
        if ready:
            self.expected_disarm = True
            self.auto_land_prepare_start = None
            self.auto_land_start = None
            self.enter_state("HOME_LAND")

    def handle_task1_home_land(self, now, dt):
        """任务1在H点返航高度判稳，满足条件或超时后直接切AUTO.LAND。"""
        if self.current_state.mode == "AUTO.LAND":
            if self.auto_land_start is None:
                self.auto_land_start = now
            return
        if self.current_pose is None:
            return

        target_z = self.home_z + safe_float(self.return_cfg.get("height_m", 1.20), 1.20)
        metrics = self.publish_point_control(
            self.home_x,
            self.home_y,
            target_z,
            max(float(dt), 0.01),
            safe_float(self.return_cfg.get("max_horizontal_speed_mps", 0.45), 0.45),
            safe_float(self.return_cfg.get("max_horizontal_accel_mps2", 0.30), 0.30),
            safe_float(self.return_cfg.get("position_kp", 0.55), 0.55),
            max_jerk=safe_float(
                self.return_cfg.get("max_horizontal_jerk_mps3", 0.70), 0.70
            ),
            max_vz=safe_float(
                self.return_cfg.get("max_vertical_speed_mps", 0.22), 0.22
            ),
            max_az=safe_float(
                self.return_cfg.get("max_vertical_accel_mps2", 0.30), 0.30
            ),
        )
        self.try_offboard_and_arm(allow_arm=True)
        if metrics is None:
            return

        xy_err, z_err, _ = metrics
        horizontal_speed = norm2(self.current_vel[0], self.current_vel[1])
        vertical_speed = abs(self.current_vel[2])
        stable_condition = (
            xy_err <= safe_float(
                self.return_cfg.get("task1_land_position_tolerance_m", 0.15),
                0.15,
            ) and
            z_err <= safe_float(
                self.return_cfg.get("task1_land_z_tolerance_m", 0.15),
                0.15,
            ) and
            horizontal_speed <= safe_float(
                self.return_cfg.get("task1_land_horizontal_speed_mps", 0.18),
                0.18,
            ) and
            vertical_speed <= safe_float(
                self.return_cfg.get("task1_land_vertical_speed_mps", 0.15),
                0.15,
            )
        )
        stable_ready = self.update_stable_timer(
            stable_condition,
            "home_land_stable_since",
            safe_float(self.return_cfg.get("task1_land_stable_time_s", 0.70), 0.70),
        )
        stabilize_timeout = safe_float(
            self.return_cfg.get("task1_land_stabilize_timeout_s", 3.0),
            3.0,
        )
        timeout_ready = self.state_elapsed() >= stabilize_timeout

        if stable_ready or timeout_ready:
            if self.auto_land_prepare_start is None:
                self.auto_land_prepare_start = now
                if stable_ready:
                    rospy.logwarn(
                        "Task1 home stable: xy=%.3fm z=%.3fm vxy=%.3fm/s vz=%.3fm/s; request AUTO.LAND.",
                        xy_err, z_err, horizontal_speed, vertical_speed,
                    )
                else:
                    rospy.logwarn(
                        "Task1 home stabilization timeout %.2fs: xy=%.3fm z=%.3fm vxy=%.3fm/s vz=%.3fm/s; force AUTO.LAND.",
                        stabilize_timeout, xy_err, z_err, horizontal_speed, vertical_speed,
                    )
            self.request_mode("AUTO.LAND")

    def handle_auto_land(self, emergency=False, dt=0.02):
        now = rospy.Time.now()

        if not emergency and self.mission_type == "drop":
            self.handle_task1_home_land(now, dt)
        elif emergency:
            if self.current_pose is not None and self.current_state.mode != "AUTO.LAND":
                p = self.current_pose.pose.position
                self.publish_position_velocity_yaw(
                    p.x, p.y, p.z, 0.0, 0.0, 0.0, self.current_yaw
                )
            if self.auto_land_prepare_start is None:
                self.auto_land_prepare_start = now
            if (now - self.auto_land_prepare_start).to_sec() >= 0.20:
                if self.current_state.mode != "AUTO.LAND":
                    self.request_mode("AUTO.LAND")
                elif self.auto_land_start is None:
                    self.auto_land_start = now
        elif self.current_state.mode != "AUTO.LAND":
            # 正常返航降落：先在OFFBOARD中持续锁定H点XY并缓慢下降。
            # XY偏差或水平速度过大时暂停降低Z目标，给无人机充分时间横向修正。
            if self.current_pose is None:
                return
            p = self.current_pose.pose.position
            handoff_z = self.home_z + safe_float(
                self.return_cfg.get("auto_land_handoff_height_m", 0.30), 0.30
            )
            if self.home_land_target_z is None:
                self.home_land_target_z = p.z

            xy_error = norm2(self.home_x - p.x, self.home_y - p.y)
            horizontal_speed = norm2(self.current_vel[0], self.current_vel[1])
            pause_descent = (
                xy_error > safe_float(
                    self.return_cfg.get("landing_pause_xy_error_m", 0.10), 0.10
                ) or
                horizontal_speed > safe_float(
                    self.return_cfg.get("landing_pause_horizontal_speed_mps", 0.12), 0.12
                )
            )
            remaining = max(0.0, self.home_land_target_z - handoff_z)
            slow_band = safe_float(
                self.return_cfg.get("landing_slow_height_m", 0.45), 0.45
            )
            descent_rate = safe_float(
                self.return_cfg.get(
                    "landing_final_rate_mps" if remaining <= slow_band
                    else "landing_descent_rate_mps",
                    0.08 if remaining <= slow_band else 0.15,
                ),
                0.08 if remaining <= slow_band else 0.15,
            )
            dt_eff = max(float(dt), 0.01)
            if pause_descent:
                # 立即把Z目标冻结在当前高度附近，避免旧的较低目标继续拉着无人机下沉。
                self.home_land_target_z = max(self.home_land_target_z, p.z)
            else:
                self.home_land_target_z = max(
                    handoff_z,
                    self.home_land_target_z - descent_rate * dt_eff,
                )

            metrics = self.publish_point_control(
                self.home_x,
                self.home_y,
                self.home_land_target_z,
                dt_eff,
                safe_float(
                    self.return_cfg.get("landing_max_horizontal_speed_mps", 0.25), 0.25
                ),
                safe_float(
                    self.return_cfg.get("landing_max_horizontal_accel_mps2", 0.25), 0.25
                ),
                safe_float(self.return_cfg.get("landing_position_kp", 0.65), 0.65),
                max_jerk=safe_float(
                    self.return_cfg.get("landing_max_horizontal_jerk_mps3", 0.60), 0.60
                ),
                max_vz=safe_float(
                    self.return_cfg.get("max_vertical_speed_mps", 0.22), 0.22
                ),
                max_az=safe_float(
                    self.return_cfg.get("max_vertical_accel_mps2", 0.30), 0.30
                ),
            )
            if metrics is None:
                return

            at_handoff = (
                xy_error <= safe_float(
                    self.return_cfg.get("auto_land_xy_tolerance_m", 0.06), 0.06
                ) and
                abs(p.z - handoff_z) <= safe_float(
                    self.return_cfg.get("auto_land_z_tolerance_m", 0.05), 0.05
                ) and
                horizontal_speed <= safe_float(
                    self.return_cfg.get("auto_land_horizontal_speed_mps", 0.08), 0.08
                ) and
                abs(self.current_vel[2]) <= safe_float(
                    self.return_cfg.get("auto_land_vertical_speed_mps", 0.08), 0.08
                )
            )
            ready = self.update_stable_timer(
                at_handoff,
                "home_land_stable_since",
                safe_float(
                    self.return_cfg.get("auto_land_stable_time_s", 0.80), 0.80
                ),
            )
            if ready:
                if self.auto_land_prepare_start is None:
                    self.auto_land_prepare_start = now
                prepare = safe_float(
                    self.return_cfg.get("auto_land_prepare_s", 0.30), 0.30
                )
                if (now - self.auto_land_prepare_start).to_sec() >= prepare:
                    self.request_mode("AUTO.LAND")
            else:
                self.auto_land_prepare_start = None
        elif self.auto_land_start is None:
            self.auto_land_start = now

        if self.is_landed():
            if self.current_state.armed:
                self.request_arm(False)
                return
            self.expected_disarm = False
            self.reset_required = True
            if emergency:
                self.mission_result = "ABORTED_LANDED"
            elif self.mission_result in {
                "RUNNING", "DROP_DONE", "LANDED_ON_CAR", "PLATFORM_TAKEOFF_DONE",
                "DROP_TASK_COMPLETE", "DYNAMIC_LAND_TASK_COMPLETE"
            }:
                self.mission_result = "MISSION_COMPLETE"
            self.publish_event_once("UAV_LANDED")
            if not emergency and self.mission_result == "MISSION_COMPLETE":
                self.publish_event_once("MISSION_DONE")
            self.enter_state("WAIT_RESET")
            return

        timeout = safe_float(self.return_cfg.get("auto_land_timeout_s", 15.0), 15.0)
        if self.auto_land_start is not None and (
            now - self.auto_land_start
        ).to_sec() > timeout:
            rospy.logerr_throttle(
                1.0, "AUTO.LAND timeout; keep requesting mode and waiting for landing."
            )
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
            "INTERCEPT", "FOLLOW_DROP", "DROP_DESCENT", "DROP_ALIGN", "DROP_WAIT_ACK",
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
            "DROP_DESCENT": "跟车下降到投放高度",
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
                "command_vx_mps": follow.get("command_vx"),
                "command_vy_mps": follow.get("command_vy"),
                "command_ax_mps2": follow.get("command_ax"),
                "command_ay_mps2": follow.get("command_ay"),
                "prediction_model": follow.get("prediction_model"),
                "prediction_s": follow.get("prediction_s"),
                "intercept_prediction_s": safe_float(
                    self.follow_cfg.get("intercept_prediction_s", 0.20), 0.20
                ),
                "intercept_prediction_raw_s": self.intercept_prediction_raw_s,
                "intercept_prediction_used_s": self.intercept_prediction_filtered_s,
                "intercept_current_distance_m": self.intercept_current_distance_m,
                "intercept_prediction_far_s": safe_float(
                    self.follow_cfg.get("intercept_prediction_far_s", 0.20), 0.20
                ),
                "intercept_prediction_near_s": safe_float(
                    self.follow_cfg.get("intercept_prediction_near_s", 0.05), 0.05
                ),
                "drop_entry_required_segment": follow.get(
                    "drop_entry_required_segment"
                ),
                "drop_entry_car_segment": follow.get("drop_entry_car_segment"),
                "drop_entry_segment_progress": follow.get(
                    "drop_entry_segment_progress"
                ),
                "drop_entry_segment_ok": follow.get("drop_entry_segment_ok"),
                "drop_entry_window_ok": follow.get("drop_entry_window_ok"),
                "drop_entry_waiting_for_cd": follow.get(
                    "drop_entry_waiting_for_cd"
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
                "home_land_target_z": self.home_land_target_z,
                "home_xy_error_m": (
                    norm2(p.x - self.home_x, p.y - self.home_y)
                    if p is not None and self.home_ready else None
                ),
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
                "descent_target_z": self.drop_target_z,
                "final_target_z": self.drop_final_target_z()
                    if self.home_ready else None,
                "target_height_above_platform_m": safe_float(
                    self.drop_cfg.get("target_height_above_platform_m", 0.35), 0.35
                ),
                "release_offset_frame": str(
                    self.drop_cfg.get("release_offset_frame", "body")
                ),
                "release_offset_x_m": safe_float(
                    self.drop_cfg.get("release_offset_x_m", 0.0), 0.0
                ),
                "release_offset_y_m": safe_float(
                    self.drop_cfg.get("release_offset_y_m", 0.0), 0.0
                ),
                "empirical_release_lead_distance_m": clamp(
                    safe_float(
                        self.drop_cfg.get(
                            "empirical_release_lead_distance_m",
                            0.0,
                        ),
                        0.0,
                    ),
                    -0.30,
                    0.30,
                ),
                "empirical_release_lead": deepcopy(self.last_drop_lead),
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
                    0.0, 0.0, 0.0, self.fixed_yaw,
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
                self.handle_drop_hover(dt)

            elif self.fsm_state == "INTERCEPT":
                self.handle_intercept(dt)

            elif self.fsm_state == "FOLLOW_DROP":
                self.handle_follow_drop(dt)

            elif self.fsm_state == "DROP_DESCENT":
                self.handle_drop_descent(dt)

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
                self.handle_platform_dwell()

            elif self.fsm_state == "PLATFORM_TAKEOFF":
                self.handle_platform_takeoff(dt)

            elif self.fsm_state == "RETURN_HOME":
                self.handle_return_home(dt)

            elif self.fsm_state == "HOME_LAND":
                self.handle_auto_land(emergency=False, dt=dt)

            elif self.fsm_state == "EMERGENCY_LAND":
                self.handle_auto_land(emergency=True, dt=dt)

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
