#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""陆空协同无人机 UDP V1.1 网关。

统一绑定 UAV:8888，处理：
- GS -> UAV: CMD:PING/STATUS/BOOT/LAND/RESET
- CAR -> UAV: CMD:START、TEL:CAR、EVT:CAR_POINT
- UAV -> GS: ACK/ERR/HB/TEL:UAV/EVT

外部协议统一使用场地坐标（左下角为原点，X 向右、Y 向上，cm）。
ROS 内部 /car/state 使用相对 H 点的 MAVROS local 坐标（m）。
"""

from collections import OrderedDict
import json
import math
import os
import socket
import threading
import time

import rospy
import yaml
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String


VALID_SEGMENTS = {"AB", "BC", "CD", "DA", "UNKNOWN"}


def clamp(value, low, high):
    return max(low, min(high, value))


def safe_float(value, default=0.0):
    try:
        result = float(value)
        return result if math.isfinite(result) else float(default)
    except Exception:
        return float(default)


def safe_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return int(default)


def normalize_task(value):
    text = str(value).strip().upper()
    if text in {"2", "T2", "TASK2", "DYNAMIC_LAND"}:
        return "T2"
    if text in {"1", "T1", "TASK1", "DROP"}:
        return "T1"
    return ""


def task_to_mission_type(task):
    return "dynamic_land" if normalize_task(task) == "T2" else "drop"


def run_id_to_int(run_id):
    digits = "".join(ch for ch in str(run_id) if ch.isdigit())
    if digits:
        return safe_int(digits, 0)
    return 0


class TrackProgressModel:
    """只根据累计里程计算区段、区段进度和整圈进度。"""

    def __init__(self, cfg):
        self.straight = safe_float(cfg.get("straight_length_m", 1.50), 1.50)
        self.radius = safe_float(cfg.get("radius_m", 0.75), 0.75)
        self.total = 2.0 * self.straight + 2.0 * math.pi * self.radius
        self.boundary_tolerance = safe_float(
            cfg.get("segment_boundary_tolerance_m", 0.08), 0.08
        )

    def calculate(self, path_s_m):
        if self.total <= 1e-6:
            return "UNKNOWN", -1.0, -1.0
        s = clamp(float(path_s_m), 0.0, self.total)
        b = self.straight
        c = b + math.pi * self.radius
        d = c + self.straight
        if s < b:
            return "AB", s / self.straight, s / self.total
        if s < c:
            return "BC", (s - b) / (math.pi * self.radius), s / self.total
        if s < d:
            return "CD", (s - c) / self.straight, s / self.total
        return "DA", (s - d) / (math.pi * self.radius), s / self.total

    def near_boundary(self, path_s_m):
        boundaries = [
            self.straight,
            self.straight + math.pi * self.radius,
            2.0 * self.straight + math.pi * self.radius,
            self.total,
        ]
        return any(abs(float(path_s_m) - value) <= self.boundary_tolerance for value in boundaries)


class FieldTransform:
    """FIELD 坐标与以 H 点为原点的 MAVROS local 相对坐标互换。"""

    def __init__(self, cfg):
        self.h_x_m = safe_float(cfg.get("h_field_x_cm", 75.0), 75.0) / 100.0
        self.h_y_m = safe_float(cfg.get("h_field_y_cm", 75.0), 75.0) / 100.0
        self.theta = math.radians(
            safe_float(cfg.get("local_x_to_field_yaw_deg", 0.0), 0.0)
        )

    def field_to_local_xy(self, x_field_m, y_field_m):
        dx = float(x_field_m) - self.h_x_m
        dy = float(y_field_m) - self.h_y_m
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return c * dx + s * dy, -s * dx + c * dy

    def local_to_field_xy(self, x_local_m, y_local_m):
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return (
            self.h_x_m + c * float(x_local_m) - s * float(y_local_m),
            self.h_y_m + s * float(x_local_m) + c * float(y_local_m),
        )

    def field_to_local_vector(self, vx_field, vy_field):
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return c * vx_field + s * vy_field, -s * vx_field + c * vy_field

    def local_to_field_vector(self, vx_local, vy_local):
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return c * vx_local - s * vy_local, s * vx_local + c * vy_local

    def field_to_local_yaw(self, yaw_field_rad):
        return math.atan2(
            math.sin(float(yaw_field_rad) - self.theta),
            math.cos(float(yaw_field_rad) - self.theta),
        )

    def local_to_field_yaw_deg(self, yaw_local_rad):
        value = math.degrees(float(yaw_local_rad) + self.theta) % 360.0
        return value if value >= 0.0 else value + 360.0


class UavUdpGateway:
    def __init__(self):
        rospy.init_node("uav_udp_gateway", anonymous=False)
        default_yaml = os.path.expanduser(
            "~/catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"
        )
        config_path = rospy.get_param("~mission_yaml", default_yaml)
        with open(config_path, "r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream) or {}

        pcfg = cfg.get("udp_protocol", {})
        ucfg = pcfg.get("uav", {})
        gcfg = pcfg.get("ground_station", {})
        ccfg = pcfg.get("car", {})
        rcfg = pcfg.get("reliability", {})
        tcfg = pcfg.get("telemetry", {})

        self.proto = str(pcfg.get("version", "1.1"))
        self.listen_ip = str(ucfg.get("listen_ip", "0.0.0.0"))
        self.listen_port = int(ucfg.get("listen_port", 8888))
        self.gs_addr = (str(gcfg.get("ip", "192.168.151.101")), int(gcfg.get("port", 8889)))
        self.car_ip = str(ccfg.get("ip", "192.168.151.103"))
        self.strict_source_ip = bool(pcfg.get("strict_source_ip", True))
        self.allow_gs_direct_start = bool(pcfg.get("allow_gs_direct_start", False))
        self.allow_legacy = bool(pcfg.get("allow_legacy_format", True))
        self.command_result_timeout = safe_float(
            rcfg.get("command_result_timeout_s", 0.22), 0.22
        )
        self.dedup_limit = max(8, safe_int(rcfg.get("dedup_cache_size", 32), 32))
        self.event_repeat = max(1, safe_int(rcfg.get("event_repeat", 3), 3))
        self.event_repeat_interval = safe_float(
            rcfg.get("event_repeat_interval_s", 0.05), 0.05
        )
        self.hb_rate_hz = max(0.2, safe_float(tcfg.get("heartbeat_rate_hz", 1.0), 1.0))
        self.tel_rate_hz = max(1.0, safe_float(tcfg.get("uav_rate_hz", 10.0), 10.0))
        self.require_segment_consistency = bool(
            ccfg.get("require_segment_consistency", True)
        )
        self.max_delay_compensation_s = safe_float(
            ccfg.get("max_delay_compensation_s", 0.30), 0.30
        )

        self.transform = FieldTransform(pcfg.get("field_transform", {}))
        self.track = TrackProgressModel(pcfg.get("track", {}))

        self.car_pub = rospy.Publisher("/car/state", String, queue_size=50)
        self.car_event_pub = rospy.Publisher("/car/event", String, queue_size=20)
        self.command_pub = rospy.Publisher("/uav/mission_command", String, queue_size=10)
        self.mission_type_pub = rospy.Publisher("/uav/mission_type", String, queue_size=10)
        self.land_pub = rospy.Publisher("/uav/land", Bool, queue_size=10)
        self.reset_pub = rospy.Publisher("/uav/reset", Bool, queue_size=10)

        rospy.Subscriber("/uav/mission_status", String, self.status_cb, queue_size=30)
        rospy.Subscriber("/uav/mission_event", String, self.event_cb, queue_size=30)
        rospy.Subscriber(
            "/uav/mission_command_result", String, self.command_result_cb, queue_size=20
        )
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_cb, queue_size=10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.settimeout(0.05)
        rospy.on_shutdown(self.shutdown)

        self.latest_status = {}
        self.latest_status_rx = 0.0
        self.battery_percent = -1
        self.boot_task = ""
        self.boot_state = "STOPPED"
        self.boot_ready_sent = False
        self.run_id = "R000"
        self.run_start_monotonic = None
        self.hb_seq = 0
        self.tel_seq = 0
        self.last_hb = 0.0
        self.last_tel = 0.0
        self.dedup = OrderedDict()
        self.result_condition = threading.Condition()
        self.command_results = {}

        rospy.logwarn(
            "UDP V%s gateway listening %s:%d, GS=%s:%d, CAR=%s",
            self.proto,
            self.listen_ip,
            self.listen_port,
            self.gs_addr[0],
            self.gs_addr[1],
            self.car_ip,
        )

    def shutdown(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def status_cb(self, msg):
        try:
            self.latest_status = json.loads(msg.data)
            self.latest_status_rx = time.monotonic()
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Invalid mission status JSON: %s", str(exc))

    def battery_cb(self, msg):
        value = safe_float(msg.percentage, -1.0)
        if 0.0 <= value <= 1.0:
            value *= 100.0
        if 0.0 <= value <= 100.0:
            self.battery_percent = int(round(value))

    def command_result_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        cmd_id = str(data.get("cmd_id", ""))
        if not cmd_id:
            return
        with self.result_condition:
            self.command_results[cmd_id] = data
            self.result_condition.notify_all()

    def event_cb(self, msg):
        try:
            data = json.loads(msg.data)
            event = str(data.get("event", "")).strip().upper()
            run_id = str(data.get("run_id", self.run_id))
            task = normalize_task(data.get("task", ""))
        except Exception:
            event = str(msg.data).strip().upper()
            run_id = self.run_id
            task = ""
        if not event:
            return
        if event == "MISSION_START":
            packet = "EVT:MISSION_START:{}:{}".format(run_id, task or self.boot_task or "T1")
        elif event == "UAV_BOOT_READY":
            packet = "EVT:UAV_BOOT_READY:{}".format(task or self.boot_task or "T1")
        else:
            packet = "EVT:{}:{}".format(event, run_id)
        self.send_event(packet)

    def send_event(self, packet):
        encoded = packet.encode("utf-8")
        for index in range(self.event_repeat):
            try:
                self.sock.sendto(encoded, self.gs_addr)
            except OSError as exc:
                rospy.logwarn_throttle(1.0, "UDP event send failed: %s", str(exc))
                break
            if index + 1 < self.event_repeat:
                time.sleep(self.event_repeat_interval)

    def source_role(self, addr):
        ip = addr[0]
        if ip == self.gs_addr[0]:
            return "GS"
        if ip == self.car_ip:
            return "CAR"
        if not self.strict_source_ip and ip in {"127.0.0.1", "localhost"}:
            return "DEBUG"
        return "UNKNOWN"

    def send_text(self, text, addr):
        try:
            self.sock.sendto(text.encode("utf-8"), addr)
        except OSError as exc:
            rospy.logwarn_throttle(1.0, "UDP send failed: %s", str(exc))

    def cache_reply(self, key, reply):
        self.dedup[key] = reply
        self.dedup.move_to_end(key)
        while len(self.dedup) > self.dedup_limit:
            self.dedup.popitem(last=False)

    def ack(self, cmd_id, action, result, detail=""):
        parts = ["ACK", str(cmd_id), str(action), str(result)]
        if detail:
            parts.append(str(detail))
        return ":".join(parts)

    def err(self, cmd_id, action, code, detail=""):
        parts = ["ERR", str(cmd_id), str(action), str(code)]
        if detail:
            parts.append(str(detail))
        return ":".join(parts)

    def parse_command(self, text):
        parts = text.strip().split(":")
        if len(parts) >= 3 and parts[0].upper() == "CMD":
            return parts[1], parts[2].upper(), parts[3:], False
        if self.allow_legacy and len(parts) == 2 and parts[0].upper() == "CMD":
            legacy_id = "L{:06d}".format(int(time.monotonic() * 1000) % 1000000)
            return legacy_id, parts[1].upper(), [], True
        return None

    def wait_command_result(self, cmd_id):
        deadline = time.monotonic() + self.command_result_timeout
        with self.result_condition:
            while cmd_id not in self.command_results:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.result_condition.wait(remaining)
            return self.command_results.pop(cmd_id, None)

    def boot_ready(self):
        status = self.latest_status
        if not status or time.monotonic() - self.latest_status_rx > 1.0:
            return False
        mavros = status.get("mavros", {})
        home = status.get("home", {})
        state = str(status.get("fsm_state", ""))
        return (
            bool(mavros.get("connected", False))
            and home.get("x") is not None
            and home.get("y") is not None
            and state == "WAIT_START"
        )

    def check_boot_transition(self):
        if self.boot_state != "STARTING" or not self.boot_task:
            return
        if self.boot_ready():
            self.boot_state = "READY"
            if not self.boot_ready_sent:
                self.boot_ready_sent = True
                self.send_event("EVT:UAV_BOOT_READY:{}".format(self.boot_task))

    def handle_command(self, text, addr):
        parsed = self.parse_command(text)
        if parsed is None:
            return
        cmd_id, action, args, legacy = parsed
        role = self.source_role(addr)
        key = (addr[0], cmd_id, action)
        if key in self.dedup:
            self.send_text(self.dedup[key], addr)
            return

        if self.strict_source_ip and role == "UNKNOWN":
            reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
        elif action == "PING":
            reply = self.ack(cmd_id, action, "OK", "UAV")
        elif action == "STATUS":
            reply = self.ack(cmd_id, action, "OK", "UAV")
            self.send_uav_telemetry(target=addr, force=True)
        elif action == "BOOT":
            if role not in {"GS", "DEBUG"}:
                reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
            else:
                task = normalize_task(args[0] if args else "")
                if not task:
                    reply = self.err(cmd_id, action, "BAD_TASK")
                elif self.latest_status.get("mavros", {}).get("armed", False):
                    reply = self.err(cmd_id, action, "BUSY", "ARMED")
                else:
                    self.boot_task = task
                    self.boot_state = "STARTING"
                    self.boot_ready_sent = False
                    self.mission_type_pub.publish(String(data=task_to_mission_type(task)))
                    reply = self.ack(cmd_id, action, "ACCEPTED", task)
        elif action == "START":
            allowed = role in {"CAR", "DEBUG"} or (
                role == "GS" and self.allow_gs_direct_start
            )
            if not allowed:
                reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
            else:
                run_id = str(args[0]) if len(args) >= 1 else "R000"
                task = normalize_task(args[1] if len(args) >= 2 else self.boot_task)
                if not task:
                    reply = self.err(cmd_id, action, "BAD_TASK")
                elif self.boot_state != "READY":
                    reply = self.err(cmd_id, action, "NOT_READY", "BOOT")
                elif self.boot_task and task != self.boot_task:
                    reply = self.err(cmd_id, action, "MODE_MISMATCH")
                else:
                    command = {
                        "action": "START",
                        "cmd_id": cmd_id,
                        "run_id": run_id,
                        "task": task,
                        "source": role,
                        "legacy": legacy,
                    }
                    self.command_pub.publish(
                        String(data=json.dumps(command, separators=(",", ":")))
                    )
                    result = self.wait_command_result(cmd_id)
                    if result is None:
                        reply = self.err(cmd_id, action, "NOT_READY", "FSM_NO_REPLY")
                    elif bool(result.get("ok", False)):
                        self.run_id = run_id
                        self.run_start_monotonic = time.monotonic()
                        reply = self.ack(cmd_id, action, "OK", run_id)
                    else:
                        reply = self.err(
                            cmd_id,
                            action,
                            str(result.get("error", "NOT_READY")),
                            str(result.get("detail", "")),
                        )
        elif action == "LAND":
            if role not in {"GS", "DEBUG"}:
                reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
            else:
                run_id = str(args[0]) if args else self.run_id
                reply = self.ack(cmd_id, action, "ACCEPTED", run_id)
                self.land_pub.publish(Bool(data=True))
                self.send_event("EVT:UAV_ABORTING:{}".format(run_id))
        elif action == "RESET":
            if role not in {"GS", "DEBUG"}:
                reply = self.err(cmd_id, action, "UNKNOWN_CMD", "SOURCE_NOT_ALLOWED")
            elif bool(self.latest_status.get("mavros", {}).get("armed", False)):
                reply = self.err(cmd_id, action, "BUSY", "ARMED")
            else:
                self.reset_pub.publish(Bool(data=True))
                self.run_id = "R000"
                self.run_start_monotonic = None
                reply = self.ack(cmd_id, action, "ACCEPTED")
        elif action == "STOP_NODES":
            reply = self.err(cmd_id, action, "UNKNOWN_CMD", "DISABLED_IN_FLIGHT_BUILD")
        else:
            reply = self.err(cmd_id, action, "UNKNOWN_CMD")

        self.cache_reply(key, reply)
        self.send_text(reply, addr)

    def parse_car_telemetry(self, text, addr):
        role = self.source_role(addr)
        if self.strict_source_ip and role not in {"CAR", "DEBUG"}:
            return
        prefix = "TEL:CAR:"
        try:
            data = json.loads(text[len(prefix):])
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Invalid TEL:CAR JSON: %s", str(exc))
            return

        required = [
            "seq", "time_ms", "run", "task", "state", "x_cm", "y_cm",
            "speed_cm_s", "yaw_deg", "segment", "path_s_cm",
            "line_detected", "error",
        ]
        missing = [key for key in required if key not in data]
        if missing:
            rospy.logwarn_throttle(1.0, "TEL:CAR missing fields: %s", ",".join(missing))
            return

        task = normalize_task(data.get("task"))
        if not task:
            rospy.logwarn_throttle(1.0, "TEL:CAR bad task: %s", str(data.get("task")))
            return
        segment_reported = str(data.get("segment", "UNKNOWN")).upper()
        if segment_reported not in VALID_SEGMENTS:
            segment_reported = "UNKNOWN"

        x_field = safe_float(data.get("x_cm")) / 100.0
        y_field = safe_float(data.get("y_cm")) / 100.0
        speed = max(0.0, safe_float(data.get("speed_cm_s")) / 100.0)
        yaw_field = math.radians(safe_float(data.get("yaw_deg")))
        vx_field = speed * math.cos(yaw_field)
        vy_field = speed * math.sin(yaw_field)
        x_local, y_local = self.transform.field_to_local_xy(x_field, y_field)
        vx_local, vy_local = self.transform.field_to_local_vector(vx_field, vy_field)
        yaw_local = self.transform.field_to_local_yaw(yaw_field)

        path_s_m = max(0.0, safe_float(data.get("path_s_cm")) / 100.0)
        segment_from_path, segment_progress, lap_progress = self.track.calculate(path_s_m)
        consistent = (
            segment_reported == segment_from_path
            or segment_reported == "UNKNOWN"
            or self.track.near_boundary(path_s_m)
        )
        control_segment = segment_reported
        if self.require_segment_consistency and not consistent:
            control_segment = "UNKNOWN"
            rospy.logwarn_throttle(
                0.8,
                "CAR segment/path mismatch: reported=%s calculated=%s path=%.2fm",
                segment_reported,
                segment_from_path,
                path_s_m,
            )

        packet_delay = 0.0
        if self.run_start_monotonic is not None and str(data.get("run")) == self.run_id:
            local_elapsed = time.monotonic() - self.run_start_monotonic
            remote_elapsed = max(0.0, safe_float(data.get("time_ms")) / 1000.0)
            packet_delay = clamp(
                local_elapsed - remote_elapsed, 0.0, self.max_delay_compensation_s
            )
            x_local += vx_local * packet_delay
            y_local += vy_local * packet_delay

        state = str(data.get("state", "RUNNING")).upper()
        output = {
            "type": "car_state",
            "stamp": rospy.Time.now().to_sec(),
            "seq": safe_int(data.get("seq"), 0),
            "time_ms": safe_int(data.get("time_ms"), 0),
            "run_id": str(data.get("run", "R000")),
            "mission_id": run_id_to_int(data.get("run", "R000")),
            "mission_type": task_to_mission_type(task),
            "x": x_local,
            "y": y_local,
            "vx": vx_local,
            "vy": vy_local,
            "yaw": yaw_local,
            "speed": speed,
            "segment": control_segment,
            "segment_reported": segment_reported,
            "segment_from_path": segment_from_path,
            "segment_consistent": consistent,
            "segment_progress": segment_progress,
            "path_s": path_s_m,
            "path_s_cm": safe_float(data.get("path_s_cm")),
            "lap_progress": lap_progress,
            "point": str(data.get("point", "")),
            "running": state not in {"IDLE", "READY", "FINISHED", "FAULT", "LINE_LOST"},
            "state": state,
            "line_detected": bool(data.get("line_detected", False)),
            "battery": safe_int(data.get("battery", -1), -1),
            "error": safe_int(data.get("error", 0), 0),
            "packet_delay_s": packet_delay,
        }
        self.car_pub.publish(
            String(data=json.dumps(output, ensure_ascii=False, separators=(",", ":")))
        )

    def parse_car_event(self, text, addr):
        role = self.source_role(addr)
        if self.strict_source_ip and role not in {"CAR", "DEBUG"}:
            return
        self.car_event_pub.publish(String(data=text))
        # 地面站即使没有直接收到小车事件，也能从无人机转发链看到一次。
        self.send_text(text, self.gs_addr)

    @staticmethod
    def map_public_state(status):
        detail = str(status.get("fsm_state", "WAIT_START"))
        mission_type = str(status.get("mission_type", "drop"))
        if detail == "WAIT_START":
            return "WAIT_START"
        if detail in {"WAIT_FCU", "TAKEOFF"}:
            return "TAKEOFF"
        if detail == "DROP_HOVER":
            return "HOVER"
        if detail == "INTERCEPT":
            return "SEARCH_CAR" if mission_type == "drop" else "APPROACH_CAR"
        if detail in {"FOLLOW_DROP", "FOLLOW_CD"}:
            return "FOLLOW" if mission_type == "drop" else "APPROACH_CAR"
        if detail == "DROP_ALIGN":
            return "PREPARE_DROP"
        if detail == "DROP_WAIT_ACK":
            return "DROPPING"
        if detail == "POST_DROP_FOLLOW":
            return "DROP_DONE"
        if detail in {"DYNAMIC_DESCENT", "PLATFORM_DISARM"}:
            return "LAND_ON_CAR"
        if detail == "PLATFORM_DWELL":
            return "ON_CAR"
        if detail == "PLATFORM_TAKEOFF":
            return "TAKEOFF_FROM_CAR"
        if detail == "RETURN_HOME":
            return "RETURN_HOME"
        if detail == "HOME_LAND":
            return "LAND_HOME"
        if detail == "EMERGENCY_LAND":
            return "LAND_HOME"
        if detail == "WAIT_RESET":
            return "DONE"
        return detail

    @staticmethod
    def map_safety(status):
        detail = str(status.get("fsm_state", ""))
        safety = str(status.get("safety_state", "NORMAL")).upper()
        if detail == "EMERGENCY_LAND":
            return "LANDING"
        if detail == "WAIT_RESET" and not bool(status.get("mavros", {}).get("armed", False)):
            return "LANDED"
        if safety in {"IDLE", "NORMAL"}:
            return "NORMAL"
        if "EMERGENCY" in safety or "ABORT" in safety:
            return "ABORTING"
        return "WARNING"

    def uav_payload(self):
        status = self.latest_status or {}
        uav = status.get("uav", {})
        x_local = safe_float(uav.get("x_rel_home"), 0.0)
        y_local = safe_float(uav.get("y_rel_home"), 0.0)
        x_field, y_field = self.transform.local_to_field_xy(x_local, y_local)
        vx_field, vy_field = self.transform.local_to_field_vector(
            safe_float(uav.get("vx"), 0.0), safe_float(uav.get("vy"), 0.0)
        )
        yaw_deg = self.transform.local_to_field_yaw_deg(safe_float(uav.get("yaw"), 0.0))
        mission_type = str(status.get("mission_type", "drop"))
        task = 2 if mission_type == "dynamic_land" else 1
        run_id = str(status.get("run_id", self.run_id))
        if run_id in {"", "None"}:
            run_id = self.run_id
        if self.run_start_monotonic is None:
            time_ms = 0
        else:
            time_ms = int(max(0.0, time.monotonic() - self.run_start_monotonic) * 1000.0)
        tracking = status.get("tracking", {})
        vision = status.get("vision", {})
        abort_reason = str(status.get("abort_reason", ""))
        return {
            "proto": self.proto,
            "seq": self.tel_seq,
            "time_ms": time_ms,
            "run": run_id,
            "task": task,
            "boot": self.boot_state,
            "state": self.map_public_state(status),
            "safety": self.map_safety(status),
            "armed": bool(status.get("mavros", {}).get("armed", False)),
            "fcu": bool(status.get("mavros", {}).get("connected", False)),
            "mode": str(status.get("mavros", {}).get("mode", "")),
            "x_cm": round(x_field * 100.0, 1),
            "y_cm": round(y_field * 100.0, 1),
            "z_cm": round(safe_float(uav.get("z_rel_home"), 0.0) * 100.0, 1),
            "vx_cm_s": round(vx_field * 100.0, 1),
            "vy_cm_s": round(vy_field * 100.0, 1),
            "vz_cm_s": round(safe_float(uav.get("vz"), 0.0) * 100.0, 1),
            "yaw_deg": round(yaw_deg, 1),
            "battery": self.battery_percent,
            "target_locked": bool(vision.get("valid", False) or tracking.get("valid", False)),
            "error": 0 if not abort_reason else 1,
        }

    def send_uav_telemetry(self, target=None, force=False):
        if not self.latest_status:
            return
        now = time.monotonic()
        if not force and now - self.last_tel < 1.0 / self.tel_rate_hz:
            return
        self.last_tel = now
        self.tel_seq += 1
        payload = self.uav_payload()
        payload["seq"] = self.tel_seq
        packet = "TEL:UAV:" + json.dumps(
            payload, ensure_ascii=False, separators=(",", ":")
        )
        self.send_text(packet, target or self.gs_addr)

    def send_heartbeat(self):
        now = time.monotonic()
        if now - self.last_hb < 1.0 / self.hb_rate_hz:
            return
        self.last_hb = now
        self.hb_seq += 1
        state = self.map_public_state(self.latest_status) if self.latest_status else self.boot_state
        self.send_text("HB:UAV:{}:{}".format(self.hb_seq, state), self.gs_addr)

    def process_packet(self, text, addr):
        if text.startswith("CMD:"):
            self.handle_command(text, addr)
        elif text.startswith("TEL:CAR:"):
            self.parse_car_telemetry(text, addr)
        elif text.startswith("EVT:CAR_POINT:"):
            self.parse_car_event(text, addr)
        elif self.allow_legacy and text.strip().upper() in {"PING", "STATUS", "LAND", "RESET"}:
            legacy = "CMD:L{:06d}:{}".format(
                int(time.monotonic() * 1000) % 1000000, text.strip().upper()
            )
            self.handle_command(legacy, addr)
        else:
            rospy.logwarn_throttle(1.0, "Unsupported UDP packet from %s: %s", addr[0], text[:100])

    def spin(self):
        while not rospy.is_shutdown():
            try:
                raw, addr = self.sock.recvfrom(65535)
                text = raw.decode("utf-8", errors="replace").strip()
                if text:
                    self.process_packet(text, addr)
            except socket.timeout:
                pass
            except OSError as exc:
                if not rospy.is_shutdown():
                    rospy.logwarn_throttle(1.0, "UDP receive failed: %s", str(exc))
            self.check_boot_transition()
            self.send_heartbeat()
            self.send_uav_telemetry()


if __name__ == "__main__":
    try:
        UavUdpGateway().spin()
    except rospy.ROSInterruptException:
        pass
