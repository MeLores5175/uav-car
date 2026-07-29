#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
小车 -> 无人机 UDP 桥接节点。

推荐小车/树莓派发送 JSON：
{
  "type":"car_state",
  "mission_id":1,
  "mission_type":"dynamic_land",
  "path_s":2.35,
  "speed":0.12,
  "running":true,
  "start":true
}

若发送 path_s/speed，本节点依据 YAML 中 A 点、A->B 航向、直线长度和半径，
换算为相对 H 点的 x/y/vx/vy/yaw、AB/BC/CD/DA 段名和段内进度。
也支持直接发送 x/y/vx/vy/yaw/segment/segment_progress。
"""

import json
import math
import os
import socket
import time

import rospy
import yaml
from std_msgs.msg import Bool, String


def clamp(value, low, high):
    return max(low, min(high, value))


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


class StadiumTrackModel:
    """A->B直线、B->C上半圆、C->D直线、D->A下半圆。"""

    def __init__(self, cfg):
        self.a_x = safe_float(cfg.get("a_x_m", 1.25), 1.25)
        self.a_y = safe_float(cfg.get("a_y_m", 0.75), 0.75)
        self.yaw_ab = math.radians(safe_float(cfg.get("ab_yaw_deg", 90.0), 90.0))
        self.length = safe_float(cfg.get("straight_length_m", 1.50), 1.50)
        self.radius = safe_float(cfg.get("radius_m", 0.75), 0.75)
        self.total_length = 2.0 * self.length + 2.0 * math.pi * self.radius

    def local_to_global(self, u, v):
        # u：沿 A->B；v：A->B 方向的右侧。
        forward_x = math.cos(self.yaw_ab)
        forward_y = math.sin(self.yaw_ab)
        right_x = math.sin(self.yaw_ab)
        right_y = -math.cos(self.yaw_ab)
        return (
            self.a_x + forward_x * u + right_x * v,
            self.a_y + forward_y * u + right_y * v,
        )

    def heading_to_global(self, du, dv):
        forward_x = math.cos(self.yaw_ab)
        forward_y = math.sin(self.yaw_ab)
        right_x = math.sin(self.yaw_ab)
        right_y = -math.cos(self.yaw_ab)
        dx = forward_x * du + right_x * dv
        dy = forward_y * du + right_y * dv
        return math.atan2(dy, dx)

    def map_s(self, path_s, speed):
        if self.total_length <= 1e-6:
            raise RuntimeError("Invalid track length")
        s = float(path_s) % self.total_length
        speed = float(speed)
        l1 = self.length
        l2 = l1 + math.pi * self.radius
        l3 = l2 + self.length

        if s < l1:
            segment = "AB"
            progress = s / self.length
            u = s
            v = 0.0
            du, dv = 1.0, 0.0

        elif s < l2:
            segment = "BC"
            arc_s = s - l1
            theta = arc_s / self.radius
            progress = theta / math.pi
            # B=(L,0)，C=(L,2R)，进入时沿 +u，离开时沿 -u。
            u = self.length + self.radius * math.sin(theta)
            v = self.radius - self.radius * math.cos(theta)
            du = math.cos(theta)
            dv = math.sin(theta)

        elif s < l3:
            segment = "CD"
            straight_s = s - l2
            progress = straight_s / self.length
            u = self.length - straight_s
            v = 2.0 * self.radius
            du, dv = -1.0, 0.0

        else:
            segment = "DA"
            arc_s = s - l3
            theta = arc_s / self.radius
            progress = theta / math.pi
            # D=(0,2R)，A=(0,0)，进入时沿 -u，离开时沿 +u。
            u = -self.radius * math.sin(theta)
            v = self.radius + self.radius * math.cos(theta)
            du = -math.cos(theta)
            dv = -math.sin(theta)

        x, y = self.local_to_global(u, v)
        yaw = self.heading_to_global(du, dv)
        vx = speed * math.cos(yaw)
        vy = speed * math.sin(yaw)
        return {
            "x": x,
            "y": y,
            "vx": vx,
            "vy": vy,
            "yaw": yaw,
            "segment": segment,
            "segment_progress": clamp(progress, 0.0, 1.0),
            "path_s": s,
            "lap_progress": s / self.total_length,
            "track_total_length": self.total_length,
        }


class CarUdpBridge:
    def __init__(self):
        rospy.init_node("car_udp_bridge", anonymous=False)
        default_yaml = os.path.expanduser(
            "~/catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"
        )
        config_path = rospy.get_param("~mission_yaml", default_yaml)
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        ccfg = cfg.get("car_udp", {})
        tcfg = ccfg.get("track", {})

        self.bind_ip = str(rospy.get_param("~bind_ip", ccfg.get("bind_ip", "0.0.0.0")))
        self.bind_port = int(rospy.get_param("~bind_port", ccfg.get("bind_port", 8890)))
        self.socket_timeout = safe_float(ccfg.get("socket_timeout_s", 0.05), 0.05)
        self.allow_text_commands = bool(ccfg.get("allow_text_commands", True))
        self.auto_start_from_car = bool(ccfg.get("auto_start_from_car", True))
        self.status_reply_enabled = bool(ccfg.get("status_reply_enabled", True))
        self.track_enabled = bool(tcfg.get("enabled", True))
        self.track_model = StadiumTrackModel(tcfg)

        self.car_pub = rospy.Publisher("/car/state", String, queue_size=50)
        self.start_pub = rospy.Publisher("/uav/start", Bool, queue_size=10)
        self.mission_type_pub = rospy.Publisher("/uav/mission_type", String, queue_size=10)
        self.last_uav_status = "{}"
        rospy.Subscriber("/uav/mission_status", String, self.status_cb, queue_size=20)

        self.last_client = None
        self.last_start_key = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.bind_port))
        self.sock.settimeout(self.socket_timeout)
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn(
            "Car UDP bridge listening on %s:%d, track_length=%.3fm",
            self.bind_ip, self.bind_port, self.track_model.total_length
        )

    def status_cb(self, msg):
        self.last_uav_status = msg.data

    def shutdown(self):
        try:
            self.sock.close()
        except Exception:
            pass

    @staticmethod
    def normalize_mission_type(value):
        text = str(value).strip().lower()
        if text in {"2", "t2", "task2", "dynamic_land", "动态起降"}:
            return "dynamic_land"
        return "drop"

    def parse_text(self, text):
        upper = text.strip().upper()
        if upper in {"START1", "TASK1", "DROP"}:
            return {"type": "mission_start", "mission_type": "drop"}
        if upper in {"START2", "TASK2", "DYNAMIC_LAND"}:
            return {"type": "mission_start", "mission_type": "dynamic_land"}
        if upper == "STATUS":
            return {"type": "status_request"}
        return None

    def map_car_state(self, data):
        output = dict(data)
        if all(key in data for key in ["x", "y"]):
            x = safe_float(data.get("x"))
            y = safe_float(data.get("y"))
            yaw = safe_float(data.get("yaw", 0.0))
            speed = safe_float(data.get("speed", 0.0))
            vx = safe_float(data.get("vx", speed * math.cos(yaw)))
            vy = safe_float(data.get("vy", speed * math.sin(yaw)))
            output.update({"x": x, "y": y, "vx": vx, "vy": vy, "yaw": yaw})
            output.setdefault("segment", "UNKNOWN")
            output.setdefault("segment_progress", -1.0)
        elif self.track_enabled and ("path_s" in data or "lap_progress" in data):
            if "path_s" in data:
                path_s = safe_float(data.get("path_s", 0.0))
            else:
                path_s = safe_float(data.get("lap_progress", 0.0)) * self.track_model.total_length
            speed = safe_float(data.get("speed", 0.0))
            output.update(self.track_model.map_s(path_s, speed))
        else:
            raise ValueError("car_state requires x/y or path_s/lap_progress")

        output["type"] = "car_state"
        output["stamp"] = safe_float(data.get("stamp", time.time()), time.time())
        output["mission_id"] = safe_int(data.get("mission_id", 0), 0)
        output["mission_type"] = self.normalize_mission_type(
            data.get("mission_type", "drop")
        )
        output["running"] = bool(data.get("running", True))
        output["seq"] = safe_int(data.get("seq", 0), 0)
        return output

    def maybe_start(self, data):
        if not self.auto_start_from_car:
            return
        start_flag = bool(data.get("start", False)) or str(data.get("type", "")).lower() == "mission_start"
        if not start_flag:
            return
        mission_type = self.normalize_mission_type(data.get("mission_type", "drop"))
        mission_id = safe_int(data.get("mission_id", 0), 0)
        start_key = (mission_id, mission_type)
        if start_key == self.last_start_key:
            return
        self.last_start_key = start_key
        self.mission_type_pub.publish(String(data=mission_type))
        rospy.sleep(0.05)
        self.start_pub.publish(Bool(data=True))
        rospy.logwarn("Car requested mission start: id=%d type=%s", mission_id, mission_type)

    def send_reply(self, addr, payload):
        try:
            if isinstance(payload, dict):
                payload = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            self.sock.sendto(str(payload).encode("utf-8"), addr)
        except OSError as exc:
            rospy.logwarn_throttle(1.0, "UDP reply failed: %s", str(exc))

    def handle_packet(self, raw, addr):
        self.last_client = addr
        text = raw.decode("utf-8", errors="ignore").strip()
        if not text:
            return
        try:
            data = json.loads(text)
        except Exception:
            data = self.parse_text(text) if self.allow_text_commands else None
        if data is None or not isinstance(data, dict):
            self.send_reply(addr, {"ok": False, "reason": "INVALID_PACKET"})
            return

        packet_type = str(data.get("type", "car_state")).strip().lower()
        if packet_type == "status_request":
            self.send_reply(addr, self.last_uav_status)
            return
        if packet_type == "mission_start":
            self.maybe_start(data)
            self.send_reply(addr, {"ok": True, "accepted": "mission_start"})
            return
        if packet_type != "car_state":
            self.send_reply(addr, {"ok": False, "reason": "UNSUPPORTED_TYPE"})
            return

        try:
            mapped = self.map_car_state(data)
        except Exception as exc:
            self.send_reply(addr, {"ok": False, "reason": str(exc)})
            return
        self.car_pub.publish(
            String(data=json.dumps(mapped, ensure_ascii=False, separators=(",", ":")))
        )
        self.maybe_start(mapped)
        if self.status_reply_enabled:
            self.send_reply(addr, {
                "ok": True,
                "seq": mapped.get("seq", 0),
                "segment": mapped.get("segment", "UNKNOWN"),
                "segment_progress": mapped.get("segment_progress", -1.0),
            })

    def spin(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            try:
                raw, addr = self.sock.recvfrom(65535)
                self.handle_packet(raw, addr)
            except socket.timeout:
                pass
            except OSError as exc:
                if not rospy.is_shutdown():
                    rospy.logerr_throttle(1.0, "Car UDP receive error: %s", str(exc))
            rate.sleep()


if __name__ == "__main__":
    try:
        CarUdpBridge().spin()
    except rospy.ROSInterruptException:
        pass
