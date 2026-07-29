#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""SITL/台架用小车状态模拟器，不用于正式比赛。"""

import json
import math
import os

import rospy
import yaml
from std_msgs.msg import Bool, String


class StadiumTrackModel:
    def __init__(self, cfg):
        self.a_x = float(cfg.get("a_x_m", 1.25))
        self.a_y = float(cfg.get("a_y_m", 0.75))
        self.yaw_ab = math.radians(float(cfg.get("ab_yaw_deg", 90.0)))
        self.length = float(cfg.get("straight_length_m", 1.50))
        self.radius = float(cfg.get("radius_m", 0.75))
        self.total = 2.0 * self.length + 2.0 * math.pi * self.radius

    def local_to_global(self, u, v):
        fx, fy = math.cos(self.yaw_ab), math.sin(self.yaw_ab)
        rx, ry = math.sin(self.yaw_ab), -math.cos(self.yaw_ab)
        return self.a_x + fx * u + rx * v, self.a_y + fy * u + ry * v

    def heading(self, du, dv):
        fx, fy = math.cos(self.yaw_ab), math.sin(self.yaw_ab)
        rx, ry = math.sin(self.yaw_ab), -math.cos(self.yaw_ab)
        return math.atan2(fy * du + ry * dv, fx * du + rx * dv)

    def map(self, s, speed):
        s %= self.total
        l1 = self.length
        l2 = l1 + math.pi * self.radius
        l3 = l2 + self.length
        if s < l1:
            seg, prog = "AB", s / self.length
            u, v, du, dv = s, 0.0, 1.0, 0.0
        elif s < l2:
            theta = (s - l1) / self.radius
            seg, prog = "BC", theta / math.pi
            u = self.length + self.radius * math.sin(theta)
            v = self.radius - self.radius * math.cos(theta)
            du, dv = math.cos(theta), math.sin(theta)
        elif s < l3:
            q = s - l2
            seg, prog = "CD", q / self.length
            u, v, du, dv = self.length - q, 2.0 * self.radius, -1.0, 0.0
        else:
            theta = (s - l3) / self.radius
            seg, prog = "DA", theta / math.pi
            u = -self.radius * math.sin(theta)
            v = self.radius + self.radius * math.cos(theta)
            du, dv = -math.cos(theta), -math.sin(theta)
        x, y = self.local_to_global(u, v)
        yaw = self.heading(du, dv)
        return x, y, speed * math.cos(yaw), speed * math.sin(yaw), yaw, seg, prog


class CarStateSimulator:
    def __init__(self):
        rospy.init_node("car_state_simulator", anonymous=False)
        default_yaml = os.path.expanduser(
            "~/catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"
        )
        path = rospy.get_param("~mission_yaml", default_yaml)
        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        self.track = StadiumTrackModel(cfg.get("car_udp", {}).get("track", {}))
        self.speed = float(rospy.get_param("~speed", 0.12))
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.task = str(rospy.get_param("~mission_type", "dynamic_land"))
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.start_delay = float(rospy.get_param("~start_delay", 2.0))
        self.mission_id = int(rospy.get_param("~mission_id", 1))
        self.s = float(rospy.get_param("~start_path_s", 0.0))
        self.start_time = rospy.Time.now()
        self.started = False
        self.car_pub = rospy.Publisher("/car/state", String, queue_size=30)
        self.type_pub = rospy.Publisher("/uav/mission_type", String, queue_size=5, latch=True)
        self.start_pub = rospy.Publisher("/uav/start", Bool, queue_size=5)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(0.0, min(0.1, (now - last).to_sec()))
            last = now
            if self.auto_start and not self.started and (now - self.start_time).to_sec() >= self.start_delay:
                self.type_pub.publish(String(data=self.task))
                rospy.sleep(0.05)
                self.start_pub.publish(Bool(data=True))
                self.started = True
            if self.started or not self.auto_start:
                self.s = (self.s + self.speed * dt) % self.track.total
            x, y, vx, vy, yaw, seg, prog = self.track.map(self.s, self.speed)
            payload = {
                "type": "car_state",
                "mission_id": self.mission_id,
                "mission_type": self.task,
                "stamp": now.to_sec(),
                "x": x,
                "y": y,
                "vx": vx,
                "vy": vy,
                "yaw": yaw,
                "segment": seg,
                "segment_progress": prog,
                "path_s": self.s,
                "path_s_cm": self.s * 100.0,
                "segment_reported": seg,
                "segment_from_path": seg,
                "segment_consistent": True,
                "running": True,
            }
            self.car_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
            rate.sleep()


if __name__ == "__main__":
    try:
        CarStateSimulator().spin()
    except rospy.ROSInterruptException:
        pass
