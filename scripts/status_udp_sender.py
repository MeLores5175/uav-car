#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""把 /uav/mission_status JSON 按固定频率发送给地面站。"""

import os
import socket
import time

import rospy
import yaml
from std_msgs.msg import String


class StatusUdpSender:
    def __init__(self):
        rospy.init_node("status_udp_sender", anonymous=False)
        default_yaml = os.path.expanduser(
            "~/catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"
        )
        config_path = rospy.get_param("~mission_yaml", default_yaml)
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        scfg = cfg.get("status_udp", {})
        self.enabled = bool(rospy.get_param("~enabled", scfg.get("enabled", True)))
        self.target_ip = str(rospy.get_param("~target_ip", scfg.get("target_ip", "127.0.0.1")))
        self.target_port = int(rospy.get_param("~target_port", scfg.get("target_port", 8891)))
        self.max_rate_hz = float(scfg.get("max_rate_hz", 10.0))
        self.last_send = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.Subscriber("/uav/mission_status", String, self.status_cb, queue_size=20)
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn(
            "Status UDP sender enabled=%s target=%s:%d",
            str(self.enabled), self.target_ip, self.target_port
        )

    def status_cb(self, msg):
        if not self.enabled:
            return
        now = time.time()
        if now - self.last_send < 1.0 / max(self.max_rate_hz, 1.0):
            return
        self.last_send = now
        try:
            self.sock.sendto(msg.data.encode("utf-8"), (self.target_ip, self.target_port))
        except OSError as exc:
            rospy.logwarn_throttle(1.0, "Status UDP send failed: %s", str(exc))

    def shutdown(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        StatusUdpSender().spin()
    except rospy.ROSInterruptException:
        pass
