#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""单路投放机构串口桥。

唯一占用 ESP32 串口的 ROS 节点：
- FSM 自动投放：/uav/drop_cmd JSON，cmd=A90；
- 地面站起飞前手动控制：同一话题，cmd=A30/A90；
- 执行后统一发布 /uav/drop_ack，供 FSM 或 UDP 网关确认。

ESP32-C3 固件约定：
- 上电自动 A30（锁止）；
- A30 = 锁止；
- A90 = 释放；
- 串口 115200，每条命令以换行结束，成功回复包含 OK。
"""

import glob
import json
import os
import threading
import time

import rospy
import yaml
from std_msgs.msg import String

try:
    import serial
except Exception:
    serial = None


class DropActuatorSerial:
    def __init__(self):
        rospy.init_node("drop_actuator_serial", anonymous=False)
        default_yaml = os.path.expanduser(
            "~/catkin_ws/src/d26_air_ground_uav/config/air_ground_mission.yaml"
        )
        config_path = rospy.get_param("~mission_yaml", default_yaml)
        with open(config_path, "r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream) or {}
        acfg = cfg.get("actuator", {})

        self.enabled = bool(rospy.get_param("~enabled", acfg.get("enabled", True)))
        self.simulate = bool(rospy.get_param("~simulate", acfg.get("simulate", False)))
        self.port = str(rospy.get_param("~port", acfg.get("port", "/dev/ttyUSB0")))
        self.baud = int(rospy.get_param("~baud", acfg.get("baud", 115200)))
        self.timeout_s = float(acfg.get("timeout_s", 0.55))
        self.boot_wait_s = float(acfg.get("boot_wait_s", 2.0))
        self.retry_count = max(1, int(acfg.get("retry_count", 3)))
        self.retry_delay_s = max(0.0, float(acfg.get("retry_delay_s", 0.20)))
        self.ack_contains = str(acfg.get("ack_contains", "OK"))
        self.line_ending = str(acfg.get("line_ending", "\n"))
        self.auto_detect = bool(acfg.get("auto_detect", True))
        self.lock_command = str(acfg.get("lock_command", "A30")).strip().upper()
        self.release_command = str(acfg.get("release_command", "A90")).strip().upper()
        self.allowed_commands = {self.lock_command, self.release_command}

        self.ser = None
        self.lock = threading.Lock()
        self.successful_actions = set()
        self.last_open_try = 0.0

        self.ack_pub = rospy.Publisher("/uav/drop_ack", String, queue_size=20)
        rospy.Subscriber("/uav/drop_cmd", String, self.command_cb, queue_size=20)
        rospy.on_shutdown(self.shutdown)

        if self.enabled and not self.simulate:
            self.open_serial(force=True)
        rospy.logwarn(
            "Single-servo actuator ready. enabled=%s simulate=%s port=%s baud=%d "
            "lock=%s release=%s",
            str(self.enabled),
            str(self.simulate),
            self.port,
            self.baud,
            self.lock_command,
            self.release_command,
        )

    def serial_candidates(self):
        candidates = [self.port]
        if self.auto_detect:
            candidates.extend(sorted(glob.glob("/dev/serial/by-id/*")))
            candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))
            candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
        result = []
        for item in candidates:
            if item and item not in result:
                result.append(item)
        return result

    def open_serial(self, force=False):
        if serial is None:
            rospy.logerr_throttle(2.0, "pyserial not installed: sudo apt install python3-serial")
            return False
        now = time.time()
        if not force and now - self.last_open_try < 2.0:
            return False
        self.last_open_try = now
        self.shutdown_serial_only()
        errors = []
        for path in self.serial_candidates():
            try:
                self.ser = serial.Serial(
                    path,
                    self.baud,
                    timeout=self.timeout_s,
                    write_timeout=self.timeout_s,
                )
                self.port = path
                # USB 串口打开可能让 ESP32 复位；等待其完成上电 A30 和 READY 输出。
                time.sleep(self.boot_wait_s)
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    pass
                rospy.logwarn("Actuator serial opened: %s @ %d", self.port, self.baud)
                return True
            except Exception as exc:
                errors.append("%s:%s" % (path, str(exc)))
        rospy.logerr_throttle(2.0, "Cannot open actuator serial: %s", " | ".join(errors))
        return False

    def shutdown_serial_only(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def shutdown(self):
        self.shutdown_serial_only()

    def publish_ack(self, mission_id, action_id, success, reply, attempts=0, source="fsm"):
        payload = {
            "mission_id": int(mission_id),
            "action_id": int(action_id),
            "success": bool(success),
            "reply": str(reply),
            "attempts": int(attempts),
            "source": str(source),
            "stamp": rospy.Time.now().to_sec(),
        }
        self.ack_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def normalize_command(self, command):
        text = str(command).strip().upper()
        aliases = {
            "LOCK": self.lock_command,
            "CLOSE": self.lock_command,
            "RELEASE": self.release_command,
            "OPEN": self.release_command,
            # 兼容上一个三路舵机项目的单路释放命令。
            "R1": self.release_command,
            "L1": self.lock_command,
        }
        text = aliases.get(text, text)
        return text if text in self.allowed_commands else ""

    def execute_serial(self, command):
        if self.simulate:
            time.sleep(0.08)
            return True, "SIMULATED_OK %s" % command, 1
        if not self.enabled:
            return False, "ACTUATOR_DISABLED", 0
        if self.ser is None or not self.ser.is_open:
            if not self.open_serial():
                return False, "SERIAL_NOT_OPEN", 0

        wire = (str(command) + self.line_ending).encode("utf-8")
        last_reply = "NO_REPLY"
        for attempt in range(1, self.retry_count + 1):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(wire)
                self.ser.flush()
                deadline = time.time() + self.timeout_s
                lines = []
                while time.time() < deadline:
                    raw = self.ser.readline()
                    if not raw:
                        continue
                    text = raw.decode("utf-8", errors="ignore").strip()
                    if text:
                        lines.append(text)
                        if self.ack_contains.upper() in text.upper():
                            return True, text, attempt
                last_reply = " | ".join(lines) if lines else "TIMEOUT"
            except Exception as exc:
                last_reply = "SERIAL_ERROR:%s" % str(exc)
                self.shutdown_serial_only()
                self.open_serial(force=True)
            if attempt < self.retry_count:
                time.sleep(self.retry_delay_s)
        return False, last_reply, self.retry_count

    def command_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {
                "cmd": msg.data.strip(),
                "mission_id": 0,
                "action_id": 0,
                "source": "legacy",
            }

        command_raw = str(data.get("cmd", "")).strip()
        command = self.normalize_command(command_raw)
        mission_id = int(data.get("mission_id", 0))
        action_id = int(data.get("action_id", 0))
        source = str(data.get("source", "fsm"))

        if not command:
            self.publish_ack(
                mission_id,
                action_id,
                False,
                "INVALID_COMMAND:%s;ONLY_%s_OR_%s" % (
                    command_raw,
                    self.lock_command,
                    self.release_command,
                ),
                0,
                source,
            )
            return

        key = (mission_id, action_id)
        with self.lock:
            if key in self.successful_actions:
                self.publish_ack(
                    mission_id,
                    action_id,
                    True,
                    "DUPLICATE_ALREADY_DONE",
                    0,
                    source,
                )
                return

            ok, reply, attempts = self.execute_serial(command)
            if ok:
                self.successful_actions.add(key)
            self.publish_ack(mission_id, action_id, ok, reply, attempts, source)

            if ok:
                rospy.logwarn(
                    "Actuator success source=%s mission=%d action=%d cmd=%s reply=%s",
                    source,
                    mission_id,
                    action_id,
                    command,
                    reply,
                )
            else:
                rospy.logerr(
                    "Actuator failed source=%s mission=%d action=%d cmd=%s reply=%s",
                    source,
                    mission_id,
                    action_id,
                    command,
                    reply,
                )

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        DropActuatorSerial().spin()
    except rospy.ROSInterruptException:
        pass
