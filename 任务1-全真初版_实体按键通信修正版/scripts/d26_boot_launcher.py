#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
D26 任务一常驻 BOOT 启动器。

运行阶段：
1. 无人机上电后由 systemd 自动运行本脚本。
2. 本脚本监听 0.0.0.0:8888。
3. 收到地面站：
       CMD:<cmd_id>:BOOT:T1
   后回复：
       ACK:<cmd_id>:BOOT:ACCEPTED:T1
4. 关闭监听 socket，释放 UDP 8888。
5. 启动：
       roslaunch d26_air_ground_uav d26_task1_real.launch
           startup_boot_task:=T1
6. launch 中的 uav_udp_gateway.py 接管 UDP 8888。
7. roslaunch 退出后，本进程退出；systemd 会重新启动 BOOT 监听器。

BOOT 只负责启动任务节点，不会触发无人机起飞。
真正 START 仍由小车实体按键倒计时结束后发送。
"""

import os
import re
import signal
import socket
import subprocess
import sys
import time
from typing import Optional, Tuple


# ==================== 固定配置 ====================

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 8888

# 三设备已固定 IP。允许 localhost 仅用于无人机本机调试。
GROUND_STATION_IP = "192.168.77.104"
ALLOWED_SOURCE_IPS = {GROUND_STATION_IP, "127.0.0.1"}

ROS_SETUP = "/opt/ros/noetic/setup.bash"
CATKIN_SETUP = "/home/nvidia/catkin_ws/devel/setup.bash"

LAUNCH_PACKAGE = "d26_air_ground_uav"
LAUNCH_FILE = "d26_task1_real.launch"
STARTUP_TASK = "T1"

SOCKET_TIMEOUT_S = 1.0
MAX_PACKET_BYTES = 2048
CMD_ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")

_child: Optional[subprocess.Popen] = None
_stop_requested = False


def log(message: str) -> None:
    """立即输出，保证 journalctl 中可实时看到日志。"""
    print(message, flush=True)


def build_launch_command() -> str:
    """
    gateway 与 BOOT 启动器先后使用同一个 UDP 8888：
    本脚本先 close socket，再执行 roslaunch。

    startup_boot_task:=T1 使新启动的 gateway 直接进入 BOOT STARTING，
    无需地面站再次发送 BOOT；准备完成后 gateway 会上报 UAV_BOOT_READY。
    """
    return (
        f"source {ROS_SETUP} && "
        f"source {CATKIN_SETUP} && "
        f"exec roslaunch {LAUNCH_PACKAGE} {LAUNCH_FILE} "
        f"startup_boot_task:={STARTUP_TASK} "
        f"ground_station_ip:={GROUND_STATION_IP} "
        f"strict_source_ip:=true "
        f"learn_ground_station_ip:=false"
    )


def send_text(sock: socket.socket, addr: Tuple[str, int], text: str) -> None:
    try:
        sock.sendto(text.encode("utf-8"), addr)
        log(f"[TX] {addr[0]}:{addr[1]} <- {text}")
    except OSError as exc:
        log(f"[WARN] UDP reply failed to {addr}: {exc}")


def parse_command(text: str):
    """
    正式格式：
        CMD:<cmd_id>:<action>[:arg...]

    兼容旧格式：
        CMD:BOOT
        CMD:PING
        CMD:STATUS
    """
    parts = [part.strip() for part in text.strip().split(":")]

    if len(parts) >= 3 and parts[0].upper() == "CMD":
        cmd_id = parts[1]
        action = parts[2].upper()
        args = [part.upper() for part in parts[3:]]

        if not CMD_ID_RE.fullmatch(cmd_id) or not action:
            return None
        return cmd_id, action, args, False

    if len(parts) == 2 and parts[0].upper() == "CMD":
        action = parts[1].upper()
        legacy_id = f"L{int(time.monotonic() * 1000) % 1000000:06d}"
        return legacy_id, action, [], True

    return None


def validate_runtime_files() -> None:
    required = [ROS_SETUP, CATKIN_SETUP]
    missing = [path for path in required if not os.path.isfile(path)]
    if missing:
        raise RuntimeError("Missing setup file(s): " + ", ".join(missing))


def terminate_child() -> None:
    global _child

    child = _child
    if child is None or child.poll() is not None:
        return

    log(f"[STOP] stopping roslaunch process group, pid={child.pid}")
    try:
        os.killpg(os.getpgid(child.pid), signal.SIGINT)
        child.wait(timeout=15)
    except subprocess.TimeoutExpired:
        log("[WARN] roslaunch did not exit after SIGINT; sending SIGTERM")
        try:
            os.killpg(os.getpgid(child.pid), signal.SIGTERM)
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            log("[WARN] roslaunch did not exit after SIGTERM; sending SIGKILL")
            os.killpg(os.getpgid(child.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def handle_signal(signum, _frame) -> None:
    global _stop_requested
    _stop_requested = True
    log(f"[SIGNAL] received {signum}")
    terminate_child()


def run_roslaunch() -> int:
    global _child

    validate_runtime_files()
    command = build_launch_command()

    log("[LAUNCH] starting task1 stack:")
    log(f"[LAUNCH] {command}")

    # 独立进程组便于 systemd stop 时把 roslaunch 及全部子节点一起关闭。
    _child = subprocess.Popen(
        ["bash", "-lc", command],
        start_new_session=True,
    )

    exit_code = _child.wait()
    log(f"[LAUNCH] roslaunch exited with code {exit_code}")
    _child = None
    return int(exit_code)


def wait_for_boot() -> bool:
    """
    返回 True 表示收到有效 BOOT:T1，应启动 launch。
    返回 False 表示收到退出信号。
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(SOCKET_TIMEOUT_S)

    log("============================================================")
    log("D26 task1 BOOT launcher is waiting")
    log(f"[LISTEN] {LISTEN_IP}:{LISTEN_PORT}")
    log(f"[GS] allowed source IP: {GROUND_STATION_IP}")
    log("[EXPECT] CMD:<cmd_id>:BOOT:T1")
    log("============================================================")

    try:
        while not _stop_requested:
            try:
                data, addr = sock.recvfrom(MAX_PACKET_BYTES)
            except socket.timeout:
                continue
            except OSError as exc:
                if _stop_requested:
                    return False
                log(f"[WARN] UDP receive error: {exc}")
                time.sleep(1.0)
                continue

            source_ip = str(addr[0])
            text = data.decode("utf-8", errors="replace").strip()
            log(f"[RX] {source_ip}:{addr[1]} -> {text}")

            if source_ip not in ALLOWED_SOURCE_IPS:
                log(f"[WARN] ignored packet from unauthorized IP {source_ip}")
                continue

            parsed = parse_command(text)
            if parsed is None:
                send_text(sock, addr, "ERR:0000:UNKNOWN:BAD_FORMAT")
                continue

            cmd_id, action, args, legacy = parsed

            if action == "PING":
                send_text(sock, addr, f"ACK:{cmd_id}:PING:OK:UAV_BOOT")
                continue

            if action == "STATUS":
                send_text(
                    sock,
                    addr,
                    f"ACK:{cmd_id}:STATUS:OK:BOOT_WAITING",
                )
                continue

            if action != "BOOT":
                send_text(
                    sock,
                    addr,
                    f"ERR:{cmd_id}:{action}:UNKNOWN_CMD:BOOT_LISTENER",
                )
                continue

            task = args[0] if args else STARTUP_TASK
            if task != STARTUP_TASK:
                send_text(
                    sock,
                    addr,
                    f"ERR:{cmd_id}:BOOT:MODE_MISMATCH:T1_ONLY",
                )
                continue

            if legacy:
                # 兼容旧程序的可读回复。
                send_text(sock, addr, "ACK:BOOT:STARTING")
            else:
                send_text(
                    sock,
                    addr,
                    f"ACK:{cmd_id}:BOOT:ACCEPTED:{STARTUP_TASK}",
                )

            # 给 ACK 一点时间进入网卡发送队列，再关闭 8888。
            time.sleep(0.15)
            log("[BOOT] valid T1 BOOT accepted; releasing UDP 8888")
            return True

        return False
    finally:
        sock.close()


def main() -> int:
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    try:
        should_launch = wait_for_boot()
        if not should_launch or _stop_requested:
            return 0
        return run_roslaunch()
    except Exception as exc:
        log(f"[FATAL] {type(exc).__name__}: {exc}")
        return 1
    finally:
        terminate_child()


if __name__ == "__main__":
    sys.exit(main())
