#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
D26 任务一 BOOT 启动器（环境与设备就绪修正版）

修正点：
1. 显式加载 /home/nvidia/env_ws/devel/setup.bash
2. 再加载 /home/nvidia/catkin_ws/devel/setup.bash
3. 收到 BOOT 后等待 Livox 192.168.1.177 可达
4. 等待飞控串口 /dev/ttyTHS0 存在
5. 再启动 d26_task1_real.launch
"""

import os
import re
import signal
import socket
import subprocess
import sys
import time
from typing import Optional, Tuple


LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 8888

GROUND_STATION_IP = "192.168.77.104"
ALLOWED_SOURCE_IPS = {GROUND_STATION_IP, "127.0.0.1"}

ROS_SETUP = "/opt/ros/noetic/setup.bash"
ENV_WS_SETUP = "/home/nvidia/env_ws/devel/setup.bash"
CATKIN_SETUP = "/home/nvidia/catkin_ws/devel/setup.bash"

LAUNCH_PACKAGE = "d26_air_ground_uav"
LAUNCH_FILE = "d26_task1_real.launch"
STARTUP_TASK = "T1"

LIDAR_IP = "192.168.1.177"
LIDAR_WAIT_TIMEOUT_S = 30
LIDAR_SETTLE_S = 3

FCU_DEVICE = "/dev/ttyTHS0"
FCU_WAIT_TIMEOUT_S = 20

SOCKET_TIMEOUT_S = 1.0
MAX_PACKET_BYTES = 2048
CMD_ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")

_child: Optional[subprocess.Popen] = None
_stop_requested = False


def log(message: str) -> None:
    print(message, flush=True)


def send_text(sock: socket.socket, addr: Tuple[str, int], text: str) -> None:
    try:
        sock.sendto(text.encode("utf-8"), addr)
        log(f"[TX] {addr[0]}:{addr[1]} <- {text}")
    except OSError as exc:
        log(f"[WARN] UDP reply failed: {exc}")


def parse_command(text: str):
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


def command_succeeds(command) -> bool:
    return subprocess.call(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ) == 0


def wait_for_lidar() -> bool:
    log(
        f"[PREFLIGHT] waiting for Livox {LIDAR_IP}, "
        f"timeout={LIDAR_WAIT_TIMEOUT_S}s"
    )

    deadline = time.monotonic() + LIDAR_WAIT_TIMEOUT_S
    while not _stop_requested and time.monotonic() < deadline:
        if command_succeeds(
            ["ping", "-c", "1", "-W", "1", LIDAR_IP]
        ):
            log(f"[PREFLIGHT] Livox {LIDAR_IP} is reachable")
            log(
                f"[PREFLIGHT] waiting another {LIDAR_SETTLE_S}s "
                "for Ethernet/Livox SDK stabilization"
            )
            time.sleep(LIDAR_SETTLE_S)
            return True
        time.sleep(1.0)

    log(f"[ERROR] Livox {LIDAR_IP} is not reachable")
    return False


def wait_for_fcu_device() -> bool:
    log(
        f"[PREFLIGHT] waiting for FCU serial {FCU_DEVICE}, "
        f"timeout={FCU_WAIT_TIMEOUT_S}s"
    )

    deadline = time.monotonic() + FCU_WAIT_TIMEOUT_S
    while not _stop_requested and time.monotonic() < deadline:
        if os.path.exists(FCU_DEVICE):
            log(f"[PREFLIGHT] FCU device exists: {FCU_DEVICE}")
            return True
        time.sleep(0.5)

    log(f"[ERROR] FCU device does not exist: {FCU_DEVICE}")
    return False


def validate_runtime_files() -> None:
    required = [ROS_SETUP, ENV_WS_SETUP, CATKIN_SETUP]
    missing = [path for path in required if not os.path.isfile(path)]
    if missing:
        raise RuntimeError(
            "Missing setup file(s): " + ", ".join(missing)
        )


def build_launch_command() -> str:
    # 顺序很重要：ROS -> Livox/Fast-LIO工作空间 -> 当前任务工作空间
    return (
        "set -e; "
        f"source {ROS_SETUP}; "
        f"source {ENV_WS_SETUP}; "
        f"source {CATKIN_SETUP}; "
        "export ROS_MASTER_URI=http://127.0.0.1:11311; "
        f"exec roslaunch {LAUNCH_PACKAGE} {LAUNCH_FILE} "
        f"startup_boot_task:={STARTUP_TASK} "
        f"ground_station_ip:={GROUND_STATION_IP} "
        "strict_source_ip:=true "
        "learn_ground_station_ip:=false"
    )


def terminate_child() -> None:
    global _child

    child = _child
    if child is None or child.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(child.pid), signal.SIGINT)
        child.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(child.pid), signal.SIGTERM)
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
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

    if command_succeeds(
        ["pgrep", "-f", "roslaunch.*d26_task1_real.launch"]
    ):
        raise RuntimeError(
            "another d26_task1_real.launch is already running"
        )

    if not wait_for_lidar():
        raise RuntimeError("Livox is not ready")

    if not wait_for_fcu_device():
        raise RuntimeError("FCU serial device is not ready")

    command = build_launch_command()
    log("[LAUNCH] starting task1 stack")
    log(f"[LAUNCH] {command}")

    _child = subprocess.Popen(
        ["/bin/bash", "-c", command],
        cwd="/home/nvidia/catkin_ws",
        env={
            **os.environ,
            "HOME": "/home/nvidia",
            "ROS_HOME": "/home/nvidia/.ros",
            "PYTHONUNBUFFERED": "1",
        },
        start_new_session=True,
    )

    exit_code = _child.wait()
    log(f"[LAUNCH] roslaunch exited with code {exit_code}")
    _child = None
    return int(exit_code)


def wait_for_boot() -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(SOCKET_TIMEOUT_S)

    log("============================================================")
    log("D26 task1 BOOT launcher waiting")
    log(f"[LISTEN] {LISTEN_IP}:{LISTEN_PORT}")
    log(f"[GS] allowed source: {GROUND_STATION_IP}")
    log("[EXPECT] CMD:<cmd_id>:BOOT:T1")
    log("============================================================")

    try:
        while not _stop_requested:
            try:
                data, addr = sock.recvfrom(MAX_PACKET_BYTES)
            except socket.timeout:
                continue

            source_ip = str(addr[0])
            text = data.decode("utf-8", errors="replace").strip()
            log(f"[RX] {source_ip}:{addr[1]} -> {text}")

            if source_ip not in ALLOWED_SOURCE_IPS:
                log(f"[WARN] ignored unauthorized IP {source_ip}")
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
                send_text(sock, addr, "ACK:BOOT:STARTING")
            else:
                send_text(
                    sock,
                    addr,
                    f"ACK:{cmd_id}:BOOT:ACCEPTED:{STARTUP_TASK}",
                )

            time.sleep(0.15)
            log("[BOOT] accepted; releasing UDP 8888")
            return True

        return False
    finally:
        sock.close()


def main() -> int:
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    try:
        if not wait_for_boot() or _stop_requested:
            return 0
        return run_roslaunch()
    except Exception as exc:
        log(f"[FATAL] {type(exc).__name__}: {exc}")
        return 1
    finally:
        terminate_child()


if __name__ == "__main__":
    sys.exit(main())
