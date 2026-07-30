#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地无人机 + 小车 UDP 模拟器。

用途：不连接真实硬件时测试地面站的 WebSocket、地图、状态、任务按钮和协议解析。
运行：
    python mock_devices.py
然后另开终端：
    python app.py --config config.mock.json
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
import os
import sys
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional, Tuple

GS_ADDR = ("127.0.0.1", 8889)
UAV_ADDR = ("127.0.0.1", 8888)
CAR_ADDR = ("127.0.0.1", 8890)
READY_FILE = Path(__file__).resolve().with_name("mock_ready.flag")
PID_FILE = Path(__file__).resolve().with_name("mock_instance.pid")


def compact(obj: dict) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))


def capsule_position(distance: float) -> Tuple[float, float, float, str]:
    """返回原场地坐标 x/y、航向角和当前关键区段。"""
    radius = 75.0
    straight = 150.0
    arc = math.pi * radius
    total = 2 * straight + 2 * arc
    d = distance % total

    # A(150,200) -> B(150,350)
    if d < straight:
        return 150.0, 200.0 + d, 90.0, "A-B"
    d -= straight

    # B -> C，上方半圆，中心 (225,350)
    if d < arc:
        theta = math.pi - d / radius
        x = 225.0 + radius * math.cos(theta)
        y = 350.0 + radius * math.sin(theta)
        yaw = 90.0 - math.degrees(d / radius)
        return x, y, yaw, "B-C"
    d -= arc

    # C(300,350) -> D(300,200)
    if d < straight:
        return 300.0, 350.0 - d, 270.0, "C-D"
    d -= straight

    # D -> A，下方半圆，中心 (225,200)
    theta = -d / radius
    x = 225.0 + radius * math.cos(theta)
    y = 200.0 + radius * math.sin(theta)
    yaw = 270.0 - math.degrees(d / radius)
    return x, y, yaw % 360.0, "D-A"


@dataclass
class SharedState:
    task: int = 1
    car_state: str = "IDLE"
    uav_state: str = "STOPPED"
    uav_boot: str = "STOPPED"
    uav_safety: str = "NORMAL"
    mission_active: bool = False
    aborting: bool = False
    abort_latched: bool = False
    start_epoch: Optional[float] = None
    abort_start_epoch: Optional[float] = None
    abort_x: float = 112.5
    abort_y: float = 75.0
    abort_z: float = 0.0
    abort_yaw: float = 90.0
    last_uav_x: float = 112.5
    last_uav_y: float = 75.0
    last_uav_z: float = 0.0
    last_uav_yaw: float = 90.0
    run_id: str = ""
    car_seq: int = 0
    uav_seq: int = 0
    car_point: str = "A"
    emitted_points: set[str] = field(default_factory=set)
    emitted_events: set[str] = field(default_factory=set)
    lock: threading.Lock = field(default_factory=threading.Lock)


STATE = SharedState()


class MockDevice:
    def __init__(self, name: str, bind_addr: Tuple[str, int]) -> None:
        self.name = name
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Windows UDP 特性：
        # 如果本 socket 向一个尚未监听的 UDP 端口发送数据，Windows 可能在后续
        # recvfrom() 中抛出 WinError 10054。旧版代码遇到 OSError 就退出接收线程，
        # 因而出现“遥测仍在刷新，但 BOOT/PING/START 永远没有 ACK”的现象。
        # SIO_UDP_CONNRESET=False 可关闭该行为；接收循环中还会再次兜底忽略 10054。
        if os.name == "nt" and hasattr(socket, "SIO_UDP_CONNRESET"):
            try:
                self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            except OSError:
                pass

        # Windows 下 SO_REUSEADDR 可能允许多个 Mock 同时占用同一 UDP 端口，
        # 导致命令被旧进程抢走、遥测却仍能收到。演示模式必须独占端口。
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        try:
            self.sock.bind(bind_addr)
        except OSError as exc:
            self.sock.close()
            raise OSError(
                f"{self.name} 无法绑定 {bind_addr[0]}:{bind_addr[1]}。"
                "该端口很可能被旧 Mock 占用，请关闭旧的 UAV + CAR MOCK 窗口后重试。"
            ) from exc
        self.sock.settimeout(0.2)
        self.running = True
        self.thread = threading.Thread(target=self.receive_loop, daemon=True)

    def start(self) -> None:
        self.thread.start()
        print(f"[{self.name}] listening on {self.sock.getsockname()}", flush=True)

    def send_gs(self, text: str) -> None:
        try:
            self.sock.sendto(text.encode("utf-8"), GS_ADDR)
        except ConnectionResetError:
            # Windows: GS 尚未监听时可能出现 UDP reset；遥测下一帧继续发送即可。
            return
        except OSError as exc:
            if getattr(exc, "winerror", None) == 10054:
                return
            if self.running:
                print(f"[{self.name}] telemetry send warning: {exc}", flush=True)

    def reply(self, addr: Tuple[str, int], text: str) -> None:
        self.sock.sendto(text.encode("utf-8"), addr)
        print(f"[{self.name}] TX -> {addr}: {text}", flush=True)

    def receive_loop(self) -> None:
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except ConnectionResetError:
                # Windows 在先前向未监听端口发送 UDP 后可能报告 WSAECONNRESET。
                # 这不代表本地命令监听 socket 已失效，必须继续接收。
                continue
            except OSError as exc:
                if not self.running:
                    break
                if getattr(exc, "winerror", None) == 10054:
                    continue
                print(f"[{self.name}] UDP receive warning: {exc}", flush=True)
                time.sleep(0.05)
                continue
            except Exception as exc:
                print(f"[{self.name}] receive loop warning: {exc}", flush=True)
                time.sleep(0.05)
                continue

            text = data.decode("utf-8", errors="ignore").strip()
            if not text:
                continue
            print(f"[{self.name}] RX <- {addr}: {text}", flush=True)
            try:
                self.handle_command(text, addr)
            except Exception as exc:
                # 单条异常命令不能杀死整个 Mock 接收线程。
                print(f"[{self.name}] command handler error: {exc}", flush=True)
                try:
                    self.reply(addr, "ERR:0000:INTERNAL:MOCK_EXCEPTION")
                except Exception:
                    pass

    def handle_command(self, text: str, addr: Tuple[str, int]) -> None:
        parts = text.split(":")
        if len(parts) < 3 or parts[0].upper() != "CMD":
            self.reply(addr, "ERR:0000:UNKNOWN:BAD_FORMAT")
            return
        cmd_id = parts[1]
        action = parts[2].upper()
        args = parts[3:]

        with STATE.lock:
            if action == "PING":
                self.reply(addr, f"ACK:{cmd_id}:PING:OK:{self.name}")
            elif action == "STATUS":
                self.reply(addr, f"ACK:{cmd_id}:STATUS:OK:{self.name}")
            elif action == "RESET":
                STATE.mission_active = False
                STATE.aborting = False
                STATE.abort_latched = False
                STATE.start_epoch = None
                STATE.abort_start_epoch = None
                STATE.abort_x, STATE.abort_y, STATE.abort_z, STATE.abort_yaw = 112.5, 75.0, 0.0, 90.0
                STATE.run_id = ""
                STATE.emitted_points.clear()
                STATE.emitted_events.clear()
                if self.name == "UAV":
                    STATE.uav_state = "WAIT_START" if STATE.uav_boot == "READY" else "STOPPED"
                    STATE.uav_safety = "NORMAL"
                else:
                    STATE.car_state = "READY" if STATE.task in (1, 2) else "IDLE"
                self.reply(addr, f"ACK:{cmd_id}:RESET:OK")
            elif self.name == "UAV":
                self.handle_uav(action, args, cmd_id, addr)
            else:
                self.handle_car(action, args, cmd_id, addr)

    def handle_uav(self, action: str, args: list[str], cmd_id: str, addr: Tuple[str, int]) -> None:
        if action == "BOOT":
            task_arg = args[0] if args else "T1"
            STATE.task = 2 if task_arg.upper() == "T2" else 1
            STATE.uav_boot = "STARTING"
            STATE.uav_state = "STARTING"
            self.reply(addr, f"ACK:{cmd_id}:BOOT:ACCEPTED:T{STATE.task}")

            def ready() -> None:
                with STATE.lock:
                    STATE.uav_boot = "READY"
                    STATE.uav_state = "WAIT_START"
                self.send_gs(f"EVT:UAV_BOOT_READY:T{STATE.task}")

            threading.Timer(1.0, ready).start()
        elif action == "LAND":
            STATE.aborting = True
            STATE.abort_latched = True
            STATE.abort_start_epoch = time.time()
            STATE.abort_x = STATE.last_uav_x
            STATE.abort_y = STATE.last_uav_y
            STATE.abort_z = max(0.0, STATE.last_uav_z)
            STATE.abort_yaw = STATE.last_uav_yaw
            STATE.uav_safety = "ABORTING"
            STATE.uav_state = "LAND_HOME"
            self.reply(addr, f"ACK:{cmd_id}:LAND:ACCEPTED:{args[0] if args else STATE.run_id}")
            self.send_gs(f"EVT:UAV_ABORTING:{STATE.run_id or 'R000'}")
            self.send_gs(f"EVT:UAV_LANDING:{STATE.run_id or 'R000'}")
        elif action == "STOP_NODES":
            STATE.uav_boot = "STOPPED"
            STATE.uav_state = "STOPPED"
            self.reply(addr, f"ACK:{cmd_id}:STOP_NODES:OK")
        elif action == "START":
            # 仅用于调试兼容。
            STATE.mission_active = True
            STATE.aborting = False
            STATE.abort_latched = False
            STATE.start_epoch = time.time()
            STATE.run_id = args[0] if args else "R999"
            self.reply(addr, f"ACK:{cmd_id}:START:OK:{STATE.run_id}")
        else:
            self.reply(addr, f"ERR:{cmd_id}:{action}:UNKNOWN_CMD")

    def handle_car(self, action: str, args: list[str], cmd_id: str, addr: Tuple[str, int]) -> None:
        if action == "MODE":
            task_arg = args[0] if args else "T1"
            STATE.task = 2 if task_arg.upper() == "T2" else 1
            STATE.car_state = "READY"
            self.reply(addr, f"ACK:{cmd_id}:MODE:OK:T{STATE.task}")
            self.send_gs(f"EVT:CAR_READY:T{STATE.task}")
        elif action == "START":
            if STATE.car_state not in ("READY", "FINISHED", "IDLE"):
                self.reply(addr, f"ERR:{cmd_id}:START:NOT_READY")
                return
            STATE.run_id = args[0] if args else "R001"
            STATE.mission_active = True
            STATE.aborting = False
            STATE.abort_latched = False
            STATE.start_epoch = time.time()
            STATE.abort_start_epoch = None
            STATE.car_state = "RUNNING"
            STATE.uav_state = "TAKEOFF"
            STATE.uav_safety = "NORMAL"
            STATE.emitted_points.clear()
            STATE.emitted_events.clear()
            self.reply(addr, f"ACK:{cmd_id}:START:OK:{STATE.run_id}")
            self.send_gs(f"EVT:MISSION_START:{STATE.run_id}:T{STATE.task}")
        else:
            self.reply(addr, f"ERR:{cmd_id}:{action}:UNKNOWN_CMD")

    def close(self) -> None:
        self.running = False
        self.sock.close()


def telemetry_loop(uav: MockDevice, car: MockDevice) -> None:
    last_hb = 0.0
    while True:
        now = time.time()
        with STATE.lock:
            active = STATE.mission_active
            elapsed = now - STATE.start_epoch if STATE.start_epoch else 0.0
            speed = 22.0
            distance = elapsed * speed if active else 0.0
            car_x, car_y, car_yaw, section = capsule_position(distance)

            if not active:
                car_x, car_y, car_yaw = 150.0, 200.0, 90.0
                STATE.car_point = "A"
            else:
                if section == "B-C" and "B" not in STATE.emitted_points:
                    STATE.emitted_points.add("B")
                    STATE.car_point = "B"
                    car.send_gs(f"EVT:CAR_POINT:{STATE.run_id}:B")
                if section == "C-D" and "C" not in STATE.emitted_points:
                    STATE.emitted_points.add("C")
                    STATE.car_point = "C"
                    car.send_gs(f"EVT:CAR_POINT:{STATE.run_id}:C")
                if section == "D-A" and "D" not in STATE.emitted_points:
                    STATE.emitted_points.add("D")
                    STATE.car_point = "D"
                    car.send_gs(f"EVT:CAR_POINT:{STATE.run_id}:D")

            if active and elapsed > 36.0:
                STATE.car_state = "FINISHED"
                STATE.mission_active = False
                if "DONE" not in STATE.emitted_events:
                    STATE.emitted_events.add("DONE")
                    if not STATE.abort_latched:
                        STATE.uav_state = "DONE"
                    car.send_gs(f"EVT:MISSION_DONE:{STATE.run_id}")

            # 无人机演示状态与位置。
            h_x, h_y = 112.5, 75.0
            uav_x, uav_y, uav_z, uav_yaw = h_x, h_y, 0.0, 90.0
            if active:
                if elapsed < 3.0:
                    STATE.uav_state = "TAKEOFF"
                    uav_z = min(150.0, elapsed / 3.0 * 150.0)
                elif elapsed < 7.0:
                    STATE.uav_state = "SEARCH_CAR"
                    alpha = (elapsed - 3.0) / 4.0
                    uav_x = h_x + (car_x - h_x) * alpha
                    uav_y = h_y + (car_y - h_y) * alpha
                    uav_z = 150.0
                elif STATE.task == 1:
                    if elapsed < 14.0:
                        STATE.uav_state = "FOLLOW"
                        uav_x, uav_y, uav_z, uav_yaw = car_x, car_y, 150.0, car_yaw
                        if "FOLLOW" not in STATE.emitted_events:
                            STATE.emitted_events.add("FOLLOW")
                            uav.send_gs(f"EVT:UAV_FOLLOW_ESTABLISHED:{STATE.run_id}")
                    elif elapsed < 17.0:
                        STATE.uav_state = "DROPPING"
                        drop_alpha = (elapsed - 14.0) / 3.0
                        uav_x, uav_y, uav_z, uav_yaw = car_x, car_y, 150.0 - 15.0 * drop_alpha, car_yaw
                    elif elapsed < 27.0:
                        if "DROP" not in STATE.emitted_events:
                            STATE.emitted_events.add("DROP")
                            uav.send_gs(f"EVT:UAV_DROP_DONE:{STATE.run_id}")
                        STATE.uav_state = "RETURN_HOME"
                        alpha = min(1.0, (elapsed - 17.0) / 10.0)
                        return_x, return_y, return_yaw, _ = capsule_position(17.0 * speed)
                        uav_x = return_x + (h_x - return_x) * alpha
                        uav_y = return_y + (h_y - return_y) * alpha
                        uav_z = 135.0 + 15.0 * min(1.0, alpha * 3.0)
                        uav_yaw = return_yaw
                    else:
                        STATE.uav_state = "LAND_HOME"
                        uav_x, uav_y = h_x, h_y
                        uav_z = max(0.0, 150.0 - (elapsed - 27.0) * 40.0)
                else:
                    if elapsed < 12.0:
                        STATE.uav_state = "APPROACH_CAR"
                        uav_x, uav_y, uav_z, uav_yaw = car_x, car_y, max(10.0, 150.0 - (elapsed - 7.0) * 28.0), car_yaw
                    elif elapsed < 17.0:
                        STATE.uav_state = "ON_CAR"
                        uav_x, uav_y, uav_z, uav_yaw = car_x, car_y, 0.0, car_yaw
                        if "LAND_CAR" not in STATE.emitted_events:
                            STATE.emitted_events.add("LAND_CAR")
                            uav.send_gs(f"EVT:UAV_LAND_ON_CAR:{STATE.run_id}")
                    elif elapsed < 20.0:
                        STATE.uav_state = "TAKEOFF_FROM_CAR"
                        uav_x, uav_y, uav_z, uav_yaw = car_x, car_y, min(150.0, (elapsed - 17.0) * 50.0), car_yaw
                        if "TAKEOFF_CAR" not in STATE.emitted_events:
                            STATE.emitted_events.add("TAKEOFF_CAR")
                            uav.send_gs(f"EVT:UAV_TAKEOFF_FROM_CAR:{STATE.run_id}")
                    elif elapsed < 30.0:
                        STATE.uav_state = "RETURN_HOME"
                        alpha = min(1.0, (elapsed - 20.0) / 10.0)
                        return_x, return_y, return_yaw, _ = capsule_position(20.0 * speed)
                        uav_x = return_x + (h_x - return_x) * alpha
                        uav_y = return_y + (h_y - return_y) * alpha
                        uav_z = 150.0
                        uav_yaw = return_yaw
                    else:
                        STATE.uav_state = "LAND_HOME"
                        uav_x, uav_y = h_x, h_y
                        uav_z = max(0.0, 150.0 - (elapsed - 30.0) * 40.0)

            # LAND 必须以收到命令瞬间的位置为锚点垂直下降，不能瞬移到小车或 H 点。
            if STATE.abort_latched:
                uav_x, uav_y, uav_yaw = STATE.abort_x, STATE.abort_y, STATE.abort_yaw
                if STATE.aborting:
                    STATE.uav_state = "LAND_HOME"
                    STATE.uav_safety = "LANDING"
                    abort_elapsed = now - STATE.abort_start_epoch if STATE.abort_start_epoch else 0.0
                    uav_z = max(0.0, STATE.abort_z - abort_elapsed * 55.0)
                    if uav_z <= 0.0:
                        STATE.aborting = False
                        STATE.uav_safety = "LANDED"
                        STATE.uav_state = "LANDED"
                        if "ABORT_LANDED" not in STATE.emitted_events:
                            STATE.emitted_events.add("ABORT_LANDED")
                            uav.send_gs(f"EVT:UAV_LANDED:{STATE.run_id or 'R000'}")
                else:
                    STATE.uav_state = "LANDED"
                    STATE.uav_safety = "LANDED"
                    uav_z = 0.0

            STATE.last_uav_x = uav_x
            STATE.last_uav_y = uav_y
            STATE.last_uav_z = uav_z
            STATE.last_uav_yaw = uav_yaw

            STATE.car_seq += 1
            STATE.uav_seq += 1
            car_payload = {
                "seq": STATE.car_seq,
                "time_ms": int(elapsed * 1000),
                "run": STATE.run_id,
                "task": STATE.task,
                "state": STATE.car_state,
                "x_cm": round(car_x, 2),
                "y_cm": round(car_y, 2),
                "speed_cm_s": speed if active else 0.0,
                "yaw_deg": round(car_yaw, 1),
                "point": STATE.car_point,
                "line_detected": True,
                "battery": 86,
                "error": 0,
            }
            uav_payload = {
                "seq": STATE.uav_seq,
                "time_ms": int(elapsed * 1000),
                "run": STATE.run_id,
                "task": STATE.task,
                "boot": STATE.uav_boot,
                "state": STATE.uav_state,
                "safety": STATE.uav_safety,
                "armed": uav_z > 2.0,
                "fcu": True,
                "mode": "OFFBOARD" if uav_z > 2.0 else "STABILIZED",
                "x_cm": round(uav_x, 2),
                "y_cm": round(uav_y, 2),
                "z_cm": round(uav_z, 2),
                "vx_cm_s": 0.0,
                "vy_cm_s": 0.0,
                "vz_cm_s": 0.0,
                "yaw_deg": round(uav_yaw, 1),
                "battery": 79,
                "target_locked": active and elapsed > 7.0,
                "error": 0,
            }

        car.send_gs("TEL:CAR:" + compact(car_payload))
        uav.send_gs("TEL:UAV:" + compact(uav_payload))
        if now - last_hb >= 1.0:
            last_hb = now
            car.send_gs(f"HB:CAR:{STATE.car_seq}:{STATE.car_state}")
            uav.send_gs(f"HB:UAV:{STATE.uav_seq}:{STATE.uav_state}")
        time.sleep(0.05)



def probe_mock(timeout: float = 2.0) -> int:
    """启动脚本使用的本地自检：确认 UAV/CAR 命令端口真的能收包并回 ACK。"""
    tests = [
        ("UAV", UAV_ADDR, "CMD:9901:PING", "ACK:9901:PING:OK:UAV"),
        ("CAR", CAR_ADDR, "CMD:9902:PING", "ACK:9902:PING:OK:CAR"),
    ]

    ok_all = True
    for name, addr, command, expected in tests:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if os.name == "nt" and hasattr(socket, "SIO_UDP_CONNRESET"):
                try:
                    sock.ioctl(socket.SIO_UDP_CONNRESET, False)
                except OSError:
                    pass
            sock.bind(("127.0.0.1", 0))
            sock.settimeout(timeout)
            sock.sendto(command.encode("ascii"), addr)
            data, source = sock.recvfrom(2048)
            reply = data.decode("utf-8", errors="ignore").strip()
            if reply != expected:
                print(f"[PROBE][FAIL] {name}: expected={expected!r}, got={reply!r} from {source}", flush=True)
                ok_all = False
            else:
                print(f"[PROBE][OK] {name} command channel: {reply}", flush=True)
        except Exception as exc:
            print(f"[PROBE][FAIL] {name} {addr[0]}:{addr[1]}: {exc}", flush=True)
            ok_all = False
        finally:
            sock.close()

    return 0 if ok_all else 3


def main() -> None:
    try:
        READY_FILE.unlink(missing_ok=True)
    except Exception:
        pass
    try:
        PID_FILE.write_text(str(os.getpid()), encoding="ascii")
    except Exception:
        pass

    uav = None
    car = None
    try:
        uav = MockDevice("UAV", UAV_ADDR)
        car = MockDevice("CAR", CAR_ADDR)
        uav.start()
        car.start()
        threading.Thread(target=telemetry_loop, args=(uav, car), daemon=True).start()

        READY_FILE.write_text(
            f"pid={os.getpid()}\nuav={UAV_ADDR[0]}:{UAV_ADDR[1]}\ncar={CAR_ADDR[0]}:{CAR_ADDR[1]}\n",
            encoding="ascii",
        )

        print("=" * 68, flush=True)
        print("UAV + CAR Mock is READY", flush=True)
        print(f"  UAV listen: {UAV_ADDR[0]}:{UAV_ADDR[1]}", flush=True)
        print(f"  CAR listen: {CAR_ADDR[0]}:{CAR_ADDR[1]}", flush=True)
        print(f"  Telemetry -> GS: {GS_ADDR[0]}:{GS_ADDR[1]}", flush=True)
        print("Commands will be printed as RX, ACK replies as TX.", flush=True)
        print("Operation: choose task -> PREPARE -> PING -> START", flush=True)
        print("Press Ctrl+C to exit.", flush=True)
        print("=" * 68, flush=True)

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nMock stopped by user.", flush=True)
    except Exception as exc:
        print("\n[FATAL] Mock startup failed:", flush=True)
        print(str(exc), flush=True)
        print("Close any old UAV + CAR MOCK window, then run start_demo.bat again.", flush=True)
        sys.exit(2)
    finally:
        try:
            READY_FILE.unlink(missing_ok=True)
        except Exception:
            pass
        try:
            PID_FILE.unlink(missing_ok=True)
        except Exception:
            pass
        if uav is not None:
            uav.close()
        if car is not None:
            car.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UAV + CAR UDP mock")
    parser.add_argument("--probe", action="store_true", help="probe an already running mock and exit")
    parser.add_argument("--probe-timeout", type=float, default=2.0)
    cli_args = parser.parse_args()
    if cli_args.probe:
        raise SystemExit(probe_mock(cli_args.probe_timeout))
    main()
