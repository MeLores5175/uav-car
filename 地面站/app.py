#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""陆空协同无人机系统 HTML 地面站（V1）

功能：
- aiohttp 本地网页服务 + 原生 WebSocket 实时推送；
- UDP 同时与无人机和小车通信；
- 支持任务 1/任务 2、BOOT/MODE、PING、STATUS、START、LAND、RESET；
- 解析 ACK/ERR/EVT/HB/TEL 以及旧仓库 STATUS/VISION 报文；
- 关键命令自动重发，使用 cmd_id 匹配 ACK 并防止误判；
- 浏览器端显示横向比赛地图、无人机/小车位置、轨迹、状态和事件日志。

运行：
    python app.py
    python app.py --config config.mock.json
"""

from __future__ import annotations

import argparse
import asyncio
import ipaddress
import json
import logging
import os
import shutil
import time
import webbrowser
from collections import deque
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from aiohttp import WSMsgType, web

BASE_DIR = Path(__file__).resolve().parent


class UdpReceiver(asyncio.DatagramProtocol):
    def __init__(self, station: "GroundStation") -> None:
        self.station = station
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]
        self.station.udp_transport = self.transport
        sockname = self.transport.get_extra_info("sockname")
        self.station.add_log("INFO", "GS", f"UDP 已监听 {sockname[0]}:{sockname[1]}")

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        try:
            text = data.decode("utf-8", errors="ignore").strip()
        except Exception as exc:
            self.station.add_log("ERROR", "UDP", f"来自 {addr} 的报文解码失败：{exc}")
            return
        if text:
            self.station.handle_udp_message(text, addr)

    def error_received(self, exc: Exception) -> None:
        self.station.add_log("ERROR", "UDP", f"UDP 接收错误：{exc}")


class GroundStation:
    def __init__(self, config: Dict[str, Any], config_path: Path) -> None:
        self.config = config
        self.config_path = config_path
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.udp_transport: Optional[asyncio.DatagramTransport] = None
        self.ws_clients: set[web.WebSocketResponse] = set()
        self.pending: Dict[str, Dict[str, Any]] = {}
        self.command_counter = 0
        self.run_counter = 0
        self.log_counter = 0
        self.logs: deque[Dict[str, Any]] = deque(maxlen=300)
        self.dirty = asyncio.Event()
        self.last_broadcast = 0.0

        self.state: Dict[str, Any] = {
            "selected_task": None,
            "task_locked": False,
            "mission": {
                "run_id": "",
                "status": "IDLE",
                "start_epoch": None,
                "last_event": "",
            },
            "uav": self._new_device_state("UAV"),
            "car": self._new_device_state("CAR"),
            "last_vision": {},
        }

    @staticmethod
    def _new_device_state(name: str) -> Dict[str, Any]:
        return {
            "name": name,
            "last_seen_epoch": 0.0,
            "last_message": "",
            "heartbeat": {"seq": None, "state": "UNKNOWN"},
            "telemetry": {},
            "raw_status": {},
        }

    def _default_device_ip(self, device: str) -> str:
        fallback = {
            "uav": "192.168.151.102",
            "car": "192.168.151.103",
        }
        defaults = self.config.get("defaults", {}).get("devices", {})
        item = defaults.get(device, {}) if isinstance(defaults, dict) else {}
        value = item.get("ip") if isinstance(item, dict) else None
        return str(value or fallback[device])

    def build_settings_payload(self) -> Dict[str, Any]:
        mission_status = str(self.state["mission"].get("status", "IDLE")).upper()
        busy = mission_status in {"STARTING", "RUNNING", "ABORTING"}
        return {
            "ok": True,
            "config_file": self.config_path.name,
            "can_edit": not busy,
            "mission_status": mission_status,
            "network": {
                "bind_ip": str(self.config["network"].get("bind_ip", "0.0.0.0")),
                "gs_port": int(self.config["network"]["gs_port"]),
            },
            "devices": {
                "uav": {
                    "ip": str(self.config["devices"]["uav"]["ip"]),
                    "port": int(self.config["devices"]["uav"]["port"]),
                    "default_ip": self._default_device_ip("uav"),
                },
                "car": {
                    "ip": str(self.config["devices"]["car"]["ip"]),
                    "port": int(self.config["devices"]["car"]["port"]),
                    "default_ip": self._default_device_ip("car"),
                },
            },
        }

    @staticmethod
    def _validate_ipv4(value: Any, label: str) -> str:
        text = str(value or "").strip()
        try:
            address = ipaddress.ip_address(text)
        except ValueError as exc:
            raise ValueError(f"{label}不是有效的 IPv4 地址：{text or '空'}") from exc
        if address.version != 4:
            raise ValueError(f"{label}只允许填写 IPv4 地址")
        return str(address)

    def _persist_config(self, config: Dict[str, Any]) -> None:
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        backup_path = self.config_path.with_name(self.config_path.name + ".bak")
        temp_path = self.config_path.with_name(self.config_path.name + ".tmp")

        if self.config_path.exists():
            shutil.copy2(self.config_path, backup_path)

        temp_path.write_text(
            json.dumps(config, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temp_path, self.config_path)

    def _clear_device_runtime_state(self) -> None:
        for device in ("uav", "car"):
            self.state[device]["last_seen_epoch"] = 0.0
            self.state[device]["last_message"] = ""
            self.state[device]["heartbeat"] = {"seq": None, "state": "UNKNOWN"}
            self.state[device]["telemetry"] = {}
            self.state[device]["raw_status"] = {}

    def add_log(self, level: str, source: str, text: str) -> None:
        self.log_counter += 1
        item = {
            "id": self.log_counter,
            "time": time.strftime("%H:%M:%S"),
            "epoch": time.time(),
            "level": level.upper(),
            "source": source,
            "text": text,
        }
        self.logs.append(item)
        logging.info("[%s][%s] %s", level, source, text)
        self.mark_dirty()

    def mark_dirty(self) -> None:
        if self.loop and self.loop.is_running():
            self.dirty.set()

    def _device_addr(self, device: str) -> Tuple[str, int]:
        dev = self.config["devices"][device]
        return str(dev["ip"]), int(dev["port"])

    def _device_from_addr(self, addr: Tuple[str, int]) -> Optional[str]:
        for device in ("uav", "car"):
            ip, port = self._device_addr(device)
            if addr[0] == ip and addr[1] == port:
                return device
        # 允许设备使用临时源端口；IP 匹配时仍归类。
        for device in ("uav", "car"):
            ip, _ = self._device_addr(device)
            if addr[0] == ip:
                return device
        return None

    def _mark_seen(self, device: str, text: str) -> None:
        if device not in ("uav", "car"):
            return
        self.state[device]["last_seen_epoch"] = time.time()
        self.state[device]["last_message"] = text[:500]
        self.mark_dirty()

    def next_cmd_id(self) -> str:
        self.command_counter = (self.command_counter + 1) % 10000
        if self.command_counter == 0:
            self.command_counter = 1
        return f"{self.command_counter:04d}"

    def next_run_id(self) -> str:
        self.run_counter += 1
        return f"R{self.run_counter:03d}"

    def send_raw(self, device: str, text: str) -> None:
        if not self.udp_transport:
            self.add_log("ERROR", "GS", "UDP 尚未启动，无法发送命令")
            return
        addr = self._device_addr(device)
        self.udp_transport.sendto(text.encode("utf-8"), addr)

    def send_command(
        self,
        device: str,
        action: str,
        args: Optional[list[str]] = None,
        emergency: bool = False,
    ) -> str:
        if device not in ("uav", "car"):
            raise ValueError(f"未知设备：{device}")
        args = args or []
        cmd_id = self.next_cmd_id()
        action = action.upper().strip()
        parts = ["CMD", cmd_id, action] + [str(x).strip() for x in args if str(x).strip()]
        text = ":".join(parts)

        proto_cfg = self.config.get("protocol", {})
        if emergency:
            retries = int(proto_cfg.get("land_retries", 5))
            interval = float(proto_cfg.get("land_retry_interval_ms", 150)) / 1000.0
        else:
            retries = int(proto_cfg.get("command_retries", 3))
            interval = float(proto_cfg.get("command_retry_interval_ms", 300)) / 1000.0

        self.pending[cmd_id] = {
            "device": device,
            "action": action,
            "text": text,
            "attempt": 0,
            "max_attempts": retries,
            "interval": interval,
            "created_epoch": time.time(),
        }
        asyncio.create_task(self._reliable_send_loop(cmd_id))
        return cmd_id

    async def _reliable_send_loop(self, cmd_id: str) -> None:
        entry = self.pending.get(cmd_id)
        if not entry:
            return
        while cmd_id in self.pending and entry["attempt"] < entry["max_attempts"]:
            entry["attempt"] += 1
            self.send_raw(entry["device"], entry["text"])
            verb = "发送" if entry["attempt"] == 1 else "重发"
            self.add_log(
                "INFO" if entry["attempt"] == 1 else "WARN",
                "GS",
                f"{verb} → {entry['device'].upper()}：{entry['text']}（{entry['attempt']}/{entry['max_attempts']}）",
            )
            await asyncio.sleep(entry["interval"])

        if cmd_id in self.pending:
            expired = self.pending.pop(cmd_id)
            self.add_log(
                "ERROR",
                "GS",
                f"命令超时：{expired['device'].upper()} {expired['action']}，未收到 ACK",
            )
            self.mark_dirty()

    def handle_udp_message(self, text: str, addr: Tuple[str, int]) -> None:
        device = self._device_from_addr(addr)
        source = device.upper() if device else f"{addr[0]}:{addr[1]}"
        if device:
            self._mark_seen(device, text)

        try:
            if text.startswith("TEL:UAV:"):
                self._handle_telemetry("uav", text.split(":", 2)[2])
                return
            if text.startswith("TEL:CAR:"):
                self._handle_telemetry("car", text.split(":", 2)[2])
                return
            if text.startswith("VISION:IMAGE:"):
                self._handle_vision(text.split(":", 2)[2], source)
                return

            parts = text.split(":")
            prefix = parts[0].upper() if parts else ""
            if prefix in ("ACK", "ERR"):
                self._handle_ack_or_err(prefix, parts, source, text)
            elif prefix == "EVT":
                self._handle_event(parts[1:], source)
            elif prefix == "HB":
                self._handle_heartbeat(parts[1:], source, device)
            elif prefix == "STATUS":
                self._handle_legacy_status(text, source, device)
            else:
                self.add_log("WARN", source, f"收到未识别报文：{text}")
        except Exception as exc:
            self.add_log("ERROR", source, f"报文解析异常：{exc}；原文：{text}")
        finally:
            self.mark_dirty()

    def _handle_ack_or_err(self, prefix: str, parts: list[str], source: str, raw: str) -> None:
        # V1.1: ACK:<cmd_id>:<action>:<result>[:detail]
        if len(parts) >= 4 and parts[1].isdigit():
            cmd_id, action, result = parts[1], parts[2].upper(), parts[3]
            detail = ":".join(parts[4:])
            pending = self.pending.pop(cmd_id, None)
            if prefix == "ACK":
                self.add_log("OK", source, f"{action} {result}" + (f"：{detail}" if detail else ""))
            else:
                self.add_log("ERROR", source, f"{action} {result}" + (f"：{detail}" if detail else ""))
            if pending and action == "BOOT" and prefix == "ACK":
                self.state["uav"]["telemetry"]["boot"] = "STARTING"
            if pending and action == "MODE" and prefix == "ACK":
                self.state["car"]["telemetry"]["state"] = "READY"
            if pending and action == "LAND" and prefix == "ACK":
                self.state["uav"]["telemetry"]["safety"] = "ABORTING"
            return

        # 兼容旧仓库，例如 ACK:PING:NORMAL / ACK:START:OK
        level = "OK" if prefix == "ACK" else "ERROR"
        self.add_log(level, source, f"旧格式回包：{raw}")

    def _handle_event(self, args: list[str], source: str) -> None:
        if not args:
            return
        event = args[0].upper()
        params = args[1:]
        self.state["mission"]["last_event"] = event

        if event == "UAV_BOOT_READY":
            self.state["uav"]["telemetry"].update({"boot": "READY", "state": "WAIT_START"})
        elif event == "CAR_READY":
            self.state["car"]["telemetry"]["state"] = "READY"
        elif event == "MISSION_START":
            run_id = params[0] if params else self.state["mission"].get("run_id", "")
            self.state["mission"].update(
                {"run_id": run_id, "status": "RUNNING", "start_epoch": time.time()}
            )
        elif event == "CAR_POINT":
            if params:
                self.state["car"]["telemetry"]["point"] = params[-1]
        elif event == "UAV_FOLLOW_ESTABLISHED":
            self.state["uav"]["telemetry"]["state"] = "FOLLOW"
        elif event == "UAV_DROP_DONE":
            self.state["uav"]["telemetry"]["state"] = "DROP_DONE"
        elif event == "UAV_LAND_ON_CAR":
            self.state["uav"]["telemetry"]["state"] = "ON_CAR"
        elif event == "UAV_TAKEOFF_FROM_CAR":
            self.state["uav"]["telemetry"]["state"] = "TAKEOFF_FROM_CAR"
        elif event == "UAV_ABORTING":
            self.state["uav"]["telemetry"]["safety"] = "ABORTING"
        elif event == "UAV_LANDING":
            self.state["uav"]["telemetry"].update({"safety": "LANDING", "state": "LAND_HOME"})
        elif event == "UAV_LANDED":
            self.state["uav"]["telemetry"].update({"safety": "LANDED", "armed": False, "z_cm": 0})
        elif event == "MISSION_DONE":
            self.state["mission"]["status"] = "DONE"

        detail = ":".join(params)
        self.add_log("EVENT", source, event + (f"：{detail}" if detail else ""))

    def _handle_heartbeat(self, args: list[str], source: str, device: Optional[str]) -> None:
        # HB:UAV:<seq>:<state> / HB:CAR:<seq>:<state>
        if len(args) < 3:
            self.add_log("WARN", source, "心跳格式不完整")
            return
        named = args[0].lower()
        target = named if named in ("uav", "car") else device
        if not target:
            return
        self._mark_seen(target, ":".join(["HB"] + args))
        self.state[target]["heartbeat"] = {"seq": args[1], "state": args[2]}
        # 心跳状态只在没有更详细遥测状态时补充。
        self.state[target]["telemetry"].setdefault("state", args[2])

    def _handle_telemetry(self, device: str, payload: str) -> None:
        data = json.loads(payload)
        if not isinstance(data, dict):
            raise ValueError("TEL JSON 不是对象")
        self._mark_seen(device, f"TEL:{device.upper()}")
        self.state[device]["telemetry"].update(data)
        # 任务选择只由地面站操作决定。设备遥测中的 task 仅用于状态显示，
        # 避免设备默认模式反复覆盖用户尚未提交的网页选择。
        if data.get("run"):
            self.state["mission"]["run_id"] = str(data["run"])
        self.mark_dirty()

    def _handle_vision(self, payload: str, source: str) -> None:
        data = json.loads(payload)
        if isinstance(data, dict):
            self.state["last_vision"] = data
            cls = data.get("class_name") or "none"
            conf = data.get("confidence", 0)
            self.add_log("VISION", source, f"视觉：{cls}，置信度 {conf}")

    def _handle_legacy_status(self, text: str, source: str, device: Optional[str]) -> None:
        body = text[len("STATUS:") :]
        result: Dict[str, str] = {}
        for item in body.split(";"):
            if "=" in item:
                key, value = item.split("=", 1)
                result[key.strip()] = value.strip()
        target = device or "uav"
        self.state[target]["raw_status"].update(result)
        # 将旧字段映射到界面常用字段。
        tel = self.state[target]["telemetry"]
        if "fsm" in result:
            tel["state"] = result["fsm"]
        if "safety" in result:
            tel["safety"] = result["safety"]
        if "armed" in result:
            tel["armed"] = result["armed"].lower() == "true"
        if "connected" in result:
            tel["fcu"] = result["connected"].lower() == "true"
        if "mode" in result:
            tel["mode"] = result["mode"]
        self.add_log("INFO", source, "收到旧格式 STATUS")

    def _link_state(self, last_seen_epoch: float) -> Dict[str, Any]:
        if not last_seen_epoch:
            return {"state": "OFFLINE", "age_s": None}
        age = max(0.0, time.time() - float(last_seen_epoch))
        if age < 1.5:
            state = "ONLINE"
        elif age < 3.0:
            state = "DELAYED"
        else:
            state = "OFFLINE"
        return {"state": state, "age_s": round(age, 2)}

    def build_snapshot(self) -> Dict[str, Any]:
        mission = dict(self.state["mission"])
        start_epoch = mission.get("start_epoch")
        mission["elapsed_ms"] = int((time.time() - start_epoch) * 1000) if start_epoch else 0
        return {
            "server_time": time.time(),
            "selected_task": self.state["selected_task"],
            "task_locked": bool(self.state.get("task_locked", False)),
            "mission": mission,
            "uav": {
                **self.state["uav"],
                "link": self._link_state(self.state["uav"]["last_seen_epoch"]),
            },
            "car": {
                **self.state["car"],
                "link": self._link_state(self.state["car"]["last_seen_epoch"]),
            },
            "last_vision": self.state["last_vision"],
            "pending": [
                {
                    "cmd_id": key,
                    "device": value["device"],
                    "action": value["action"],
                    "attempt": value["attempt"],
                    "max_attempts": value["max_attempts"],
                }
                for key, value in self.pending.items()
            ],
            "logs": list(self.logs)[-120:],
            "network": {
                "gs_port": self.config["network"]["gs_port"],
                "uav": self.config["devices"]["uav"],
                "car": self.config["devices"]["car"],
            },
        }

    async def broadcast_loop(self) -> None:
        while True:
            try:
                await asyncio.wait_for(self.dirty.wait(), timeout=0.25)
            except asyncio.TimeoutError:
                pass
            self.dirty.clear()
            now = time.monotonic()
            # 20 Hz 上限，使地图标记在浏览器中移动更平滑。
            if now - self.last_broadcast < 0.045:
                await asyncio.sleep(0.045 - (now - self.last_broadcast))
            self.last_broadcast = time.monotonic()
            await self.broadcast({"type": "snapshot", "data": self.build_snapshot()})

    async def broadcast(self, payload: Dict[str, Any]) -> None:
        if not self.ws_clients:
            return
        text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        dead: list[web.WebSocketResponse] = []
        for ws in list(self.ws_clients):
            try:
                await ws.send_str(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.ws_clients.discard(ws)

    async def ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20.0, receive_timeout=60.0)
        await ws.prepare(request)
        self.ws_clients.add(ws)
        await ws.send_json({"type": "snapshot", "data": self.build_snapshot()}, dumps=lambda x: json.dumps(x, ensure_ascii=False))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    if msg.data == "ping":
                        await ws.send_str("pong")
                elif msg.type in (WSMsgType.ERROR, WSMsgType.CLOSE, WSMsgType.CLOSED):
                    break
        finally:
            self.ws_clients.discard(ws)
        return ws

    async def api_state(self, request: web.Request) -> web.Response:
        return web.json_response(self.build_snapshot(), dumps=lambda x: json.dumps(x, ensure_ascii=False))

    async def api_settings_get(self, request: web.Request) -> web.Response:
        return web.json_response(
            self.build_settings_payload(),
            dumps=lambda x: json.dumps(x, ensure_ascii=False),
        )

    async def api_settings_save(self, request: web.Request) -> web.Response:
        mission_status = str(self.state["mission"].get("status", "IDLE")).upper()
        if mission_status in {"STARTING", "RUNNING", "ABORTING"}:
            return web.json_response(
                {
                    "ok": False,
                    "error": "任务正在运行或降落中，禁止修改通信 IP。请任务结束后再设置。",
                },
                status=409,
                dumps=lambda x: json.dumps(x, ensure_ascii=False),
            )

        body = await self._json_body(request)
        try:
            uav_ip = self._validate_ipv4(body.get("uav_ip"), "无人机 IP")
            car_ip = self._validate_ipv4(body.get("car_ip"), "小车 IP")
        except ValueError as exc:
            return web.json_response(
                {"ok": False, "error": str(exc)},
                status=400,
                dumps=lambda x: json.dumps(x, ensure_ascii=False),
            )

        new_config = json.loads(json.dumps(self.config))
        new_config["devices"]["uav"]["ip"] = uav_ip
        new_config["devices"]["car"]["ip"] = car_ip

        try:
            self._persist_config(new_config)
        except Exception as exc:
            self.add_log("ERROR", "SETTINGS", f"保存 IP 配置失败：{exc}")
            return web.json_response(
                {"ok": False, "error": f"配置文件写入失败：{exc}"},
                status=500,
                dumps=lambda x: json.dumps(x, ensure_ascii=False),
            )

        cancelled = len(self.pending)
        self.pending.clear()
        self.config = new_config
        self._clear_device_runtime_state()
        self.add_log(
            "INFO",
            "SETTINGS",
            f"通信 IP 已更新并立即应用：UAV={uav_ip}，CAR={car_ip}"
            + (f"；已取消 {cancelled} 条旧地址待确认命令" if cancelled else ""),
        )
        self.mark_dirty()

        payload = self.build_settings_payload()
        payload["message"] = "保存成功，新的 IP 已立即生效，无需重启地面站。"
        return web.json_response(
            payload,
            dumps=lambda x: json.dumps(x, ensure_ascii=False),
        )

    @staticmethod
    async def _json_body(request: web.Request) -> Dict[str, Any]:
        try:
            body = await request.json()
            return body if isinstance(body, dict) else {}
        except Exception:
            return {}

    async def api_prepare(self, request: web.Request) -> web.Response:
        body = await self._json_body(request)
        task = str(body.get("task", "")).upper()
        if task not in ("T1", "T2"):
            return web.json_response({"ok": False, "error": "task 必须为 T1 或 T2"}, status=400)
        if self.state.get("task_locked", False):
            locked_task = self.state.get("selected_task")
            return web.json_response(
                {"ok": False, "error": f"任务已锁定为 {locked_task}，请先 RESET 后再切换"},
                status=409,
            )
        self.state["selected_task"] = task
        self.state["task_locked"] = True
        self.state["mission"].update({"status": "PREPARING", "start_epoch": None, "last_event": ""})
        self.state["uav"]["telemetry"].update({"boot": "STARTING", "state": "STARTING"})
        self.state["car"]["telemetry"].update({"task": 1 if task == "T1" else 2, "state": "PREPARING"})
        uav_id = self.send_command("uav", "BOOT", [task])
        car_id = self.send_command("car", "MODE", [task])
        self.add_log("INFO", "GS", f"准备任务 {task[-1]}：无人机 BOOT + 小车 MODE")
        return web.json_response({"ok": True, "uav_cmd_id": uav_id, "car_cmd_id": car_id})

    async def api_ping(self, request: web.Request) -> web.Response:
        uav_id = self.send_command("uav", "PING")
        car_id = self.send_command("car", "PING")
        return web.json_response({"ok": True, "uav_cmd_id": uav_id, "car_cmd_id": car_id})

    async def api_status(self, request: web.Request) -> web.Response:
        uav_id = self.send_command("uav", "STATUS")
        car_id = self.send_command("car", "STATUS")
        return web.json_response({"ok": True, "uav_cmd_id": uav_id, "car_cmd_id": car_id})

    async def api_start(self, request: web.Request) -> web.Response:
        task = self.state.get("selected_task")
        if task not in ("T1", "T2") or not self.state.get("task_locked", False):
            return web.json_response({"ok": False, "error": "请先选择并准备任务"}, status=400)
        run_id = self.next_run_id()
        self.state["mission"].update(
            {"run_id": run_id, "status": "STARTING", "start_epoch": time.time(), "last_event": ""}
        )
        cmd_id = self.send_command("car", "START", [run_id])
        self.add_log("EVENT", "GS", f"任务启动请求：{run_id} / {task}（仅发送给小车）")
        return web.json_response({"ok": True, "cmd_id": cmd_id, "run_id": run_id})

    async def api_land(self, request: web.Request) -> web.Response:
        run_id = self.state["mission"].get("run_id") or "R000"
        self.state["mission"]["status"] = "ABORTING"
        self.state["uav"]["telemetry"]["safety"] = "ABORTING"
        cmd_id = self.send_command("uav", "LAND", [run_id], emergency=True)
        self.add_log("ERROR", "GS", f"已发送安全中止并原地降落：{run_id}")
        return web.json_response({"ok": True, "cmd_id": cmd_id})

    async def api_reset(self, request: web.Request) -> web.Response:
        uav_id = self.send_command("uav", "RESET")
        car_id = self.send_command("car", "RESET")
        self.state["selected_task"] = None
        self.state["task_locked"] = False
        self.state["mission"].update({"run_id": "", "status": "IDLE", "start_epoch": None, "last_event": ""})
        self.add_log("INFO", "GS", "已向无人机和小车发送 RESET，任务选择已解锁")
        return web.json_response({"ok": True, "uav_cmd_id": uav_id, "car_cmd_id": car_id})

    async def api_stop_nodes(self, request: web.Request) -> web.Response:
        cmd_id = self.send_command("uav", "STOP_NODES")
        return web.json_response({"ok": True, "cmd_id": cmd_id})

    async def api_clear_trails(self, request: web.Request) -> web.Response:
        await self.broadcast({"type": "clear_trails"})
        self.add_log("INFO", "GS", "已清空网页轨迹")
        return web.json_response({"ok": True})

    async def index(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(BASE_DIR / "templates" / "index.html")

    async def settings_page(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(BASE_DIR / "templates" / "settings.html")

    async def on_startup(self, app: web.Application) -> None:
        self.loop = asyncio.get_running_loop()
        network = self.config["network"]
        await self.loop.create_datagram_endpoint(
            lambda: UdpReceiver(self),
            local_addr=(str(network.get("bind_ip", "0.0.0.0")), int(network["gs_port"])),
        )
        asyncio.create_task(self.broadcast_loop())
        ui = self.config.get("ui", {})
        if bool(ui.get("auto_open_browser", True)):
            host = str(ui.get("browser_host", "127.0.0.1"))
            port = int(ui.get("port", 5000))
            self.loop.call_later(1.0, lambda: webbrowser.open(f"http://{host}:{port}"))

    async def on_cleanup(self, app: web.Application) -> None:
        if self.udp_transport:
            self.udp_transport.close()
        for ws in list(self.ws_clients):
            await ws.close()


def load_config(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        config = json.load(f)
    required = [
        ("network", "gs_port"),
        ("devices", "uav"),
        ("devices", "car"),
        ("ui", "port"),
    ]
    for group, key in required:
        if group not in config or key not in config[group]:
            raise ValueError(f"配置缺少 {group}.{key}")
    return config


def create_app(config: Dict[str, Any], config_path: Path) -> web.Application:
    station = GroundStation(config, config_path)
    app = web.Application(client_max_size=2 * 1024 * 1024)
    app["station"] = station
    app.router.add_get("/", station.index)
    app.router.add_get("/settings", station.settings_page)
    app.router.add_get("/ws", station.ws_handler)
    app.router.add_get("/api/state", station.api_state)
    app.router.add_get("/api/settings", station.api_settings_get)
    app.router.add_post("/api/settings", station.api_settings_save)
    app.router.add_post("/api/prepare", station.api_prepare)
    app.router.add_post("/api/ping", station.api_ping)
    app.router.add_post("/api/status", station.api_status)
    app.router.add_post("/api/start", station.api_start)
    app.router.add_post("/api/land", station.api_land)
    app.router.add_post("/api/reset", station.api_reset)
    app.router.add_post("/api/stop_nodes", station.api_stop_nodes)
    app.router.add_post("/api/clear_trails", station.api_clear_trails)
    app.router.add_static("/static", BASE_DIR / "static", show_index=False)
    app.on_startup.append(station.on_startup)
    app.on_cleanup.append(station.on_cleanup)
    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="陆空协同无人机系统 HTML 地面站")
    parser.add_argument("--config", default="config.json", help="配置文件路径")
    args = parser.parse_args()

    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = BASE_DIR / config_path
    config = load_config(config_path)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    ui = config["ui"]
    app = create_app(config, config_path)
    web.run_app(
        app,
        host=str(ui.get("host", "0.0.0.0")),
        port=int(ui.get("port", 5000)),
        print=lambda msg: logging.info(msg),
        access_log=None,
    )


if __name__ == "__main__":
    main()
