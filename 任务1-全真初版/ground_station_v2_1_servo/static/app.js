"use strict";

const ui = {
  wsBadge: document.getElementById("wsBadge"),
  clockText: document.getElementById("clockText"),
  taskOptions: [...document.querySelectorAll(".task-option")],
  prepareBtn: document.getElementById("prepareBtn"),
  pingBtn: document.getElementById("pingBtn"),
  statusBtn: document.getElementById("statusBtn"),
  startBtn: document.getElementById("startBtn"),
  resetBtn: document.getElementById("resetBtn"),
  landSlider: document.getElementById("landSlider"),
  landSliderCard: document.getElementById("landSliderCard"),
  landSliderHint: document.getElementById("landSliderHint"),
  clearTrailBtn: document.getElementById("clearTrailBtn"),
  uavMarker: document.getElementById("uavMarker"),
  carMarker: document.getElementById("carMarker"),
  uavTrail: document.getElementById("uavTrail"),
  carTrail: document.getElementById("carTrail"),
  toast: document.getElementById("toast"),
  logList: document.getElementById("logList"),
  missionStatus: document.getElementById("missionStatus"),
  missionLock: document.getElementById("missionLock"),
  servoState: document.getElementById("servoState"),
  servoHint: document.getElementById("servoHint"),
  servoLockBtn: document.getElementById("servoLockBtn"),
  servoReleaseBtn: document.getElementById("servoReleaseBtn"),
};

let selectedTask = "T1";
let snapshot = null;
let socket = null;
let reconnectTimer = null;
let toastTimer = null;
let uavTrailPoints = [];
let carTrailPoints = [];
let lastUavTrailKey = "";
let lastCarTrailKey = "";
let taskLocked = false;
let landSending = false;
let servoSending = false;

const stateLabels = {
  IDLE: "空闲",
  STOPPED: "节点已停止",
  STARTING: "启动中",
  READY: "准备完成",
  FAILED: "启动失败",
  WAIT_START: "等待启动",
  TAKEOFF: "起飞中",
  HOVER: "定高悬停",
  SEARCH_CAR: "搜索小车",
  FOLLOW: "伴飞中",
  FOLLOW_CAR: "伴飞中",
  PREPARE_DROP: "准备抛投",
  DROPPING: "正在抛投",
  DROP_DONE: "抛投完成",
  RETURN_HOME: "返回 H 点",
  LAND_HOME: "H 点降落",
  APPROACH_CAR: "接近小车",
  LAND_ON_CAR: "动态降落",
  ON_CAR: "已降落小车",
  TAKEOFF_FROM_CAR: "从小车起飞",
  DONE: "任务完成",
  RUNNING: "循线运行",
  PASS_B: "已过 B 点",
  PASS_C: "已过 C 点",
  PASS_D: "已过 D 点",
  RETURN_A: "返回 A 点",
  FINISHED: "循线完成",
  LINE_LOST: "循线丢失",
  FAULT: "故障",
  NORMAL: "正常",
  WARNING: "警告",
  ABORTING: "安全中止",
  ABORTED: "任务已中止",
  LANDING: "受控降落",
  LANDED: "已落地",
  PREPARING: "准备中",
  MISSION_START: "任务开始",
  LOCKED: "A30 已锁止",
  RELEASED: "A90 已释放",
  COMMANDING: "命令发送中",
  UNKNOWN: "未知",
};

function label(value, fallback = "--") {
  if (value === undefined || value === null || value === "") return fallback;
  const key = String(value).toUpperCase();
  return stateLabels[key] || String(value);
}

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function numberValue(value) {
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

function fmt(value, digits = 1, suffix = "") {
  const n = numberValue(value);
  return n === null ? "--" : `${n.toFixed(digits)}${suffix}`;
}

function boolText(value) {
  if (value === true || String(value).toLowerCase() === "true") return "是";
  if (value === false || String(value).toLowerCase() === "false") return "否";
  return "--";
}

function taskText(task) {
  if (task === "T1" || task === 1 || task === "1") return "任务 1";
  if (task === "T2" || task === 2 || task === "2") return "任务 2";
  return "未选择";
}

function showToast(message, isError = false) {
  clearTimeout(toastTimer);
  ui.toast.textContent = message;
  ui.toast.classList.toggle("error", isError);
  ui.toast.classList.remove("hidden");
  toastTimer = setTimeout(() => ui.toast.classList.add("hidden"), 2600);
}

async function api(path, body = {}) {
  try {
    const response = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok || data.ok === false) {
      throw new Error(data.error || `请求失败：HTTP ${response.status}`);
    }
    return data;
  } catch (error) {
    showToast(error.message || String(error), true);
    throw error;
  }
}

function connectWebSocket() {
  clearTimeout(reconnectTimer);
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${scheme}://${location.host}/ws`);
  ui.wsBadge.textContent = "网页连接中";
  ui.wsBadge.className = "status-badge offline";

  socket.onopen = () => {
    ui.wsBadge.textContent = "WebSocket 已连接";
    ui.wsBadge.className = "status-badge online";
  };

  socket.onmessage = (event) => {
    if (event.data === "pong") return;
    try {
      const message = JSON.parse(event.data);
      if (message.type === "snapshot") {
        snapshot = message.data;
        render(snapshot);
      } else if (message.type === "clear_trails") {
        clearTrailsLocal();
      }
    } catch (error) {
      console.error("WebSocket 数据解析失败", error, event.data);
    }
  };

  socket.onclose = () => {
    ui.wsBadge.textContent = "网页连接断开";
    ui.wsBadge.className = "status-badge offline";
    reconnectTimer = setTimeout(connectWebSocket, 1200);
  };

  socket.onerror = () => socket.close();
}

function render(data) {
  if (data.selected_task) selectedTask = data.selected_task;
  taskLocked = Boolean(data.task_locked);
  renderTaskSelector();
  renderMission(data);
  renderDevice("uav", data.uav || {});
  renderDevice("car", data.car || {});
  renderMap(data);
  renderServoControls(data);
  renderLogs(data.logs || []);
  setText("pendingCommands", String((data.pending || []).length));

  const net = data.network || {};
  const uav = net.uav || {};
  const car = net.car || {};
  setText(
    "networkSummary",
    `GS UDP ${net.gs_port ?? "--"} · UAV ${uav.ip ?? "--"}:${uav.port ?? "--"} · CAR ${car.ip ?? "--"}:${car.port ?? "--"}`
  );
}

function renderTaskSelector() {
  ui.taskOptions.forEach((button) => {
    const active = button.dataset.task === selectedTask;
    button.classList.toggle("active", active);
    button.classList.toggle("locked-out", taskLocked && !active);
    button.disabled = taskLocked;
    button.setAttribute("aria-checked", active ? "true" : "false");
    button.setAttribute("aria-disabled", taskLocked ? "true" : "false");
    button.title = taskLocked ? "任务已锁定；点击 RESET 双方后才能重新选择" : "";
  });
}

function renderMission(data) {
  const mission = data.mission || {};
  const status = String(mission.status || "IDLE").toUpperCase();

  setText("missionTask", taskText(data.selected_task));
  setText("missionRun", mission.run_id || "--");
  setText("missionStatus", label(status, "空闲"));
  setText("lastEvent", label(mission.last_event, "--"));

  if (ui.missionStatus) {
    const statusClass =
      ["RUNNING", "STARTING"].includes(status) ? "running" :
      ["PREPARING"].includes(status) ? "preparing" :
      ["DONE"].includes(status) ? "done" :
      ["ABORTING", "ABORTED", "FAILED", "FAULT"].includes(status) ? "danger" :
      "idle";
    ui.missionStatus.className = `mission-status-text ${statusClass}`;
  }

  if (ui.missionLock) {
    ui.missionLock.textContent = data.task_locked
      ? `${taskText(data.selected_task)} 已锁定`
      : "任务未锁定";
    ui.missionLock.className = `mission-lock-badge ${data.task_locked ? "locked" : "unlocked"}`;
  }

  const active = ["STARTING", "RUNNING", "ABORTING"].includes(status);
  const terminal = ["DONE", "ABORTED", "FAILED", "FAULT"].includes(status);
  ui.startBtn.disabled =
    active || terminal || !data.selected_task || !data.task_locked;
  ui.prepareBtn.disabled = active || Boolean(data.task_locked);
}

function renderDevice(device, data) {
  const tel = data.telemetry || {};
  const link = data.link || { state: "OFFLINE" };
  const prefix = device === "uav" ? "uav" : "car";
  const dot = document.getElementById(`${prefix}LinkDot`);
  const linkText = document.getElementById(`${prefix}LinkText`);
  const state = String(link.state || "OFFLINE").toLowerCase();
  dot.className = `link-dot ${state}`;
  const linkLabel = state === "online" ? "在线" : state === "delayed" ? "延迟" : "离线";
  linkText.textContent = link.age_s == null ? linkLabel : `${linkLabel} · ${link.age_s}s`;

  if (device === "uav") {
    setText("uavStateText", label(tel.state || data.heartbeat?.state, "未知"));
    setText("uavBoot", label(tel.boot, "--"));
    setText("uavSafety", label(tel.safety, "--"));
    setText("uavAltitude", fmt(tel.z_cm, 1, " cm"));
    setText("uavYaw", fmt(tel.yaw_deg, 1, "°"));
    setText("uavMode", tel.mode || "--");
    setText("uavArmed", boolText(tel.armed));
    setText("uavFcu", boolText(tel.fcu));
    setText("uavBattery", fmt(tel.battery, 0, "%"));
  } else {
    setText("carStateText", label(tel.state || data.heartbeat?.state, "未知"));
    setText("carTask", taskText(tel.task));
    setText("carPointCard", tel.point || "--");
    setText("carSpeed", fmt(tel.speed_cm_s, 1, " cm/s"));
    setText("carYaw", fmt(tel.yaw_deg, 1, "°"));
    setText("carLine", tel.line_detected === undefined ? "--" : tel.line_detected ? "正常" : "丢线");
    setText("carBattery", fmt(tel.battery, 0, "%"));
  }
}

function renderServoControls(data) {
  if (!ui.servoLockBtn || !ui.servoReleaseBtn) return;
  const servo = data.servo || {};
  const uav = data.uav || {};
  const tel = uav.telemetry || {};
  const missionStatus = String(data.mission?.status || "IDLE").toUpperCase();
  const armed = tel.armed === true || String(tel.armed).toLowerCase() === "true";
  const linkOnline = String(uav.link?.state || "OFFLINE").toUpperCase() !== "OFFLINE";
  const missionActive = ["STARTING", "RUNNING", "ABORTING"].includes(missionStatus);
  const canManual = linkOnline && !armed && !missionActive && !servoSending;

  ui.servoLockBtn.disabled = !canManual;
  ui.servoReleaseBtn.disabled = !canManual;
  ui.servoState.textContent = label(servo.state, "未知");

  if (!linkOnline) {
    ui.servoHint.textContent = "无人机离线，无法发送舵机命令。";
  } else if (armed) {
    ui.servoHint.textContent = "无人机已解锁，手动舵机控制已禁用。";
  } else if (missionActive) {
    ui.servoHint.textContent = "任务已启动，投放只能由 FSM 自动执行。";
  } else if (servoSending) {
    ui.servoHint.textContent = "正在等待无人机接收舵机命令……";
  } else if (String(servo.state || "").toUpperCase() === "LOCKED") {
    ui.servoHint.textContent = "舵机位于 A30 锁止位，可以装载物块。";
  } else if (String(servo.state || "").toUpperCase() === "RELEASED") {
    ui.servoHint.textContent = "舵机位于 A90 释放位，装载前请重新锁止。";
  } else {
    ui.servoHint.textContent = "ESP32 上电默认 A30；建议装载前再点一次锁止确认。";
  }
}

function fieldToLandscape(xCm, yCm) {
  const x = numberValue(xCm);
  const y = numberValue(yCm);
  if (x === null || y === null) return null;
  // 原场地：X 向右 0~400，Y 向上 0~500。
  // 页面顺时针旋转 90°：SVG_X = 原 Y，SVG_Y = 原 X。
  return {
    sx: Math.max(0, Math.min(500, y)),
    sy: Math.max(0, Math.min(400, x)),
  };
}

function renderMap(data) {
  const uav = data.uav?.telemetry || {};
  const car = data.car?.telemetry || {};
  const uavPos = fieldToLandscape(uav.x_cm, uav.y_cm);
  const carPos = fieldToLandscape(car.x_cm, car.y_cm);

  updateMarker(ui.uavMarker, uavPos, uav.yaw_deg, "uav");
  updateMarker(ui.carMarker, carPos, car.yaw_deg, "car");
  updateTrail("uav", uavPos, uav.seq);
  updateTrail("car", carPos, car.seq);

  const ux = numberValue(uav.x_cm), uy = numberValue(uav.y_cm), uz = numberValue(uav.z_cm);
  const cx = numberValue(car.x_cm), cy = numberValue(car.y_cm);
  setText("uavPosition", ux === null || uy === null ? "--" : `X ${ux.toFixed(1)} / Y ${uy.toFixed(1)} / Z ${uz?.toFixed(1) ?? "--"} cm`);
  setText("carPosition", cx === null || cy === null ? "--" : `X ${cx.toFixed(1)} / Y ${cy.toFixed(1)} cm`);
  setText("carPoint", car.point || "--");

  if (ux !== null && uy !== null && cx !== null && cy !== null) {
    const distance = Math.hypot(ux - cx, uy - cy);
    setText("relativeDistance", `${distance.toFixed(1)} cm`);
  } else {
    setText("relativeDistance", "--");
  }
}

function updateMarker(element, point, yawDeg, type) {
  if (!point) {
    element.classList.add("hidden");
    return;
  }
  element.classList.remove("hidden");
  const yaw = numberValue(yawDeg) ?? 0;
  const angle = 90 - yaw;
  element.setAttribute("transform", `translate(${point.sx} ${point.sy}) rotate(${angle})`);
  // 文字反向旋转，避免跟着航向倒置。
  const text = element.querySelector(".marker-label");
  if (text) text.setAttribute("transform", `rotate(${-angle})`);
}

function updateTrail(device, point, seq) {
  if (!point) return;
  const key = `${seq ?? ""}:${point.sx.toFixed(2)}:${point.sy.toFixed(2)}`;
  if (device === "uav") {
    if (key === lastUavTrailKey) return;
    lastUavTrailKey = key;
    appendTrailPoint(uavTrailPoints, point);
    ui.uavTrail.setAttribute("points", uavTrailPoints.map((p) => `${p.sx},${p.sy}`).join(" "));
  } else {
    if (key === lastCarTrailKey) return;
    lastCarTrailKey = key;
    appendTrailPoint(carTrailPoints, point);
    ui.carTrail.setAttribute("points", carTrailPoints.map((p) => `${p.sx},${p.sy}`).join(" "));
  }
}

function appendTrailPoint(list, point) {
  const last = list[list.length - 1];
  if (last && Math.hypot(last.sx - point.sx, last.sy - point.sy) < 0.5) return;
  list.push(point);
  if (list.length > 600) list.splice(0, list.length - 600);
}

function clearTrailsLocal() {
  uavTrailPoints = [];
  carTrailPoints = [];
  lastUavTrailKey = "";
  lastCarTrailKey = "";
  ui.uavTrail.setAttribute("points", "");
  ui.carTrail.setAttribute("points", "");
}

function renderLogs(logs) {
  if (!logs.length) {
    ui.logList.innerHTML = '<div class="empty-log">等待通信数据……</div>';
    return;
  }
  const atBottom = ui.logList.scrollHeight - ui.logList.scrollTop - ui.logList.clientHeight < 36;
  ui.logList.innerHTML = logs.map((item) => {
    const level = String(item.level || "INFO").toLowerCase();
    return `<div class="log-item ${escapeHtml(level)}">
      <span class="log-time">${escapeHtml(item.time || "")}</span>
      <span class="log-source">${escapeHtml(item.source || "")}</span>
      <span class="log-text">${escapeHtml(item.text || "")}</span>
    </div>`;
  }).join("");
  if (atBottom) ui.logList.scrollTop = ui.logList.scrollHeight;
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

ui.taskOptions.forEach((button) => {
  button.addEventListener("click", () => {
    if (taskLocked) {
      showToast("任务已经锁定，请先 RESET 双方后再切换", true);
      return;
    }
    selectedTask = button.dataset.task;
    renderTaskSelector();
  });
});

ui.prepareBtn.addEventListener("click", async () => {
  await api("/api/prepare", { task: selectedTask });
  showToast(`已发送任务 ${selectedTask.slice(1)} 准备命令`);
});
ui.pingBtn.addEventListener("click", async () => {
  await api("/api/ping");
  showToast("已向无人机和小车发送 PING");
});
ui.statusBtn.addEventListener("click", async () => {
  await api("/api/status");
  showToast("已查询双方完整状态");
});
ui.startBtn.addEventListener("click", async () => {
  const data = await api("/api/start");
  showToast(`已向小车发送 START：${data.run_id}`);
});
ui.resetBtn.addEventListener("click", async () => {
  await api("/api/reset");
  clearTrailsLocal();
  showToast("已向无人机和小车发送 RESET");
});
ui.clearTrailBtn.addEventListener("click", async () => {
  clearTrailsLocal();
  await api("/api/clear_trails");
});

async function sendServoCommand(action) {
  if (servoSending) return;
  if (action === "RELEASE") {
    const confirmed = window.confirm("确认执行 A90 释放？已装载的物块会立即落下。此按钮只用于起飞前测试。");
    if (!confirmed) return;
  }
  servoSending = true;
  if (snapshot) renderServoControls(snapshot);
  try {
    const data = await api("/api/servo", { action });
    showToast(action === "LOCK"
      ? `已发送锁止命令 ${data.angle}`
      : `已发送释放命令 ${data.angle}`);
  } finally {
    setTimeout(() => {
      servoSending = false;
      if (snapshot) renderServoControls(snapshot);
    }, 650);
  }
}

ui.servoLockBtn?.addEventListener("click", () => sendServoCommand("LOCK"));
ui.servoReleaseBtn?.addEventListener("click", () => sendServoCommand("RELEASE"));

function setLandSliderProgress(value) {
  const v = Math.max(0, Math.min(100, Number(value) || 0));
  ui.landSlider.style.setProperty("--land-progress", `${v}%`);
  ui.landSliderCard.classList.toggle("armed", v >= 98);
  if (!landSending) {
    ui.landSliderHint.textContent = v >= 98
      ? "已到末端，松手后发送 LAND"
      : v > 0
        ? `继续向右拖动：${Math.round(v)}%`
        : "向右拖动；未到末端松手会自动复位";
  }
}

function resetLandSlider() {
  ui.landSlider.value = "0";
  setLandSliderProgress(0);
}

async function commitLandSlider() {
  if (landSending) return;
  const value = Number(ui.landSlider.value);
  if (value < 98) {
    resetLandSlider();
    return;
  }

  landSending = true;
  ui.landSlider.disabled = true;
  ui.landSliderCard.classList.add("sending");
  ui.landSliderHint.textContent = "正在发送 LAND，等待无人机确认……";
  try {
    await api("/api/land");
    showToast("已发送 LAND：无人机中止任务并在当前位置受控垂直降落", true);
    ui.landSliderHint.textContent = "LAND 已发送；等待无人机落地状态";
  } catch (_) {
    ui.landSliderHint.textContent = "LAND 发送失败，请检查无人机通信";
  } finally {
    setTimeout(() => {
      landSending = false;
      ui.landSlider.disabled = false;
      ui.landSliderCard.classList.remove("sending");
      resetLandSlider();
    }, 900);
  }
}

ui.landSlider.addEventListener("input", () => setLandSliderProgress(ui.landSlider.value));
ui.landSlider.addEventListener("change", commitLandSlider);
ui.landSlider.addEventListener("blur", () => {
  if (!landSending && Number(ui.landSlider.value) < 98) resetLandSlider();
});
setLandSliderProgress(0);

setInterval(() => {
  const now = new Date();
  ui.clockText.textContent = now.toLocaleTimeString("zh-CN", { hour12: false });
  if (socket?.readyState === WebSocket.OPEN) socket.send("ping");
}, 1000);

connectWebSocket();
renderTaskSelector();
