"use strict";

const ui = {
  wsBadge: document.getElementById("wsBadge"),
  clockText: document.getElementById("clockText"),
  taskOptions: [...document.querySelectorAll(".debug-task-option")],
  uavPingBtn: document.getElementById("uavPingBtn"),
  uavStatusBtn: document.getElementById("uavStatusBtn"),
  uavBootBtn: document.getElementById("uavBootBtn"),
  uavResetBtn: document.getElementById("uavResetBtn"),
  directStartBtn: document.getElementById("directStartBtn"),
  directStartHint: document.getElementById("directStartHint"),
  landSlider: document.getElementById("landSlider"),
  landSliderCard: document.getElementById("landSliderCard"),
  landSliderHint: document.getElementById("landSliderHint"),
  clearTrailBtn: document.getElementById("clearTrailBtn"),
  uavMarker: document.getElementById("uavMarker"),
  uavTrail: document.getElementById("uavTrail"),
  carMarker: document.getElementById("carMarker"),
  carTrail: document.getElementById("carTrail"),
  logList: document.getElementById("logList"),
  toast: document.getElementById("toast"),
};

let selectedTask = "T1";
let socket = null;
let reconnectTimer = null;
let toastTimer = null;
let uavTrailPoints = [];
let carTrailPoints = [];
let lastUavTrailKey = "";
let lastCarTrailKey = "";
let landSending = false;
let startHoldTimer = null;
let startHoldTick = null;
let startHoldStartedAt = 0;
let startSending = false;
let lastDebugCommand = "--";

const stateLabels = {
  IDLE: "空闲", STOPPED: "节点已停止", STARTING: "启动中", READY: "准备完成",
  FAILED: "启动失败", WAIT_START: "等待启动", WAIT_FCU: "等待飞控准备",
  TAKEOFF: "起飞中", HOVER: "定高悬停", SEARCH_CAR: "搜索小车", FOLLOW: "伴飞中",
  FOLLOW_CAR: "伴飞中", PREPARE_DROP: "准备抛投", DROPPING: "正在抛投",
  DROP_DONE: "抛投完成", RETURN_HOME: "返回 H 点", LAND_HOME: "H 点降落",
  APPROACH_CAR: "接近小车", LAND_ON_CAR: "动态降落", ON_CAR: "已降落小车",
  TAKEOFF_FROM_CAR: "从小车起飞", DONE: "任务完成", NORMAL: "正常",
  WARNING: "警告", ABORTING: "安全中止", ABORTED: "任务已中止",
  LANDING: "受控降落", LANDED: "已落地", FAULT: "故障",
};

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

function label(value, fallback = "--") {
  if (value === undefined || value === null || value === "") return fallback;
  const key = String(value).toUpperCase();
  return stateLabels[key] || String(value);
}

function showToast(message, isError = false) {
  clearTimeout(toastTimer);
  ui.toast.textContent = message;
  ui.toast.classList.toggle("error", isError);
  ui.toast.classList.remove("hidden");
  toastTimer = setTimeout(() => ui.toast.classList.add("hidden"), 2800);
}

async function post(path, body = {}) {
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

async function sendUavCommand(action) {
  const body = { action };
  if (action === "BOOT" || action === "START") body.task = selectedTask;
  const data = await post("/api/debug/uav_command", body);
  lastDebugCommand = `CMD:${data.cmd_id}:${action}`;
  if (action === "START" && data.run_id) lastDebugCommand += `:${data.run_id}:${selectedTask}`;
  else if (action === "LAND" && data.run_id) lastDebugCommand += `:${data.run_id}`;
  else if (action === "BOOT") lastDebugCommand += `:${selectedTask}`;
  setText("lastDebugCommand", lastDebugCommand);
  if (data.run_id) setText("debugRunId", data.run_id);
  return data;
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
      if (message.type === "snapshot") render(message.data);
      else if (message.type === "clear_trails") clearTrail();
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
  const uavData = data.uav || {};
  const tel = uavData.telemetry || {};
  const link = uavData.link || { state: "OFFLINE", age_s: null };
  const linkState = String(link.state || "OFFLINE").toLowerCase();
  const linkLabel = linkState === "online" ? "在线" : linkState === "delayed" ? "延迟" : "离线";

  const dot = document.getElementById("uavLinkDot");
  dot.className = `link-dot ${linkState}`;
  setText("uavLinkText", link.age_s == null ? linkLabel : `${linkLabel} · ${link.age_s}s`);
  setText("uavLinkMap", link.age_s == null ? linkLabel : `${linkLabel} · ${link.age_s}s`);

  setText("uavStateText", label(tel.state || uavData.heartbeat?.state, "未知"));
  setText("uavBoot", label(tel.boot, "--"));
  setText("uavSafety", label(tel.safety, "--"));
  setText("uavAltitude", fmt(tel.z_cm, 1, " cm"));
  setText("uavAltitudeMap", fmt(tel.z_cm, 1, " cm"));
  setText("uavYaw", fmt(tel.yaw_deg, 1, "°"));
  setText("uavYawMap", fmt(tel.yaw_deg, 1, "°"));
  setText("uavMode", tel.mode || "--");
  setText("uavArmed", boolText(tel.armed));
  setText("uavFcu", boolText(tel.fcu));
  setText("uavBattery", fmt(tel.battery, 0, "%"));
  setText("missionResult", tel.mission_result || "--");
  setText("abortReason", tel.abort_reason || "--");
  setText("trackingError", fmt(tel.tracking_error_cm, 1, " cm"));
  setText("uavFsmDetail", tel.fsm_state || "--");
  setText("debugRunId", data.mission?.run_id || "--");
  setText("pendingCommands", String((data.pending || []).filter((x) => x.device === "uav").length));

  const armed = tel.armed === true;
  ui.uavResetBtn.disabled = armed;
  ui.uavBootBtn.disabled = armed;

  const net = data.network || {};
  const uav = net.uav || {};
  setText("networkSummary", `GS UDP ${net.gs_port ?? "--"} · UAV ${uav.ip ?? "--"}:${uav.port ?? "--"}`);
  renderMap(tel, data.car?.telemetry || {});
  renderLogs(data.logs || []);
}

function fieldToLandscape(xCm, yCm) {
  const x = numberValue(xCm);
  const y = numberValue(yCm);
  if (x === null || y === null) return null;
  return { sx: Math.max(0, Math.min(500, y)), sy: Math.max(0, Math.min(400, x)) };
}

function updateMarker(element, point, yawDeg) {
  if (!point) {
    element.classList.add("hidden");
    return;
  }
  element.classList.remove("hidden");
  const yaw = numberValue(yawDeg) ?? 0;
  const angle = 90 - yaw;
  element.setAttribute("transform", `translate(${point.sx} ${point.sy}) rotate(${angle})`);
  const text = element.querySelector(".marker-label");
  if (text) text.setAttribute("transform", `rotate(${-angle})`);
}

function appendTrail(list, point, seq, device) {
  const key = `${seq ?? ""}:${point.sx.toFixed(2)}:${point.sy.toFixed(2)}`;
  if (device === "uav") {
    if (key === lastUavTrailKey) return;
    lastUavTrailKey = key;
  } else {
    if (key === lastCarTrailKey) return;
    lastCarTrailKey = key;
  }
  const last = list[list.length - 1];
  if (!last || Math.hypot(last.sx - point.sx, last.sy - point.sy) >= 0.5) {
    list.push(point);
    if (list.length > 600) list.splice(0, list.length - 600);
  }
}

function renderMap(tel, car) {
  const uavPoint = fieldToLandscape(tel.x_cm, tel.y_cm);
  const carPoint = fieldToLandscape(car.x_cm, car.y_cm);
  updateMarker(ui.uavMarker, uavPoint, tel.yaw_deg);
  updateMarker(ui.carMarker, carPoint, car.yaw_deg);
  if (uavPoint) appendTrail(uavTrailPoints, uavPoint, tel.seq, "uav");
  if (carPoint) appendTrail(carTrailPoints, carPoint, car.seq, "car");
  ui.uavTrail.setAttribute("points", uavTrailPoints.map((p) => `${p.sx},${p.sy}`).join(" "));
  ui.carTrail.setAttribute("points", carTrailPoints.map((p) => `${p.sx},${p.sy}`).join(" "));

  const x = numberValue(tel.x_cm), y = numberValue(tel.y_cm), z = numberValue(tel.z_cm);
  const cx = numberValue(car.x_cm), cy = numberValue(car.y_cm);
  setText("uavPosition", x === null || y === null ? "--" : `X ${x.toFixed(1)} / Y ${y.toFixed(1)} / Z ${z?.toFixed(1) ?? "--"} cm`);
  setText("carPosition", cx === null || cy === null ? "--" : `X ${cx.toFixed(1)} / Y ${cy.toFixed(1)} cm`);
  setText("relativeDistance", x === null || y === null || cx === null || cy === null ? "--" : `${Math.hypot(x-cx, y-cy).toFixed(1)} cm`);
}

function clearTrail() {
  uavTrailPoints = [];
  carTrailPoints = [];
  lastUavTrailKey = "";
  lastCarTrailKey = "";
  ui.uavTrail.setAttribute("points", "");
  ui.carTrail.setAttribute("points", "");
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;").replaceAll("<", "&lt;").replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;").replaceAll("'", "&#039;");
}

function renderLogs(logs) {
  const filtered = logs.filter((item) => {
    const source = String(item.source || "").toUpperCase();
    const text = String(item.text || "").toUpperCase();
    return source === "UAV" || source === "CAR" || source === "GS" || source === "DEBUG" || text.includes("UAV") || text.includes("无人机") || text.includes("假小车");
  });
  if (!filtered.length) {
    ui.logList.innerHTML = '<div class="empty-log">等待无人机通信数据……</div>';
    return;
  }
  const atBottom = ui.logList.scrollHeight - ui.logList.scrollTop - ui.logList.clientHeight < 36;
  ui.logList.innerHTML = filtered.slice(-100).map((item) => {
    const level = String(item.level || "INFO").toLowerCase();
    return `<div class="log-item ${escapeHtml(level)}">
      <span class="log-time">${escapeHtml(item.time || "")}</span>
      <span class="log-source">${escapeHtml(item.source || "")}</span>
      <span class="log-text">${escapeHtml(item.text || "")}</span>
    </div>`;
  }).join("");
  if (atBottom) ui.logList.scrollTop = ui.logList.scrollHeight;
}

function renderTaskOptions() {
  ui.taskOptions.forEach((button) => {
    button.classList.toggle("active", button.dataset.task === selectedTask);
  });
  ui.directStartHint.textContent = `当前选择任务 ${selectedTask.slice(1)}；松开过早不会发送`;
}

ui.taskOptions.forEach((button) => {
  button.addEventListener("click", () => {
    selectedTask = button.dataset.task;
    renderTaskOptions();
  });
});

ui.uavPingBtn.addEventListener("click", async () => {
  await sendUavCommand("PING");
  showToast("已单独向无人机发送 PING");
});
ui.uavStatusBtn.addEventListener("click", async () => {
  await sendUavCommand("STATUS");
  showToast("已单独向无人机查询 STATUS");
});
ui.uavBootBtn.addEventListener("click", async () => {
  await sendUavCommand("BOOT");
  showToast(`已向无人机发送 BOOT:${selectedTask}`);
});
ui.uavResetBtn.addEventListener("click", async () => {
  await sendUavCommand("RESET");
  clearTrail();
  showToast("已单独向无人机发送 RESET");
});
ui.clearTrailBtn.addEventListener("click", async () => {
  clearTrail();
  await post("/api/clear_trails");
});

function cancelStartHold() {
  clearTimeout(startHoldTimer);
  clearInterval(startHoldTick);
  startHoldTimer = null;
  startHoldTick = null;
  startHoldStartedAt = 0;
  ui.directStartBtn.classList.remove("holding");
  ui.directStartBtn.style.setProperty("--hold-progress", "0%");
  if (!startSending) renderTaskOptions();
}

function beginStartHold(event) {
  if (startSending || ui.directStartBtn.disabled) return;
  if (event.type === "pointerdown") ui.directStartBtn.setPointerCapture?.(event.pointerId);
  cancelStartHold();
  startHoldStartedAt = performance.now();
  ui.directStartBtn.classList.add("holding");
  ui.directStartHint.textContent = "保持按住，正在确认直接起飞……";
  startHoldTick = setInterval(() => {
    const progress = Math.min(100, ((performance.now() - startHoldStartedAt) / 1000) * 100);
    ui.directStartBtn.style.setProperty("--hold-progress", `${progress}%`);
  }, 25);
  startHoldTimer = setTimeout(async () => {
    clearInterval(startHoldTick);
    startHoldTimer = null;
    startHoldTick = null;
    ui.directStartBtn.style.setProperty("--hold-progress", "100%");
    startSending = true;
    ui.directStartBtn.disabled = true;
    ui.directStartHint.textContent = "正在发送无人机直接 START……";
    try {
      const data = await sendUavCommand("START");
      showToast(`已直接向无人机发送 START：${data.run_id}`, true);
      ui.directStartHint.textContent = `已发送 ${data.run_id} / ${selectedTask}；等待 UAV ACK`;
    } finally {
      setTimeout(() => {
        startSending = false;
        ui.directStartBtn.disabled = false;
        cancelStartHold();
      }, 1100);
    }
  }, 1000);
}

ui.directStartBtn.addEventListener("pointerdown", beginStartHold);
["pointerup", "pointercancel", "pointerleave"].forEach((name) => {
  ui.directStartBtn.addEventListener(name, () => {
    if (!startSending && startHoldTimer) cancelStartHold();
  });
});
ui.directStartBtn.addEventListener("keydown", (event) => {
  if ((event.key === " " || event.key === "Enter") && !event.repeat) {
    event.preventDefault();
    beginStartHold(event);
  }
});
ui.directStartBtn.addEventListener("keyup", (event) => {
  if (event.key === " " || event.key === "Enter") {
    event.preventDefault();
    if (!startSending && startHoldTimer) cancelStartHold();
  }
});

function setLandProgress(value) {
  const v = Math.max(0, Math.min(100, Number(value) || 0));
  ui.landSlider.style.setProperty("--land-progress", `${v}%`);
  ui.landSliderCard.classList.toggle("armed", v >= 98);
  if (!landSending) {
    ui.landSliderHint.textContent = v >= 98 ? "已到末端，松手后发送 LAND" : v > 0 ? `继续向右拖动：${Math.round(v)}%` : "向右拖动；未到末端松手会自动复位";
  }
}

function resetLand() {
  ui.landSlider.value = "0";
  setLandProgress(0);
}

async function commitLand() {
  if (landSending) return;
  if (Number(ui.landSlider.value) < 98) {
    resetLand();
    return;
  }
  landSending = true;
  ui.landSlider.disabled = true;
  ui.landSliderCard.classList.add("sending");
  ui.landSliderHint.textContent = "正在直接向无人机发送 LAND……";
  try {
    const data = await sendUavCommand("LAND");
    showToast(`已发送安全 LAND：${data.run_id || "R000"}`, true);
    ui.landSliderHint.textContent = "LAND 已发送；等待无人机落地状态";
  } finally {
    setTimeout(() => {
      landSending = false;
      ui.landSlider.disabled = false;
      ui.landSliderCard.classList.remove("sending");
      resetLand();
    }, 900);
  }
}

ui.landSlider.addEventListener("input", () => setLandProgress(ui.landSlider.value));
ui.landSlider.addEventListener("change", commitLand);
ui.landSlider.addEventListener("blur", () => {
  if (!landSending && Number(ui.landSlider.value) < 98) resetLand();
});

setLandProgress(0);
renderTaskOptions();
setInterval(() => {
  ui.clockText.textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false });
  if (socket?.readyState === WebSocket.OPEN) socket.send("ping");
}, 1000);
connectWebSocket();
