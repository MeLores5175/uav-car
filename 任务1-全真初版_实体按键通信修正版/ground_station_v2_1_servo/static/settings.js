"use strict";

const ui = {
  form: document.getElementById("settingsForm"),
  uavIp: document.getElementById("uavIp"),
  carIp: document.getElementById("carIp"),
  uavDefault: document.getElementById("uavDefault"),
  carDefault: document.getElementById("carDefault"),
  uavPort: document.getElementById("uavPort"),
  carPort: document.getElementById("carPort"),
  gsListen: document.getElementById("gsListen"),
  missionStatus: document.getElementById("missionStatus"),
  configFile: document.getElementById("configFile"),
  restoreDefaultsBtn: document.getElementById("restoreDefaultsBtn"),
  reloadBtn: document.getElementById("reloadBtn"),
  saveBtn: document.getElementById("saveBtn"),
  saveBadge: document.getElementById("saveBadge"),
  resultBox: document.getElementById("resultBox"),
  busyWarning: document.getElementById("busyWarning"),
};

let settings = null;

function setResult(message, isError = false) {
  ui.resultBox.textContent = message;
  ui.resultBox.classList.toggle("error", isError);
  ui.resultBox.classList.remove("hidden");
}

function clearResult() {
  ui.resultBox.classList.add("hidden");
  ui.resultBox.classList.remove("error");
  ui.resultBox.textContent = "";
}

function isIPv4(value) {
  const parts = String(value).trim().split(".");
  if (parts.length !== 4) return false;
  return parts.every((part) => {
    if (!/^\d{1,3}$/.test(part)) return false;
    if (part.length > 1 && part.startsWith("0")) return Number(part) === 0;
    const number = Number(part);
    return number >= 0 && number <= 255;
  });
}

function validateInputs() {
  let ok = true;
  for (const input of [ui.uavIp, ui.carIp]) {
    const valid = isIPv4(input.value);
    input.classList.toggle("invalid", !valid);
    ok = ok && valid;
  }
  return ok;
}

function applyEditableState(canEdit) {
  ui.uavIp.disabled = !canEdit;
  ui.carIp.disabled = !canEdit;
  ui.restoreDefaultsBtn.disabled = !canEdit;
  ui.saveBtn.disabled = !canEdit;
  ui.busyWarning.classList.toggle("hidden", canEdit);
  ui.saveBadge.textContent = canEdit ? "允许修改" : "任务运行中";
  ui.saveBadge.className = `status-badge ${canEdit ? "online" : "offline"}`;
}

function render(data) {
  settings = data;
  const uav = data.devices?.uav || {};
  const car = data.devices?.car || {};
  const network = data.network || {};

  ui.uavIp.value = uav.ip || "";
  ui.carIp.value = car.ip || "";
  ui.uavDefault.textContent = uav.default_ip || "--";
  ui.carDefault.textContent = car.default_ip || "--";
  ui.uavPort.textContent = uav.port ?? "--";
  ui.carPort.textContent = car.port ?? "--";
  ui.gsListen.textContent = `${network.bind_ip ?? "--"}:${network.gs_port ?? "--"}`;
  ui.missionStatus.textContent = data.mission_status || "--";
  ui.configFile.textContent = data.config_file || "--";
  applyEditableState(Boolean(data.can_edit));
  validateInputs();
}

async function loadSettings() {
  clearResult();
  ui.reloadBtn.disabled = true;
  try {
    const response = await fetch("/api/settings", { cache: "no-store" });
    const data = await response.json();
    if (!response.ok || data.ok === false) {
      throw new Error(data.error || `读取失败：HTTP ${response.status}`);
    }
    render(data);
  } catch (error) {
    setResult(error.message || String(error), true);
  } finally {
    ui.reloadBtn.disabled = false;
  }
}

async function saveSettings(event) {
  event.preventDefault();
  clearResult();

  if (!validateInputs()) {
    setResult("请检查无人机和小车的 IPv4 地址格式。", true);
    return;
  }

  ui.saveBtn.disabled = true;
  ui.saveBadge.textContent = "正在保存";
  ui.saveBadge.className = "status-badge offline";

  try {
    const response = await fetch("/api/settings", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        uav_ip: ui.uavIp.value.trim(),
        car_ip: ui.carIp.value.trim(),
      }),
    });
    const data = await response.json().catch(() => ({}));
    if (!response.ok || data.ok === false) {
      throw new Error(data.error || `保存失败：HTTP ${response.status}`);
    }
    render(data);
    setResult(data.message || "保存成功，新的 IP 已生效。");
  } catch (error) {
    setResult(error.message || String(error), true);
    ui.saveBtn.disabled = false;
    ui.saveBadge.textContent = "保存失败";
    ui.saveBadge.className = "status-badge offline";
  }
}

ui.form.addEventListener("submit", saveSettings);
ui.reloadBtn.addEventListener("click", loadSettings);

ui.restoreDefaultsBtn.addEventListener("click", () => {
  if (!settings) return;
  ui.uavIp.value = settings.devices?.uav?.default_ip || "";
  ui.carIp.value = settings.devices?.car?.default_ip || "";
  validateInputs();
  setResult("默认值已填入输入框；点击“保存并立即应用”后才会生效。");
});

ui.uavIp.addEventListener("input", validateInputs);
ui.carIp.addEventListener("input", validateInputs);

loadSettings();
