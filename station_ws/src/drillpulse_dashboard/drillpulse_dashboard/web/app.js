// =========================================================
// DRILLPULSE DUAL-PANEL DASHBOARD CONTROLLER
// Handles WebSocket stream, dual-panel locking, map rendering,
// telemetry HUD updates, keyboard recovery shortcuts, and E-Stop.
// =========================================================

const socket = io();

// State tracking
let currentState = null;
let exploredMapImg = null;
let exploredMapMeta = null;

// DOM Elements
const linkOrb = document.getElementById("linkOrb");
const fsmState = document.getElementById("fsmState");
const batteryPct = document.getElementById("batteryPct");
const roverSpeed = document.getElementById("roverSpeed");
const loraRssi = document.getElementById("loraRssi");
const loraSnr = document.getElementById("loraSnr");
const wifiStatus = document.getElementById("wifiStatus");

const valCh4 = document.getElementById("valCh4");
const barCh4 = document.getElementById("barCh4");
const cardCh4 = document.getElementById("cardCh4");

const valCo = document.getElementById("valCo");
const barCo = document.getElementById("barCo");
const cardCo = document.getElementById("cardCo");

const valO2 = document.getElementById("valO2");
const valTemp = document.getElementById("valTemp");
const valHum = document.getElementById("valHum");
const valAttitude = document.getElementById("valAttitude");
const tiltWarning = document.getElementById("tiltWarning");
const cardAttitude = document.getElementById("cardAttitude");

const valOdomState = document.getElementById("valOdomState");
const valOdomErr = document.getElementById("valOdomErr");

const panelExplored = document.getElementById("panelExplored");
const panelUnexplored = document.getElementById("panelUnexplored");
const ackExplored = document.getElementById("ackExplored");
const ackUnexplored = document.getElementById("ackUnexplored");
const poseExplored = document.getElementById("poseExplored");
const poseUnexplored = document.getElementById("poseUnexplored");
const frontiersCount = document.getElementById("frontiersCount");

const recStatusText = document.getElementById("recStatusText");
const liveFeedImg = document.getElementById("liveFeedImg");
const thermalFeedImg = document.getElementById("thermalFeedImg");
const hazardList = document.getElementById("hazardList");

const canvasExp = document.getElementById("canvasExplored");
const ctxExp = canvasExp.getContext("2d");
const canvasUnexp = document.getElementById("canvasUnexplored");
const ctxUnexp = canvasUnexp.getContext("2d");

// Socket Events
socket.on("connect", () => {
  console.log("Connected to DrillPulse Dashboard Server");
  linkOrb.classList.add("connected");
  fetchExploredMap();
});

socket.on("disconnect", () => {
  console.warn("Disconnected from DrillPulse Server");
  linkOrb.classList.remove("connected");
});

socket.on("dashboard_state", (state) => {
  currentState = state;
  updateHUD(state);
  updatePanelActivation(state);
  drawExploredCanvas(state);
  drawUnexploredCanvas(state);
});

socket.on("live_frame", (msg) => {
  if (liveFeedImg) liveFeedImg.src = msg.data;
});

socket.on("thermal_frame", (msg) => {
  if (thermalFeedImg) thermalFeedImg.src = msg.data;
});

socket.on("hazard_alert", (hazard) => {
  appendHazardItem(hazard);
});

socket.on("recovery_status", (msg) => {
  if (recStatusText) recStatusText.innerText = msg.status;
});

// Update Telemetry & HUD
function updateHUD(state) {
  if (!state) return;
  const t = state.telemetry || {};
  const d = state.diagnostics || {};
  const l = state.link || {};

  fsmState.innerText = state.rover_state || "IDLE";
  batteryPct.innerText = `${t.battery || 0}%`;
  roverSpeed.innerText = `${(t.speed || 0).toFixed(2)} m/s`;

  loraRssi.innerText = `${l.lora_rssi || -65} dBm`;
  loraSnr.innerText = `SNR ${l.lora_snr || 0} dB`;
  wifiStatus.innerText = l.wifi_connected ? "ONLINE" : "OFFLINE";

  // Gas Telemetry
  valCh4.innerText = `${(t.ch4 || 0).toFixed(2)} ppm`;
  barCh4.style.width = `${Math.min(100, (t.ch4 || 0) * 10)}%`;
  if (t.ch4 > 10.0) {
    cardCh4.classList.add("danger");
  } else {
    cardCh4.classList.remove("danger");
  }

  valCo.innerText = `${(t.co || 0).toFixed(1)} ppm`;
  barCo.style.width = `${Math.min(100, (t.co || 0) * 2)}%`;
  if (t.co > 50.0) {
    cardCo.classList.add("danger");
  } else {
    cardCo.classList.remove("danger");
  }

  valO2.innerText = `${(t.o2 || 20.9).toFixed(1)} %`;
  valTemp.innerText = `${(t.temp || 24.0).toFixed(1)} °C`;
  valHum.innerText = `Humidity: ${t.humidity || 60}%`;

  valAttitude.innerText = `P: ${(t.pitch || 0).toFixed(1)}° | R: ${(t.roll || 0).toFixed(1)}°`;
  const maxTilt = Math.max(Math.abs(t.pitch || 0), Math.abs(t.roll || 0));
  if (maxTilt > 35.0) {
    cardAttitude.classList.add("danger");
    tiltWarning.innerText = "CRITICAL ROLLOVER RISK!";
  } else {
    cardAttitude.classList.remove("danger");
    tiltWarning.innerText = "Normal";
  }

  // Diagnostics
  valOdomState.innerText = `${d.fusion_state || "CONSISTENT"} (${d.source || "FUSED"})`;
  valOdomErr.innerText = `v_err: ${(d.vel_err || 0).toFixed(2)} | w_err: ${(d.yaw_err || 0).toFixed(2)}`;

  frontiersCount.innerText = state.frontiers_count || 0;
  if (recStatusText && state.recovery_status) {
    recStatusText.innerText = state.recovery_status;
  }
}

// Strict Dual-Panel Locking Logic
// Active panel turns GREEN ONLY upon Rover ACK
function updatePanelActivation(state) {
  const isExpAck = state.rover_ack && state.active_panel === "EXPLORED";
  const isUnexpAck = state.rover_ack && state.active_panel === "UNEXPLORED";

  const t = state.telemetry || {};
  const poseStr = `(${t.x ? t.x.toFixed(2) : "0.00"}, ${t.y ? t.y.toFixed(2) : "0.00"}, ${t.yaw ? (t.yaw * 180 / Math.PI).toFixed(1) : "0.0"}°)`;

  if (isExpAck) {
    panelExplored.classList.add("active");
    panelUnexplored.classList.remove("active");
    ackExplored.innerText = "ACTIVE (ROVER CONFIRMED)";
    ackUnexplored.innerText = "STANDBY / INACTIVE";
    poseExplored.innerText = poseStr;
  } else if (isUnexpAck) {
    panelUnexplored.classList.add("active");
    panelExplored.classList.remove("active");
    ackUnexplored.innerText = "ACTIVE (ROVER CONFIRMED)";
    ackExplored.innerText = "STANDBY / INACTIVE";
    poseUnexplored.innerText = poseStr;
  } else {
    // Standby: neither active
    panelExplored.classList.remove("active");
    panelUnexplored.classList.remove("active");
    ackExplored.innerText = "AWAITING ACK";
    ackUnexplored.innerText = "AWAITING ACK";
  }
}

// Canvas Drawing for Explored Panel
function drawExploredCanvas(state) {
  ctxExp.clearRect(0, 0, canvasExp.width, canvasExp.height);

  if (exploredMapImg && exploredMapImg.complete) {
    ctxExp.drawImage(exploredMapImg, 0, 0, canvasExp.width, canvasExp.height);
  } else {
    // Placeholder grid
    drawGrid(ctxExp, canvasExp.width, canvasExp.height);
  }

  // Draw Nav2 Plan
  if (state && state.plan && state.plan.length > 1) {
    ctxExp.strokeStyle = "#38bdf8";
    ctxExp.lineWidth = 2;
    ctxExp.beginPath();
    state.plan.forEach((pt, idx) => {
      const [cx, cy] = worldToCanvas(pt[0], pt[1], canvasExp.width, canvasExp.height);
      if (idx === 0) ctxExp.moveTo(cx, cy);
      else ctxExp.lineTo(cx, cy);
    });
    ctxExp.stroke();
  }

  // Draw Rover Position
  const t = (state && state.telemetry) ? state.telemetry : { x: 0, y: 0, yaw: 0 };
  const [rx, ry] = worldToCanvas(t.x || 0, t.y || 0, canvasExp.width, canvasExp.height);
  drawRover(ctxExp, rx, ry, t.yaw || 0, "#22c55e");
}

// Canvas Drawing for Unexplored Panel
function drawUnexploredCanvas(state) {
  ctxUnexp.clearRect(0, 0, canvasUnexp.width, canvasUnexp.height);
  drawGrid(ctxUnexp, canvasUnexp.width, canvasUnexp.height);

  // Draw Discovered Frontier Clusters Simulation/Grid
  const count = (state && state.frontiers_count) ? state.frontiers_count : 0;
  ctxUnexp.fillStyle = "#f59e0b";
  ctxUnexp.font = "12px monospace";
  ctxUnexp.fillText(`Active Frontier Clusters: ${count}`, 15, 25);

  const t = (state && state.telemetry) ? state.telemetry : { x: 0, y: 0, yaw: 0 };
  const [rx, ry] = worldToCanvas(t.x || 0, t.y || 0, canvasUnexp.width, canvasUnexp.height);
  drawRover(ctxUnexp, rx, ry, t.yaw || 0, "#38bdf8");
}

function drawGrid(ctx, w, h) {
  ctx.strokeStyle = "#1e293b";
  ctx.lineWidth = 1;
  const step = 40;
  for (let x = 0; x < w; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
}

function worldToCanvas(wx, wy, cw, ch) {
  // Simple center-offset projection: 1 meter = 40 pixels
  const scale = 35.0;
  const cx = cw / 2 + wx * scale;
  const cy = ch / 2 - wy * scale;
  return [cx, cy];
}

function drawRover(ctx, x, y, yaw, color) {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);

  // Rover body
  ctx.fillStyle = color;
  ctx.fillRect(-12, -8, 24, 16);

  // Direction pointer
  ctx.strokeStyle = "#fff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(16, 0);
  ctx.stroke();

  ctx.restore();
}

function fetchExploredMap() {
  fetch("/api/map")
    .then((res) => res.json())
    .then((data) => {
      if (data && data.image_data) {
        exploredMapImg = new Image();
        exploredMapImg.src = data.image_data;
        exploredMapMeta = data;
        exploredMapImg.onload = () => {
          if (currentState) drawExploredCanvas(currentState);
        };
      }
    })
    .catch((err) => console.log("Explored map not yet loaded:", err));
}

function appendHazardItem(h) {
  const empty = hazardList.querySelector(".empty-hint");
  if (empty) empty.remove();

  const li = document.createElement("li");
  li.className = `hazard-item ${h.severity ? h.severity.toLowerCase() : ""}`;
  li.innerHTML = `<strong>[${h.time || "LOG"}] ${h.type || "HAZARD"}</strong>: ${h.desc || ""} (Observed: ${h.value || 0})`;
  hazardList.prepend(li);
}

// User Actions & Button Event Handlers
document.getElementById("btnActivateExplored").addEventListener("click", () => {
  socket.emit("request_mode", { mode: "EXPLORED" });
});

document.getElementById("btnActivateUnexplored").addEventListener("click", () => {
  socket.emit("request_mode", { mode: "UNEXPLORED" });
});

document.getElementById("btnSendWaypoint").addEventListener("click", () => {
  const x = parseFloat(document.getElementById("wpX").value) || 0.0;
  const y = parseFloat(document.getElementById("wpY").value) || 0.0;
  const yaw = parseFloat(document.getElementById("wpYaw").value) || 0.0;
  socket.emit("send_waypoint", { x, y, yaw });
});

document.getElementById("btnEstop").addEventListener("click", () => {
  if (confirm("CONFIRM EMERGENCY STOP? Rover motion will halt immediately.")) {
    socket.emit("emergency_stop");
  }
});

document.getElementById("btnRecSwivel").addEventListener("click", () => {
  socket.emit("trigger_recovery", { action: "SWIVEL_45" });
});

document.getElementById("btnRecExtend").addEventListener("click", () => {
  socket.emit("trigger_recovery", { action: "EXTEND_ACTUATORS" });
});

document.getElementById("btnGenReport").addEventListener("click", () => {
  socket.emit("generate_report");
  fetch("/api/reports")
    .then((r) => r.json())
    .then((reports) => {
      if (reports.length > 0) {
        window.open(reports[0].url, "_blank");
      } else {
        alert("Report generation dispatched. Please wait a moment.");
      }
    });
});

// Keyboard Shortcuts: Key 6, Key 7, E-Stop
window.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT") return;

  if (e.key === "6") {
    console.log("Key 6 pressed: Triggering 45° Swivel Recovery");
    socket.emit("trigger_recovery", { action: "SWIVEL_45" });
  } else if (e.key === "7") {
    console.log("Key 7 pressed: Triggering Telescoping Extension Recovery");
    socket.emit("trigger_recovery", { action: "EXTEND_ACTUATORS" });
  } else if (e.key === "Escape") {
    console.warn("ESC pressed: Emergency Stop");
    socket.emit("emergency_stop");
  }
});
