/* ---------------------------------------------------------------
   Rover Dashboard client
   - connects to the ROS2 node's Socket.IO stream
   - updates feeds / gauges / joystick live
   - lets the user rearrange, hide, and tune panels; saved locally
   ------------------------------------------------------------- */

const GAUGE_RANGES = {
  temperature: { min: -10, max: 80 },
  humidity: { min: 0, max: 100 },
  gas_reading: { min: 0, max: 1000 },
};

const STORAGE_KEY = "roverDashboard.settings.v1";

const defaultSettings = {
  order: [
    "live_feed",
    "thermal_feed",
    "temperature",
    "humidity",
    "gas_reading",
    "central_telemetry",
    "joystick_values",
  ],
  hidden: [],
  density: "comfortable",
  feedSizes: {},
  gasThreshold: 400,
  tempThreshold: 45,
};

function loadSettings() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return { ...defaultSettings };
    return { ...defaultSettings, ...JSON.parse(raw) };
  } catch (e) {
    return { ...defaultSettings };
  }
}

function saveSettings(s) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(s));
  } catch (e) {
    /* storage unavailable — dashboard still works, just won't persist */
  }
}

let settings = loadSettings();

/* ---------------- layout: order / visibility / density ---------------- */

const grid = document.getElementById("grid");

function applyOrder() {
  settings.order.forEach((key) => {
    const el = grid.querySelector(`[data-panel="${key}"]`);
    if (el) grid.appendChild(el);
  });
}

function applyVisibility() {
  document.querySelectorAll(".panel").forEach((el) => {
    const key = el.dataset.panel;
    el.dataset.hidden = settings.hidden.includes(key) ? "true" : "false";
  });
}

function applyDensity() {
  const gapMap = { compact: "10px", comfortable: "18px", spacious: "28px" };
  document.documentElement.style.setProperty("--gap", gapMap[settings.density] || "18px");
  document.querySelectorAll("#densitySeg button").forEach((b) => {
    b.classList.toggle("active", b.dataset.density === settings.density);
  });
}

applyOrder();
applyVisibility();
applyDensity();

/* ---- resize the two video panels and persist their layout ---- */
const feedDefaults = { span: 2, height: 320 };

function applyFeedSize(panel, size) {
  panel.style.setProperty("--feed-span", size.span);
  panel.style.setProperty("--feed-height", `${size.height}px`);
}

function addFeedResizer(panel) {
  const key = panel.dataset.panel;
  const body = panel.querySelector(".feed-body");
  const handle = document.createElement("button");
  handle.type = "button";
  handle.className = "feed-resizer";
  handle.title = "Resize video feed";
  handle.setAttribute("aria-label", `Resize ${key}`);
  body.appendChild(handle);

  const saved = settings.feedSizes[key] || { ...feedDefaults };
  applyFeedSize(panel, saved);

  handle.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    event.stopPropagation();
    handle.setPointerCapture(event.pointerId);
    const startX = event.clientX;
    const startY = event.clientY;
    const startWidth = panel.getBoundingClientRect().width;
    const startHeight = panel.getBoundingClientRect().height;
    const startSpan = Number(panel.style.getPropertyValue("--feed-span")) || 2;
    const startFeedHeight = Number.parseInt(panel.style.getPropertyValue("--feed-height"), 10) || 320;
    const columnWidth = Math.max(220, startWidth / startSpan);

    const move = (moveEvent) => {
      const span = Math.max(1, Math.min(4, Math.round((startWidth + moveEvent.clientX - startX) / columnWidth)));
      const height = Math.max(220, Math.min(760, startFeedHeight + moveEvent.clientY - startY));
      applyFeedSize(panel, { span, height });
      panel.dataset.resizing = "true";
    };
    const stop = () => {
      const span = Number(panel.style.getPropertyValue("--feed-span")) || 2;
      const height = Number.parseInt(panel.style.getPropertyValue("--feed-height"), 10) || 320;
      settings.feedSizes[key] = { span, height };
      saveSettings(settings);
      panel.removeAttribute("data-resizing");
      handle.removeEventListener("pointermove", move);
      handle.removeEventListener("pointerup", stop);
      handle.removeEventListener("pointercancel", stop);
    };
    handle.addEventListener("pointermove", move);
    handle.addEventListener("pointerup", stop);
    handle.addEventListener("pointercancel", stop);
  });
}

document.querySelectorAll('.panel[data-panel="live_feed"], .panel[data-panel="thermal_feed"]')
  .forEach(addFeedResizer);

/* ---- drag to reorder (native HTML5 DnD on the panel headers) ---- */
let dragKey = null;

document.querySelectorAll(".panel .panel-head").forEach((head) => {
  head.addEventListener("dragstart", (e) => {
    dragKey = head.closest(".panel").dataset.panel;
    head.closest(".panel").classList.add("dragging");
    e.dataTransfer.effectAllowed = "move";
  });
  head.addEventListener("dragend", () => {
    head.closest(".panel").classList.remove("dragging");
    dragKey = null;
  });
});

grid.addEventListener("dragover", (e) => {
  e.preventDefault();
  const target = e.target.closest(".panel");
  if (!target || !dragKey) return;
  const targetKey = target.dataset.panel;
  if (targetKey === dragKey) return;

  const order = settings.order;
  const from = order.indexOf(dragKey);
  const to = order.indexOf(targetKey);
  if (from === -1 || to === -1) return;
  order.splice(from, 1);
  order.splice(to, 0, dragKey);
  applyOrder();
});

grid.addEventListener("drop", (e) => {
  e.preventDefault();
  saveSettings(settings);
});

/* ---------------- settings overlay wiring ---------------- */

const settingsOverlay = document.getElementById("settingsOverlay");
document.getElementById("settingsBtn").addEventListener("click", () => {
  buildVisibilityToggles();
  document.getElementById("gasThreshold").value = settings.gasThreshold;
  document.getElementById("tempThreshold").value = settings.tempThreshold;
  settingsOverlay.classList.add("open");
});
document.getElementById("closeSettings").addEventListener("click", () => {
  settingsOverlay.classList.remove("open");
});
settingsOverlay.addEventListener("click", (e) => {
  if (e.target === settingsOverlay) settingsOverlay.classList.remove("open");
});

function buildVisibilityToggles() {
  const container = document.getElementById("visibilityToggles");
  container.innerHTML = "";
  document.querySelectorAll(".panel").forEach((panel) => {
    const key = panel.dataset.panel;
    const title = panel.dataset.title || key;
    const row = document.createElement("div");
    row.className = "toggle-row";
    row.innerHTML = `
      <span>${title}</span>
      <label class="switch">
        <input type="checkbox" data-key="${key}" ${settings.hidden.includes(key) ? "" : "checked"} />
        <span class="slider"></span>
      </label>`;
    container.appendChild(row);
  });
  container.querySelectorAll("input[type=checkbox]").forEach((cb) => {
    cb.addEventListener("change", () => {
      const key = cb.dataset.key;
      settings.hidden = settings.hidden.filter((k) => k !== key);
      if (!cb.checked) settings.hidden.push(key);
      applyVisibility();
      saveSettings(settings);
    });
  });
}

document.querySelectorAll("#densitySeg button").forEach((btn) => {
  btn.addEventListener("click", () => {
    settings.density = btn.dataset.density;
    applyDensity();
    saveSettings(settings);
  });
});

document.getElementById("gasThreshold").addEventListener("change", (e) => {
  settings.gasThreshold = parseFloat(e.target.value) || defaultSettings.gasThreshold;
  saveSettings(settings);
});
document.getElementById("tempThreshold").addEventListener("change", (e) => {
  settings.tempThreshold = parseFloat(e.target.value) || defaultSettings.tempThreshold;
  saveSettings(settings);
});

document.getElementById("resetLayoutBtn").addEventListener("click", () => {
  settings = { ...defaultSettings };
  saveSettings(settings);
  applyOrder();
  applyVisibility();
  applyDensity();
  settings.feedSizes = {};
  document.querySelectorAll('.panel[data-panel="live_feed"], .panel[data-panel="thermal_feed"]')
    .forEach((panel) => applyFeedSize(panel, { ...feedDefaults }));
  buildVisibilityToggles();
});

/* ---------------- socket.io live data ---------------- */

const socket = io();
const linkDot = document.getElementById("linkDot");
const linkText = document.getElementById("linkText");

socket.on("connect", () => {
  linkDot.classList.add("connected");
  linkText.textContent = "Live";
});
socket.on("disconnect", () => {
  linkDot.classList.remove("connected");
  linkText.textContent = "Disconnected";
});

function setBadge(key, text, level) {
  const badge = document.querySelector(`[data-badge="${key}"]`);
  if (!badge) return;
  badge.textContent = text;
  badge.className = "badge" + (level ? ` ${level}` : "");
}

/* ---- feeds ---- */
function bindFeed(event, imgId, placeholderId) {
  const img = document.getElementById(imgId);
  socket.on(event, (msg) => {
    img.src = msg.data_uri;
    img.classList.add("has-frame");
    setBadge(event, "live", "live");
  });
  let staleTimer = null;
  socket.on(event, () => {
    clearTimeout(staleTimer);
    staleTimer = setTimeout(() => setBadge(event, "no signal"), 4000);
  });
}
bindFeed("live_feed", "liveFeedImg", "liveFeedPlaceholder");
bindFeed("thermal_feed", "thermalFeedImg", "thermalFeedPlaceholder");

/* ---- gauges ---- */
const GAUGE_ARC_LENGTH = 220; // matches stroke-dasharray in CSS

function updateGauge(key, value) {
  const range = GAUGE_RANGES[key];
  const clamped = Math.max(range.min, Math.min(range.max, value));
  const pct = (clamped - range.min) / (range.max - range.min);
  const fill = document.querySelector(`#gauge-${key} .gauge-fill`);
  if (fill) fill.style.strokeDashoffset = String(GAUGE_ARC_LENGTH * (1 - pct));

  const valEl = document.getElementById(`val-${key}`);
  if (valEl) valEl.textContent = value.toFixed(1);

  let level = null;
  let text = value.toFixed(1);
  if (key === "gas_reading" && value >= settings.gasThreshold) level = "alert";
  if (key === "temperature" && value >= settings.tempThreshold) level = "warn";
  setBadge(key, text, level);
}

socket.on("temperature", (msg) => updateGauge("temperature", msg.value));
socket.on("humidity", (msg) => updateGauge("humidity", msg.value));
socket.on("gas_reading", (msg) => updateGauge("gas_reading", msg.value));

function updateTelemetryValue(key, value, formatter = (item) => item) {
  const element = document.getElementById(`val-${key}`);
  if (element) element.textContent = formatter(value);
}

function updateRadar(sensorKey, value, maxRange = 50) {
  const label = sensorKey === "left" ? "leftRadar" : "rightRadar";
  const valueEl = document.getElementById(`${label}Value`);
  const blip = document.getElementById(`${label}Blip`);
  const display = document.getElementById(`${label}Display`);
  const numericValue = Number.isFinite(value) ? Number(value) : 0;
  const cappedValue = Math.max(0, Math.min(maxRange, numericValue));

  if (valueEl) valueEl.textContent = numericValue > maxRange ? `${maxRange}+` : cappedValue.toFixed(0);
  if (blip) {
    const ratio = cappedValue / maxRange;
    const radius = 72 * ratio;
    const angle = Math.PI / 2;
    const x = 95 + Math.cos(angle) * radius;
    const y = 95 + Math.sin(angle) * radius;
    blip.style.left = `${x}px`;
    blip.style.top = `${y}px`;
    blip.style.opacity = ratio > 0 ? "1" : "0";
    display.classList.toggle("active", cappedValue > 0);
  }
}

socket.on("gas_status", (msg) => {
  const alarm = Number(msg.value) !== 0;
  updateTelemetryValue("gas_status", alarm ? "ALARM" : "CLEAR");
  setBadge("status", alarm ? "gas alarm" : "online", alarm ? "alert" : "live");
});
socket.on("left_distance", (msg) => {
  const rawValue = Number(msg.value) || 0;
  const value = rawValue > 50 ? 50 : rawValue;
  const displayLabel = rawValue > 50 ? "50+" : value.toFixed(1);
  updateTelemetryValue("left_distance", displayLabel);
  updateRadar("left", rawValue, 50);
  setBadge("left_distance", rawValue > 50 ? "50+ cm" : `${value.toFixed(0)} cm`, value < 50 ? "warn" : "live");
});
socket.on("right_distance", (msg) => {
  const rawValue = Number(msg.value) || 0;
  const value = rawValue > 50 ? 50 : rawValue;
  const displayLabel = rawValue > 50 ? "50+" : value.toFixed(1);
  updateTelemetryValue("right_distance", displayLabel);
  updateRadar("right", rawValue, 50);
  setBadge("right_distance", rawValue > 50 ? "50+ cm" : `${value.toFixed(0)} cm`, value < 50 ? "warn" : "live");
});
socket.on("speed_mode", (msg) => updateTelemetryValue("speed_mode", msg.value));
socket.on("motor_left", (msg) => updateTelemetryValue("motor_left", msg.value));
socket.on("motor_right", (msg) => updateTelemetryValue("motor_right", msg.value));
socket.on("status", (msg) => {
  updateTelemetryValue("status", msg.value);
  setBadge("status", "online", "live");
});

/* ---- joystick ---- */
const joyCanvas = document.getElementById("joystickCanvas");
const joyCtx = joyCanvas.getContext("2d");
const joyXEl = document.getElementById("joyX");
const joyYEl = document.getElementById("joyY");

function drawJoystick(x, y) {
  const w = joyCanvas.width;
  const h = joyCanvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const r = Math.min(w, h) / 2 - 14;

  joyCtx.clearRect(0, 0, w, h);

  // crosshair
  joyCtx.strokeStyle = "rgba(255,255,255,0.12)";
  joyCtx.lineWidth = 1;
  joyCtx.beginPath();
  joyCtx.moveTo(cx, cy - r);
  joyCtx.lineTo(cx, cy + r);
  joyCtx.moveTo(cx - r, cy);
  joyCtx.lineTo(cx + r, cy);
  joyCtx.stroke();

  joyCtx.beginPath();
  joyCtx.arc(cx, cy, r, 0, Math.PI * 2);
  joyCtx.strokeStyle = "rgba(255,255,255,0.18)";
  joyCtx.stroke();

  // stick position (y inverted: forward = up)
  const px = cx + x * r;
  const py = cy - y * r;

  joyCtx.beginPath();
  joyCtx.arc(px, py, 14, 0, Math.PI * 2);
  const grad = joyCtx.createRadialGradient(px, py, 2, px, py, 14);
  grad.addColorStop(0, "#bdfcff");
  grad.addColorStop(1, "#4fd8e0");
  joyCtx.fillStyle = grad;
  joyCtx.shadowColor = "rgba(79,216,224,0.7)";
  joyCtx.shadowBlur = 16;
  joyCtx.fill();
}
drawJoystick(0, 0);

socket.on("joystick_values", (msg) => {
  const x = msg.axes && msg.axes.length > 0 ? msg.axes[0] : 0;
  const y = msg.axes && msg.axes.length > 1 ? msg.axes[1] : 0;
  drawJoystick(x, y);
  joyXEl.textContent = x.toFixed(2);
  joyYEl.textContent = y.toFixed(2);
  setBadge("joystick_values", "active", "live");
});
