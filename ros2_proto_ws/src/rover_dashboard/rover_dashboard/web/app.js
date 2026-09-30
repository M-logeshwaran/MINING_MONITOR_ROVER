/* ---------------------------------------------------------------
   Rover Dashboard Client
   - connects to the ROS2 node Socket.IO stream
   - real-time feeds, gauges, radar, telemetry, and joystick
   - 12-column adaptive layout with drag-and-drop & width resizing
   - interactive LiDAR navigation map with A* obstacle avoidance
   ------------------------------------------------------------- */

const GAUGE_RANGES = {
  temperature: { min: -10, max: 80 },
  humidity: { min: 0, max: 100 },
  gas_reading: { min: 0, max: 1000 },
};

const STORAGE_KEY = "roverDashboard.settings.v2";
const BATTERY_MAX_RUNTIME_HOURS = 8;
const BATTERY_MAX_RANGE_METERS = 25000;

const ALL_PANELS = [
  "live_feed",
  "thermal_feed",
  "lidar_map",
  "temperature",
  "humidity",
  "gas_reading",
  "left_radar",
  "central_telemetry",
  "navigation_state",
  "joystick_values",
];

const defaultSettings = {
  order: [...ALL_PANELS],
  panelSpans: {
    live_feed: 6,
    thermal_feed: 6,
    lidar_map: 12,
    temperature: 4,
    humidity: 4,
    gas_reading: 4,
    left_radar: 4,
    central_telemetry: 4,
    navigation_state: 4,
    joystick_values: 4,
  },
  hidden: [],
  density: "comfortable",
  gasThreshold: 400,
  tempThreshold: 45,
};

const liveTelemetry = {
  temperature: null,
  humidity: null,
  gas_reading: null,
  gas_status: 0,
  left_distance: null,
  right_distance: null,
  speed_mode: 1,
  motor_left: 0,
  motor_right: 0,
  status: "NORMAL",
  battery: 75,
  lora: 85,
  wifihalow: 80,
  odom: { x: 0, y: 0, yaw: 0 },
};


function loadSettings() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return JSON.parse(JSON.stringify(defaultSettings));
    const parsed = JSON.parse(raw);

    // Merge missing panels in order and panelSpans
    const mergedOrder = Array.isArray(parsed.order) ? [...parsed.order] : [...defaultSettings.order];
    ALL_PANELS.forEach((key) => {
      if (!mergedOrder.includes(key)) mergedOrder.push(key);
    });

    const mergedSpans = { ...defaultSettings.panelSpans, ...(parsed.panelSpans || {}) };

    return {
      ...defaultSettings,
      ...parsed,
      order: mergedOrder,
      panelSpans: mergedSpans,
      hidden: Array.isArray(parsed.hidden) ? parsed.hidden : [],
    };
  } catch (e) {
    return JSON.parse(JSON.stringify(defaultSettings));
  }
}

function saveSettings(s) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(s));
  } catch (e) {
    /* localStorage unavailable */
  }
}

let settings = loadSettings();

/* ---------------- Layout & Panel Controllers ---------------- */

const grid = document.getElementById("grid");

function applyOrder() {
  settings.order.forEach((key) => {
    const el = grid.querySelector(`[data-panel="${key}"]`);
    if (el) grid.appendChild(el);
  });
}

function applySpans() {
  document.querySelectorAll(".panel").forEach((panel) => {
    const key = panel.dataset.panel;
    const span = settings.panelSpans[key] || (key === "lidar_map" ? 12 : 4);
    panel.dataset.span = String(span);
  });
}

function applyVisibility() {
  document.querySelectorAll(".panel").forEach((el) => {
    const key = el.dataset.panel;
    el.dataset.hidden = settings.hidden.includes(key) ? "true" : "false";
  });
}

function applyDensity() {
  const gapMap = { compact: "10px", comfortable: "18px", spacious: "26px" };
  document.documentElement.style.setProperty("--gap", gapMap[settings.density] || "18px");
  document.querySelectorAll("#densitySeg button").forEach((b) => {
    b.classList.toggle("active", b.dataset.density === settings.density);
  });
}

/* Initial Layout Application */
applyOrder();
applySpans();
applyVisibility();
applyDensity();

/* ---------------- Reordering & Panel Quick Controls ---------------- */

let dragSourceKey = null;
let currentDropTarget = null;
let dropAfter = false;

function initPanelDragAndControls() {
  document.querySelectorAll(".panel").forEach((panel) => {
    const key = panel.dataset.panel;
    const grip = panel.querySelector(".drag-grip");

    // Configure Draggable Grip
    if (grip) {
      grip.setAttribute("draggable", "true");

      grip.addEventListener("dragstart", (e) => {
        dragSourceKey = key;
        panel.classList.add("dragging");
        e.dataTransfer.setData("text/plain", key);
        e.dataTransfer.effectAllowed = "move";
      });

      grip.addEventListener("dragend", () => {
        panel.classList.remove("dragging");
        clearDragHighlights();
        dragSourceKey = null;
      });
    }

    // Panel Drag Over
    panel.addEventListener("dragover", (e) => {
      e.preventDefault();
      if (!dragSourceKey || dragSourceKey === key) return;

      const rect = panel.getBoundingClientRect();
      const midX = rect.left + rect.width / 2;
      const midY = rect.top + rect.height / 2;

      // Determine before or after relative to pointer
      const isHorizontal = rect.width > 300;
      dropAfter = isHorizontal ? e.clientX > midX : e.clientY > midY;

      clearDragHighlights();
      panel.classList.add(dropAfter ? "drag-over-after" : "drag-over-before");
      currentDropTarget = panel;
    });

    panel.addEventListener("dragleave", (e) => {
      if (!panel.contains(e.relatedTarget)) {
        panel.classList.remove("drag-over-before", "drag-over-after");
      }
    });

    panel.addEventListener("drop", (e) => {
      e.preventDefault();
      if (!dragSourceKey || dragSourceKey === key) return;

      const fromIndex = settings.order.indexOf(dragSourceKey);
      if (fromIndex !== -1) {
        settings.order.splice(fromIndex, 1);
      }

      let toIndex = settings.order.indexOf(key);
      if (toIndex === -1) toIndex = settings.order.length;
      if (dropAfter) toIndex += 1;

      settings.order.splice(toIndex, 0, dragSourceKey);
      applyOrder();
      saveSettings(settings);
      clearDragHighlights();
    });

    // Move Left / Earlier Button
    const btnLeft = panel.querySelector(".btn-move-left");
    if (btnLeft) {
      btnLeft.addEventListener("click", (e) => {
        e.stopPropagation();
        const idx = settings.order.indexOf(key);
        if (idx > 0) {
          const tmp = settings.order[idx];
          settings.order[idx] = settings.order[idx - 1];
          settings.order[idx - 1] = tmp;
          applyOrder();
          saveSettings(settings);
        }
      });
    }

    // Move Right / Later Button
    const btnRight = panel.querySelector(".btn-move-right");
    if (btnRight) {
      btnRight.addEventListener("click", (e) => {
        e.stopPropagation();
        const idx = settings.order.indexOf(key);
        if (idx !== -1 && idx < settings.order.length - 1) {
          const tmp = settings.order[idx];
          settings.order[idx] = settings.order[idx + 1];
          settings.order[idx + 1] = tmp;
          applyOrder();
          saveSettings(settings);
        }
      });
    }

    // Width Toggle Button (cycles 4 -> 6 -> 8 -> 12 -> 4)
    const btnSpan = panel.querySelector(".btn-size-toggle");
    if (btnSpan) {
      btnSpan.addEventListener("click", (e) => {
        e.stopPropagation();
        const cycle = [4, 6, 8, 12];
        const curSpan = Number(panel.dataset.span) || 4;
        const nextIdx = (cycle.indexOf(curSpan) + 1) % cycle.length;
        const nextSpan = cycle[nextIdx];
        settings.panelSpans[key] = nextSpan;
        panel.dataset.span = String(nextSpan);
        saveSettings(settings);
      });
    }
  });
}

function clearDragHighlights() {
  document.querySelectorAll(".panel").forEach((p) => {
    p.classList.remove("drag-over-before", "drag-over-after", "dragging");
  });
}

initPanelDragAndControls();

/* ---------------- Settings Overlay Wiring ---------------- */

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

  settings.order.forEach((key) => {
    const panel = grid.querySelector(`[data-panel="${key}"]`);
    if (!panel) return;
    const title = panel.dataset.title || key;
    const currentSpan = settings.panelSpans[key] || 4;

    const row = document.createElement("div");
    row.className = "toggle-row";
    row.innerHTML = `
      <div style="display:flex; align-items:center; gap:8px;">
        <span style="font-weight:600; font-size:12px; min-width:140px;">${title}</span>
        <div class="panel-span-selector">
          <button class="span-btn ${currentSpan === 4 ? "active" : ""}" data-key="${key}" data-span="4">⅓</button>
          <button class="span-btn ${currentSpan === 6 ? "active" : ""}" data-key="${key}" data-span="6">½</button>
          <button class="span-btn ${currentSpan === 8 ? "active" : ""}" data-key="${key}" data-span="8">⅔</button>
          <button class="span-btn ${currentSpan === 12 ? "active" : ""}" data-key="${key}" data-span="12">Full</button>
        </div>
      </div>
      <label class="switch">
        <input type="checkbox" data-key="${key}" ${settings.hidden.includes(key) ? "" : "checked"} />
        <span class="slider"></span>
      </label>`;
    container.appendChild(row);
  });

  // Visibility toggles
  container.querySelectorAll("input[type=checkbox]").forEach((cb) => {
    cb.addEventListener("change", () => {
      const key = cb.dataset.key;
      settings.hidden = settings.hidden.filter((k) => k !== key);
      if (!cb.checked) settings.hidden.push(key);
      applyVisibility();
      saveSettings(settings);
    });
  });

  // Span buttons
  container.querySelectorAll(".span-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      const key = btn.dataset.key;
      const span = Number(btn.dataset.span);
      settings.panelSpans[key] = span;
      applySpans();
      saveSettings(settings);

      btn.parentElement.querySelectorAll(".span-btn").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
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
  settings = JSON.parse(JSON.stringify(defaultSettings));
  saveSettings(settings);
  applyOrder();
  applySpans();
  applyVisibility();
  applyDensity();
  buildVisibilityToggles();
});

/* ---------------- Socket.IO Live Telemetry & Feeds ---------------- */

const socket = io();
const linkDot = document.getElementById("linkDot");
const linkText = document.getElementById("linkText");
let sensorsBlocked = false;

socket.on("connect", () => {
  linkDot.classList.add("connected");
  linkText.textContent = "Live";
});

socket.on("disconnect", () => {
  linkDot.classList.remove("connected");
  linkText.textContent = "Disconnected";
});

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function getSignalLevel(value) {
  const normalized = clamp(Number(value) || 0, 0, 100);
  if (normalized >= 75) return { level: "good", label: "Strong" };
  if (normalized >= 45) return { level: "medium", label: "Stable" };
  if (normalized >= 20) return { level: "warn", label: "Weak" };
  return { level: "critical", label: "Poor" };
}

function formatDuration(hours) {
  const totalMinutes = Math.max(0, Math.round(hours * 60));
  const hr = Math.floor(totalMinutes / 60);
  const min = totalMinutes % 60;
  if (hr > 0) return `${hr}h ${min}m`;
  return `${min}m`;
}

function formatDistance(distanceMeters) {
  const km = distanceMeters / 1000;
  if (km >= 10) return `${km.toFixed(1)} km`;
  if (km >= 1) return `${km.toFixed(2)} km`;
  return `${Math.round(distanceMeters)} m`;
}

function updateMeshStatus(meshKey, rawValue) {
  const wrapper = document.querySelector(`[data-mesh="${meshKey}"]`);
  if (!wrapper) return;
  const value = clamp(Number(rawValue) || 0, 0, 100);
  const signal = getSignalLevel(value);
  wrapper.dataset.level = signal.level;
  const levelEl = wrapper.querySelector(".mesh-level");
  if (levelEl) levelEl.textContent = `${Math.round(value)}%`;
}

function updateBatteryStatus(rawValue) {
  const batteryCluster = document.querySelector(".battery-cluster");
  const batteryStatus = document.querySelector(".battery-status");
  const batteryFill = document.getElementById("batteryFill");
  const batteryValue = document.getElementById("batteryValue");
  const runtimeValue = document.getElementById("runtimeValue");
  const rangeValue = document.getElementById("rangeValue");

  const pct = clamp(Number(rawValue) || 0, 0, 100);
  const level = pct >= 70 ? "good" : pct >= 35 ? "medium" : pct >= 15 ? "warn" : "critical";

  if (batteryCluster) batteryCluster.dataset.level = level;
  if (batteryStatus) batteryStatus.dataset.level = level;
  if (batteryFill) batteryFill.style.width = `${pct}%`;
  if (batteryValue) batteryValue.textContent = `${Math.round(pct)}%`;

  const runtimeHours = (pct / 100) * BATTERY_MAX_RUNTIME_HOURS;
  const rangeMeters = (pct / 100) * BATTERY_MAX_RANGE_METERS;

  if (runtimeValue) runtimeValue.textContent = formatDuration(runtimeHours);
  if (rangeValue) rangeValue.textContent = formatDistance(rangeMeters);
}

socket.on("battery", (msg) => { liveTelemetry.battery = msg.value; queueUiUpdate("battery", () => updateBatteryStatus(msg.value)); });
socket.on("lora", (msg) => { liveTelemetry.lora = msg.value; queueUiUpdate("lora", () => updateMeshStatus("lora", msg.value)); });
socket.on("wifihalow", (msg) => { liveTelemetry.wifihalow = msg.value; queueUiUpdate("wifihalow", () => updateMeshStatus("wifihalow", msg.value)); });

function setBadge(key, text, level) {
  const badge = document.querySelector(`[data-badge="${key}"]`);
  if (!badge) return;
  badge.textContent = text;
  badge.className = "badge" + (level ? ` ${level}` : "");
}

const pendingUi = new Map();
let uiFrameScheduled = false;

function queueUiUpdate(key, update) {
  pendingUi.set(key, update);
  if (uiFrameScheduled) return;
  uiFrameScheduled = true;
  requestAnimationFrame(() => {
    uiFrameScheduled = false;
    const updates = Array.from(pendingUi.values());
    pendingUi.clear();
    updates.forEach((update) => update());
  });
}

/* ---- Video Feeds ---- */
function bindFeed(event, imgId, placeholderId) {
  const img = document.getElementById(imgId);
  let latestFrame = null;
  let frameScheduled = false;
  let staleTimer = null;

  socket.on(event, (msg) => {
    latestFrame = msg.data_uri;
    if (!frameScheduled) {
      frameScheduled = true;
      requestAnimationFrame(() => {
        frameScheduled = false;
        if (!latestFrame) return;
        img.src = latestFrame;
        img.classList.add("has-frame");
      });
    }
    setBadge(event, "live", "live");
    clearTimeout(staleTimer);
    staleTimer = setTimeout(() => setBadge(event, "no signal"), 4000);
  });
}

bindFeed("live_feed", "liveFeedImg", "liveFeedPlaceholder");
bindFeed("thermal_feed", "thermalFeedImg", "thermalFeedPlaceholder");

/* ---- Gauges ---- */
const GAUGE_ARC_LENGTH = 220;

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

socket.on("temperature", (msg) => {
  if (!sensorsBlocked) queueUiUpdate("temperature", () => updateGauge("temperature", msg.value));
});
socket.on("humidity", (msg) => {
  if (!sensorsBlocked) queueUiUpdate("humidity", () => updateGauge("humidity", msg.value));
});
socket.on("gas_reading", (msg) => {
  if (!sensorsBlocked) queueUiUpdate("gas_reading", () => updateGauge("gas_reading", msg.value));
});

function updateTelemetryValue(key, value) {
  const element = document.getElementById(`val-${key}`);
  if (element) element.textContent = value;
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
    if (display) display.classList.toggle("active", cappedValue > 0);
  }
}

socket.on("gas_status", (msg) => {
  if (sensorsBlocked) return;
  queueUiUpdate("gas_status", () => {
    liveTelemetry.gas_status = Number(msg.value);
    liveTelemetry.gas_status = Number(msg.value);
    const alarm = Number(msg.value) !== 0;
    updateTelemetryValue("gas_status", alarm ? "ALARM" : "CLEAR");
    setBadge("status", alarm ? "gas alarm" : "online", alarm ? "alert" : "live");
  });
});

socket.on("left_distance", (msg) => {
  if (sensorsBlocked) return;
  queueUiUpdate("left_distance", () => {
    const rawValue = Number(msg.value) || 0;
    liveTelemetry.left_distance = rawValue;
    const value = rawValue > 50 ? 50 : rawValue;
    const displayLabel = rawValue > 50 ? "50+" : value.toFixed(1);
    updateTelemetryValue("left_distance", displayLabel);
    updateRadar("left", rawValue, 50);
    setBadge("left_distance", rawValue > 50 ? "50+ cm" : `${value.toFixed(0)} cm`, value < 50 ? "warn" : "live");
  });
});

socket.on("right_distance", (msg) => {
  if (sensorsBlocked) return;
  queueUiUpdate("right_distance", () => {
    const rawValue = Number(msg.value) || 0;
    liveTelemetry.right_distance = rawValue;
    const value = rawValue > 50 ? 50 : rawValue;
    const displayLabel = rawValue > 50 ? "50+" : value.toFixed(1);
    updateTelemetryValue("right_distance", displayLabel);
    updateRadar("right", rawValue, 50);
    setBadge("right_distance", rawValue > 50 ? "50+ cm" : `${value.toFixed(0)} cm`, value < 50 ? "warn" : "live");
  });
});

socket.on("speed_mode", (msg) => { liveTelemetry.speed_mode = msg.value; queueUiUpdate("speed_mode", () => updateTelemetryValue("speed_mode", msg.value)); });
socket.on("motor_left", (msg) => queueUiUpdate("motor_left", () => updateTelemetryValue("motor_left", msg.value)));
socket.on("motor_right", (msg) => queueUiUpdate("motor_right", () => updateTelemetryValue("motor_right", msg.value)));
socket.on("status", (msg) => {
  liveTelemetry.status = msg.value;
  queueUiUpdate("status", () => {
    updateTelemetryValue("status", msg.value);
    setBadge("status", "online", "live");
  });
});

function setNavigationBadge(text, level = null) {
  setBadge("navigation_state", text, level);
}

socket.on("full_control", (msg) => {
  const active = Boolean(msg.active);
  sensorsBlocked = active;
  updateTelemetryValue("full_control", active ? "ON" : "OFF");
  if (active) {
    ["temperature", "humidity", "gas_reading", "gas_status", "left_distance", "right_distance"].forEach((key) => {
      updateTelemetryValue(key, "--");
      setBadge(key, "blocked", "warn");
    });
    setBadge("status", "sensors blocked", "warn");
  }
  setNavigationBadge(active ? "FULL CONTROL" : "NORMAL", active ? "alert" : "live");
});

socket.on("rollback_status", (msg) => {
  const parts = String(msg.value || "").split("|");
  const status = parts[0] || "IDLE";
  const progressPart = parts.find((part) => part.startsWith("progress="));
  const progress = progressPart ? Number.parseFloat(progressPart.split("=", 2)[1]) : 0;
  updateTelemetryValue("rollback_status", status);
  updateTelemetryValue("rollback_progress", `${Number.isFinite(progress) ? progress.toFixed(1) : "0.0"}%`);
  if (status.includes("ACTIVE") || status.includes("RETURN")) {
    setNavigationBadge("ROLLBACK", "warn");
  }
});

socket.on("autonomous_status", (msg) => {
  const status = String(msg.value || "MANUAL");
  updateTelemetryValue("autonomous_status", status);
  if (status.includes("ACTIVE") || status.includes("DRIVING") || status.includes("ALIGNING") || status.includes("APPROACH")) {
    setNavigationBadge("AUTONOMOUS", "live");
  }
});

socket.on("odom", (msg) => {
  queueUiUpdate("odom", () => {
    const x = Number(msg.x) || 0;
    const y = Number(msg.y) || 0;
    const yaw = Number(msg.yaw) || 0;
    updateTelemetryValue("pose", `${x.toFixed(2)}, ${y.toFixed(2)}`);
    updateTelemetryValue("heading", (yaw * 180 / Math.PI).toFixed(1));
  });
});

socket.on("path", (msg) => {
  const points = Array.isArray(msg.points) ? msg.points : [];
  let length = 0;
  for (let index = 1; index < points.length; index += 1) {
    const dx = Number(points[index].x) - Number(points[index - 1].x);
    const dy = Number(points[index].y) - Number(points[index - 1].y);
    length += Math.hypot(dx, dy);
  }
  queueUiUpdate("path", () => updateTelemetryValue("path_length", length.toFixed(2)));
});

/* ---- Joystick Canvas & Virtual Controls ---- */
const joyCanvas = document.getElementById("joystickCanvas");
const joyCtx = joyCanvas ? joyCanvas.getContext("2d") : null;
const joyXEl = document.getElementById("joyX");
const joyYEl = document.getElementById("joyY");

function drawJoystick(x, y) {
  if (!joyCtx) return;
  const w = joyCanvas.width;
  const h = joyCanvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const r = Math.min(w, h) / 2 - 14;

  joyCtx.clearRect(0, 0, w, h);

  // Crosshairs
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

  // Stick position
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

let currentJoystick = { x: 0, y: 0 };
let lastSocketBtn1 = false;
let lastGamepadBtn1 = false;
let lastAutoDriveToggleTime = 0;

function requestToggleAutoDrive() {
  const now = Date.now();
  if (now - lastAutoDriveToggleTime < 500) return;
  lastAutoDriveToggleTime = now;
  if (lidarMapManager) lidarMapManager.toggleAutoDrive();
}

function applyJoystickInput(rawX, rawY) {
  const deadzone = 0.04;
  const x = Math.abs(rawX) > deadzone ? rawX : 0;
  const y = Math.abs(rawY) > deadzone ? rawY : 0;
  currentJoystick.x = x;
  currentJoystick.y = y;

  drawJoystick(x, y);
  if (joyXEl) joyXEl.textContent = x.toFixed(2);
  if (joyYEl) joyYEl.textContent = y.toFixed(2);
  const isMoving = Math.abs(x) > 0 || Math.abs(y) > 0;
  setBadge("joystick_values", isMoving ? "active" : "idle", isMoving ? "live" : "");
}

socket.on("joystick_values", (msg) => {
  const axes = msg.axes || [];
  // Use primary stick axes (axes[0]: steer, axes[1]: throttle)
  const x = axes.length > 0 ? axes[0] : 0;
  const y = axes.length > 1 ? axes[1] : 0;

  // Button 1 edge detection with shared debounce
  const buttons = msg.buttons || [];
  const btn1 = Boolean((buttons.length > 0 && buttons[0] === 1) || (buttons.length > 1 && buttons[1] === 1));
  if (btn1 && !lastSocketBtn1) {
    requestToggleAutoDrive();
  }
  lastSocketBtn1 = btn1;

  if (lidarMapManager && lidarMapManager.autoNavigating) {
    // Only manual stick deflection above 0.70 after grace period cancels auto drive
    if (Math.hypot(x, y) > 0.70 && (Date.now() - lidarMapManager.autoDriveStartTime > 1000)) {
      requestToggleAutoDrive();
    }
    return;
  }

  if (lidarMapManager) {
    lidarMapManager.joyX = x;
    lidarMapManager.joyY = y;
  }
  applyJoystickInput(x, y);
});

// HTML5 Web Gamepad API for direct ultra-low latency browser joystick polling
function pollWebGamepad() {
  if (navigator.getGamepads) {
    const gamepads = navigator.getGamepads();
    for (let i = 0; i < gamepads.length; i++) {
      const gp = gamepads[i];
      if (gp && gp.connected && gp.axes && gp.axes.length >= 2) {
        const x = gp.axes[0];
        const y = -gp.axes[1]; // Browser Y inverted

        const btn1 = Boolean(gp.buttons && ((gp.buttons[0] && gp.buttons[0].pressed) || (gp.buttons[1] && gp.buttons[1].pressed)));
        if (btn1 && !lastGamepadBtn1) {
          requestToggleAutoDrive();
        }
        lastGamepadBtn1 = btn1;

        if (lidarMapManager && lidarMapManager.autoNavigating) {
          if (Math.hypot(x, y) > 0.70 && (Date.now() - lidarMapManager.autoDriveStartTime > 1000)) {
            requestToggleAutoDrive();
          }
          break;
        }

        if (Math.abs(x) > 0.05 || Math.abs(y) > 0.05) {
          if (lidarMapManager) {
            lidarMapManager.joyX = x;
            lidarMapManager.joyY = y;
          }
          applyJoystickInput(x, y);
        }
        break;
      }
    }
  }
  requestAnimationFrame(pollWebGamepad);
}
requestAnimationFrame(pollWebGamepad);

/* ===============================================================
   LiDAR Map & Obstacle-Avoiding Guidance System
   - Interactive high-res occupancy grid map
   - In-box zoom with pan & wheel navigation
   - Flawless waypoint tracking (no corner sticking)
   - Color-coded target survey zone (Green=Good, Yellow=Normal, Red=Bad)
   - Instant audio chime & HUD arrival alert
   - Automated Mission Inspection Report & TXT/JSON Export
   ============================================================= */
class LidarMapManager {
  constructor() {
    this.canvas = document.getElementById("lidarMapCanvas");
    this.wrapper = document.getElementById("mapCanvasWrapper");
    if (!this.canvas || !this.wrapper) return;
    this.ctx = this.canvas.getContext("2d");

    // Header buttons
    this.btnZoomIn = document.getElementById("btnZoomIn");
    this.btnZoomOut = document.getElementById("btnZoomOut");
    this.btnResetView = document.getElementById("btnResetView");
    this.btnSetStart = document.getElementById("btnSetStart");
    this.btnSetGoal = document.getElementById("btnSetGoal");
    this.btnResetRover = document.getElementById("btnResetRover");
    this.btnAutoDrive = document.getElementById("btnAutoDrive");
    this.btnViewReport = document.getElementById("btnViewReport");
    this.hudStatus = document.getElementById("hudStatus");
    this.hudDistance = document.getElementById("hudDistance");
    this.hudCoords = document.getElementById("hudCoords");

    // Report modal & toast
    this.reportOverlay = document.getElementById("reportOverlay");
    this.closeReport = document.getElementById("closeReport");
    this.btnCloseReportBtn = document.getElementById("btnCloseReportBtn");
    this.reportToast = document.getElementById("reportToast");
    this.btnOpenReportFromToast = document.getElementById("btnOpenReportFromToast");
    this.btnExportTxt = document.getElementById("btnExportTxt");
    this.btnExportJson = document.getElementById("btnExportJson");

    this.mapData = null;
    this.mapImage = new Image();
    this.mapLoaded = false;
    this.traversable = null;

    this.mode = "idle"; // "idle" | "set_start" | "set_goal"
    this.startPoint = null;
    this.goalPoint = null;
    this.rover = { x: 0, y: 0, heading: 0 };

    this.currentPath = [];
    this.currentWaypointIndex = 1;
    this.pathLengthMeters = 0;
    this.pathStatus = "Initializing…";
    this.autoNavigating = false;
    this.autoDriveStartTime = 0;

    // Survey zone placed upon arrival
    this.surveyZone = null;
    this._toastTimer = null;

    // Zoom & pan parameters (default 1.45x zoom so map fills box nicely)
    this.zoomLevel = 1.45;
    this.panX = 0;
    this.panY = 0;
    this.scale = 1.0;
    this.offsetX = 0;
    this.offsetY = 0;
    this.isDragging = false;
    this.dragStart = { x: 0, y: 0 };
    this.hasDragged = false;

    this.joyX = 0;
    this.joyY = 0;
    this.keys = {};

    this.lastFrameTime = performance.now();
    this.lastPathCalcPos = { x: -999, y: -999 };

    this.init();
  }

  async init() {
    this.bindEvents();
    await this.fetchMap();
    requestAnimationFrame((t) => this.renderLoop(t));
  }

  async fetchMap() {
    setBadge("lidar_map", "loading");
    try {
      const res = await fetch("/api/map");
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const data = await res.json();
      this.mapData = data;

      const binaryStr = atob(data.grid_b64);
      const len = binaryStr.length;
      this.traversable = new Uint8Array(len);
      for (let i = 0; i < len; i++) {
        this.traversable[i] = binaryStr.charCodeAt(i);
      }

      this.mapImage.onload = () => {
        this.mapLoaded = true;
        this.startPoint = { x: data.default_start[0], y: data.default_start[1] };
        this.goalPoint = { x: data.default_goal[0], y: data.default_goal[1] };
        this.rover.x = this.startPoint.x;
        this.rover.y = this.startPoint.y;
        this.rover.heading = Math.atan2(this.goalPoint.y - this.rover.y, this.goalPoint.x - this.rover.x);
        this.fitMapToContainer();
        this.calculateAStarPath();
        setBadge("lidar_map", "active", "live");
      };
      this.mapImage.src = data.image_data;
    } catch (err) {
      console.warn("Failed to load LiDAR map:", err);
      setBadge("lidar_map", "offline", "warn");
      if (this.hudStatus) this.hudStatus.textContent = "Map offline";
    }
  }

  bindEvents() {
    window.addEventListener("resize", () => this.fitMapToContainer());

    if (this.btnZoomIn) {
      this.btnZoomIn.addEventListener("click", () => {
        this.zoomLevel = Math.min(3.8, this.zoomLevel * 1.25);
        this.fitMapToContainer();
      });
    }

    if (this.btnZoomOut) {
      this.btnZoomOut.addEventListener("click", () => {
        this.zoomLevel = Math.max(0.70, this.zoomLevel * 0.80);
        this.fitMapToContainer();
      });
    }

    if (this.btnResetView) {
      this.btnResetView.addEventListener("click", () => {
        this.zoomLevel = 1.45;
        this.panX = 0;
        this.panY = 0;
        this.fitMapToContainer();
      });
    }

    if (this.btnSetStart) {
      this.btnSetStart.addEventListener("click", () => {
        this.setMode(this.mode === "set_start" ? "idle" : "set_start");
      });
    }

    if (this.btnSetGoal) {
      this.btnSetGoal.addEventListener("click", () => {
        this.setMode(this.mode === "set_goal" ? "idle" : "set_goal");
      });
    }

    if (this.btnResetRover) {
      this.btnResetRover.addEventListener("click", () => {
        this.autoNavigating = false;
        this.updateAutoDriveButton();
        applyJoystickInput(0, 0);
        this.surveyZone = null;
        if (this.hudStatus) {
          this.hudStatus.className = "hud-chip";
          this.hudStatus.textContent = "Rover Reset";
        }
        if (this.startPoint) {
          this.rover.x = this.startPoint.x;
          this.rover.y = this.startPoint.y;
          this.calculateAStarPath();
        }
      });
    }

    if (this.btnAutoDrive) {
      this.btnAutoDrive.addEventListener("click", () => {
        this.toggleAutoDrive();
      });
    }

    if (this.btnViewReport) {
      this.btnViewReport.addEventListener("click", () => {
        this.openInspectionReport();
      });
    }

    if (this.closeReport) {
      this.closeReport.addEventListener("click", () => this.closeInspectionReport());
    }
    if (this.btnCloseReportBtn) {
      this.btnCloseReportBtn.addEventListener("click", () => this.closeInspectionReport());
    }
    if (this.reportOverlay) {
      this.reportOverlay.addEventListener("click", (e) => {
        if (e.target === this.reportOverlay) this.closeInspectionReport();
      });
    }
    if (this.btnOpenReportFromToast) {
      this.btnOpenReportFromToast.addEventListener("click", () => {
        if (this.reportToast) this.reportToast.classList.remove("show");
        this.openInspectionReport();
      });
    }
    if (this.btnExportTxt) {
      this.btnExportTxt.addEventListener("click", () => this.exportReportTxt());
    }
    if (this.btnExportJson) {
      this.btnExportJson.addEventListener("click", () => this.exportReportJson());
    }

    // Interactive mouse wheel zoom centered at cursor
    this.canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      const rect = this.canvas.getBoundingClientRect();
      const clientX = e.clientX - rect.left;
      const clientY = e.clientY - rect.top;

      const factor = e.deltaY < 0 ? 1.15 : 0.87;
      const newZoom = Math.max(0.70, Math.min(4.0, this.zoomLevel * factor));
      if (newZoom !== this.zoomLevel) {
        // Adjust pan to zoom towards mouse
        const w = this.canvas.width / (window.devicePixelRatio || 1);
        const h = this.canvas.height / (window.devicePixelRatio || 1);
        const mapX = (clientX - this.offsetX) / this.scale;
        const mapY = (clientY - this.offsetY) / this.scale;

        this.zoomLevel = newZoom;
        const baseScale = Math.min((w - 24) / this.mapData.width, (h - 24) / this.mapData.height);
        this.scale = baseScale * this.zoomLevel;

        this.offsetX = clientX - mapX * this.scale;
        this.offsetY = clientY - mapY * this.scale;
        this.panX = this.offsetX - (w - this.mapData.width * this.scale) / 2;
        this.panY = this.offsetY - (h - this.mapData.height * this.scale) / 2;
      }
    }, { passive: false });

    // Drag to pan map when in idle mode
    this.canvas.addEventListener("mousedown", (e) => {
      this.isDragging = true;
      this.hasDragged = false;
      this.dragStart = { x: e.clientX, y: e.clientY };
    });

    window.addEventListener("mousemove", (e) => {
      if (!this.isDragging) return;
      const dx = e.clientX - this.dragStart.x;
      const dy = e.clientY - this.dragStart.y;
      if (Math.abs(dx) > 3 || Math.abs(dy) > 3) {
        this.hasDragged = true;
      }
      this.dragStart = { x: e.clientX, y: e.clientY };
      this.panX += dx;
      this.panY += dy;
      this.fitMapToContainer();
    });

    window.addEventListener("mouseup", () => {
      this.isDragging = false;
    });

    this.canvas.addEventListener("click", (e) => {
      if (this.hasDragged) return; // Ignore drag release
      this.onCanvasClick(e);
    });

    window.addEventListener("keydown", (e) => {
      this.keys[e.code] = true;
      if (e.key === "1" && !e.repeat && document.activeElement.tagName !== "INPUT") {
        this.toggleAutoDrive();
      }
    });
    window.addEventListener("keyup", (e) => {
      this.keys[e.code] = false;
    });
  }

  toggleAutoDrive() {
    this.autoNavigating = !this.autoNavigating;
    this.updateAutoDriveButton();

    if (this.autoNavigating) {
      this.autoDriveStartTime = Date.now();
      const distToGoal = this.goalPoint
        ? Math.hypot(this.goalPoint.x - this.rover.x, this.goalPoint.y - this.rover.y) * (this.mapData ? this.mapData.resolution : 0.05)
        : 0;

      // If already near goal (< 0.70m) or survey zone exists, reset to start point first
      if ((distToGoal < 0.70 || this.surveyZone) && this.startPoint) {
        this.rover.x = this.startPoint.x;
        this.rover.y = this.startPoint.y;
        this.surveyZone = null;
        if (this.hudStatus) this.hudStatus.className = "hud-chip";
      }

      this.currentWaypointIndex = 1;
      this.calculateAStarPath();
      this.pathStatus = "Auto Driving to Target…";
      setBadge("lidar_map", "auto drive", "live");
    } else {
      applyJoystickInput(0, 0);
      this.pathStatus = "Manual Control";
      setBadge("lidar_map", "active", "live");
    }
  }

  updateAutoDriveButton() {
    if (this.btnAutoDrive) {
      this.btnAutoDrive.classList.toggle("active", this.autoNavigating);
      this.btnAutoDrive.textContent = this.autoNavigating ? "⏸ Stop Auto (Btn 1)" : "▶ Auto Drive (Btn 1)";
    }
  }

  setMode(newMode) {
    this.mode = newMode;
    if (this.btnSetStart) this.btnSetStart.classList.toggle("active", newMode === "set_start");
    if (this.btnSetGoal) this.btnSetGoal.classList.toggle("active", newMode === "set_goal");
    this.canvas.style.cursor = newMode !== "idle" ? "crosshair" : "grab";
  }

  onCanvasClick(e) {
    if (!this.mapLoaded || !this.mapData) return;
    const rect = this.canvas.getBoundingClientRect();
    const clientX = e.clientX - rect.left;
    const clientY = e.clientY - rect.top;

    const mapX = (clientX - this.offsetX) / this.scale;
    const mapY = (clientY - this.offsetY) / this.scale;

    if (mapX < 0 || mapX >= this.mapData.width || mapY < 0 || mapY >= this.mapData.height) return;

    const roundX = Math.round(mapX);
    const roundY = Math.round(mapY);

    if (this.mode === "set_start") {
      this.startPoint = { x: roundX, y: roundY };
      this.rover.x = roundX;
      this.rover.y = roundY;
      this.surveyZone = null;
      if (this.hudStatus) this.hudStatus.className = "hud-chip";
      this.setMode("idle");
      this.calculateAStarPath();
    } else if (this.mode === "set_goal") {
      this.goalPoint = { x: roundX, y: roundY };
      this.surveyZone = null;
      if (this.hudStatus) this.hudStatus.className = "hud-chip";
      this.setMode("idle");
      this.calculateAStarPath();
    } else {
      this.goalPoint = { x: roundX, y: roundY };
      this.surveyZone = null;
      if (this.hudStatus) this.hudStatus.className = "hud-chip";
      this.calculateAStarPath();
    }
  }

  fitMapToContainer() {
    if (!this.mapData) return;
    const dpr = window.devicePixelRatio || 1;
    const w = this.wrapper.clientWidth || 600;
    const h = this.wrapper.clientHeight || 480;

    this.canvas.width = w * dpr;
    this.canvas.height = h * dpr;
    this.canvas.style.width = `${w}px`;
    this.canvas.style.height = `${h}px`;

    this.ctx.setTransform(1, 0, 0, 1, 0, 0);
    this.ctx.scale(dpr, dpr);

    const baseScaleX = (w - 24) / this.mapData.width;
    const baseScaleY = (h - 24) / this.mapData.height;
    const baseScale = Math.min(baseScaleX, baseScaleY);
    this.scale = baseScale * this.zoomLevel;

    // Center map around the midpoint of rover/goal or canvas center
    const defaultCenterX = (w - this.mapData.width * this.scale) / 2;
    const defaultCenterY = (h - this.mapData.height * this.scale) / 2;

    this.offsetX = defaultCenterX + this.panX;
    this.offsetY = defaultCenterY + this.panY;
  }

  isTraversable(mapX, mapY) {
    if (!this.mapData || !this.traversable) return false;
    const gx = Math.floor(mapX / this.mapData.grid_scale);
    const gy = Math.floor(mapY / this.mapData.grid_scale);
    if (gx < 0 || gx >= this.mapData.grid_width || gy < 0 || gy >= this.mapData.grid_height) return false;
    return this.traversable[gy * this.mapData.grid_width + gx] === 1;
  }

  findNearestTraversable(x, y, gw, gh) {
    for (let r = 1; r < 35; r++) {
      for (let dy = -r; dy <= r; dy++) {
        for (let dx = -r; dx <= r; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          if (nx >= 0 && nx < gw && ny >= 0 && ny < gh) {
            if (this.traversable[ny * gw + nx] === 1) return [nx, ny];
          }
        }
      }
    }
    return null;
  }

  calculateAStarPath() {
    if (!this.mapData || !this.traversable || !this.goalPoint) return;
    const scale = this.mapData.grid_scale;
    const gw = this.mapData.grid_width;
    const gh = this.mapData.grid_height;

    const sx = Math.max(0, Math.min(gw - 1, Math.floor(this.rover.x / scale)));
    const sy = Math.max(0, Math.min(gh - 1, Math.floor(this.rover.y / scale)));
    const gx = Math.max(0, Math.min(gw - 1, Math.floor(this.goalPoint.x / scale)));
    const gy = Math.max(0, Math.min(gh - 1, Math.floor(this.goalPoint.y / scale)));

    let actualSx = sx, actualSy = sy;
    if (this.traversable[sy * gw + sx] !== 1) {
      const near = this.findNearestTraversable(sx, sy, gw, gh);
      if (near) { actualSx = near[0]; actualSy = near[1]; }
    }

    let actualGx = gx, actualGy = gy;
    if (this.traversable[gy * gw + gx] !== 1) {
      const near = this.findNearestTraversable(gx, gy, gw, gh);
      if (near) { actualGx = near[0]; actualGy = near[1]; }
    }

    const startIdx = actualSy * gw + actualSx;
    const goalIdx = actualGy * gw + actualGx;

    if (startIdx === goalIdx) {
      this.currentPath = [{ x: this.rover.x, y: this.rover.y }, { x: this.goalPoint.x, y: this.goalPoint.y }];
      this.currentWaypointIndex = 1;
      this.pathLengthMeters = 0;
      this.pathStatus = "🎯 At Goal";
      return;
    }

    const openSet = new BinaryMinHeap((a, b) => a.f - b.f);
    const cameFrom = new Int32Array(gw * gh).fill(-1);
    const gScore = new Float32Array(gw * gh).fill(Infinity);
    const closedSet = new Uint8Array(gw * gh);

    const heuristicWeight = 1.15;
    gScore[startIdx] = 0;
    openSet.push({ idx: startIdx, f: Math.hypot(actualGx - actualSx, actualGy - actualSy) * heuristicWeight });

    const dx = [-1, 1, 0, 0, -1, 1, -1, 1];
    const dy = [0, 0, -1, 1, -1, -1, 1, 1];
    const cost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414];

    let found = false;
    let iterations = 0;
    const maxIterations = 25000;

    while (openSet.size() > 0 && iterations++ < maxIterations) {
      const current = openSet.pop();
      const currIdx = current.idx;

      if (closedSet[currIdx]) continue;
      closedSet[currIdx] = 1;

      if (currIdx === goalIdx) {
        found = true;
        break;
      }

      const cx = currIdx % gw;
      const cy = Math.floor(currIdx / gw);
      const currentG = gScore[currIdx];

      for (let i = 0; i < 8; i++) {
        const nx = cx + dx[i];
        const ny = cy + dy[i];
        if (nx < 0 || nx >= gw || ny < 0 || ny >= gh) continue;

        const nextIdx = ny * gw + nx;
        if (this.traversable[nextIdx] !== 1 || closedSet[nextIdx]) continue;

        const tentativeG = currentG + cost[i];
        if (tentativeG < gScore[nextIdx]) {
          cameFrom[nextIdx] = currIdx;
          gScore[nextIdx] = tentativeG;
          const h = Math.hypot(actualGx - nx, actualGy - ny) * heuristicWeight;
          openSet.push({ idx: nextIdx, f: tentativeG + h });
        }
      }
    }

    if (!found) {
      this.pathStatus = "Routing to Target…";
      this.currentPath = [{ x: this.rover.x, y: this.rover.y }, { x: this.goalPoint.x, y: this.goalPoint.y }];
      this.currentWaypointIndex = 1;
      this.pathLengthMeters = Math.hypot(this.goalPoint.x - this.rover.x, this.goalPoint.y - this.rover.y) * this.mapData.resolution;
      return;
    }

    const rawWaypoints = [];
    let curr = goalIdx;
    while (curr !== -1) {
      const px = (curr % gw) * scale + scale / 2;
      const py = Math.floor(curr / gw) * scale + scale / 2;
      rawWaypoints.push({ x: px, y: py });
      curr = cameFrom[curr];
    }
    rawWaypoints.reverse();

    const smoothed = this.smoothPath(rawWaypoints);

    if (smoothed.length > 0) {
      smoothed[0] = { x: this.rover.x, y: this.rover.y };
      smoothed[smoothed.length - 1] = { x: this.goalPoint.x, y: this.goalPoint.y };
    }

    this.currentPath = smoothed;
    this.currentWaypointIndex = 1;

    let totalLenPx = 0;
    for (let i = 1; i < smoothed.length; i++) {
      totalLenPx += Math.hypot(smoothed[i].x - smoothed[i - 1].x, smoothed[i].y - smoothed[i - 1].y);
    }
    this.pathLengthMeters = totalLenPx * this.mapData.resolution;
    if (!this.autoNavigating) {
      this.pathStatus = "Obstacle-Avoiding Route Ready";
    }
  }

  smoothPath(points) {
    if (points.length <= 2) return points;
    const result = [points[0]];
    let currentIdx = 0;

    while (currentIdx < points.length - 1) {
      let furthest = currentIdx + 1;
      const maxAhead = Math.min(points.length - 1, currentIdx + 8);
      for (let next = maxAhead; next > currentIdx + 1; next--) {
        if (this.hasLineOfSight(points[currentIdx], points[next])) {
          furthest = next;
          break;
        }
      }
      result.push(points[furthest]);
      currentIdx = furthest;
    }
    return result;
  }

  hasLineOfSight(p1, p2) {
    const dist = Math.hypot(p2.x - p1.x, p2.y - p1.y);
    const steps = Math.ceil(dist / 2);
    for (let i = 1; i < steps; i++) {
      const t = i / steps;
      const x = p1.x + (p2.x - p1.x) * t;
      const y = p1.y + (p2.y - p1.y) * t;
      if (!this.isTraversable(x, y)) return false;
    }
    return true;
  }

  playArrivalChime() {
    try {
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      if (!AudioCtx) return;
      const ctx = new AudioCtx();
      const playTone = (freq, start, dur) => {
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.type = "sine";
        osc.frequency.setValueAtTime(freq, ctx.currentTime + start);
        gain.gain.setValueAtTime(0.18, ctx.currentTime + start);
        gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + start + dur);
        osc.connect(gain);
        gain.connect(ctx.destination);
        osc.start(ctx.currentTime + start);
        osc.stop(ctx.currentTime + start + dur);
      };
      playTone(587.33, 0.0, 0.16); // D5
      playTone(880.00, 0.14, 0.20); // A5
      playTone(1174.66, 0.30, 0.38); // D6
    } catch (_) {}
  }

  updateRoverPhysics(dt) {
    if (!this.mapLoaded || !this.mapData) return;

    if (this.autoNavigating) {
      if (!this.currentPath || this.currentPath.length < 2) {
        this.calculateAStarPath();
      }

      const distToGoal = this.goalPoint
        ? Math.hypot(this.goalPoint.x - this.rover.x, this.goalPoint.y - this.rover.y) * this.mapData.resolution
        : 0;

      // Arrival trigger: when within 0.85m of goal OR finished last waypoint
      if (distToGoal < 0.85 || (this.currentWaypointIndex >= this.currentPath.length)) {
        this.onTargetReached();
        return;
      }

      // Waypoint tracking
      let targetPt = this.currentPath[this.currentWaypointIndex];
      if (!targetPt) targetPt = this.goalPoint;

      let dx = targetPt.x - this.rover.x;
      let dy = targetPt.y - this.rover.y;
      let distToWpt = Math.hypot(dx, dy);

      // Advance waypoint if within 7.5 pixels
      if (distToWpt < 7.5 && this.currentWaypointIndex < this.currentPath.length - 1) {
        this.currentWaypointIndex++;
        targetPt = this.currentPath[this.currentWaypointIndex];
        dx = targetPt.x - this.rover.x;
        dy = targetPt.y - this.rover.y;
        distToWpt = Math.hypot(dx, dy);
      }

      const targetAngle = Math.atan2(dy, dx);
      let angleDiff = targetAngle - this.rover.heading;
      while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
      while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

      // Turn smoothly towards waypoint
      this.rover.heading += angleDiff * Math.min(1.0, 10.0 * dt);

      const linearSpeed = 52.0;
      const step = Math.min(linearSpeed * dt, distToWpt);
      this.rover.x += Math.cos(this.rover.heading) * step;
      this.rover.y += Math.sin(this.rover.heading) * step;

      // Mirror steering & throttle onto dashboard analog joystick
      const simSteer = Math.max(-1.0, Math.min(1.0, angleDiff * 2.2));
      const simThrottle = 0.70;
      applyJoystickInput(simSteer, simThrottle);
      return;
    }

    // Manual driving mode
    let inputSteer = this.joyX;
    let inputDrive = this.joyY;

    if (this.keys["ArrowLeft"] || this.keys["KeyA"]) inputSteer = -1.0;
    if (this.keys["ArrowRight"] || this.keys["KeyD"]) inputSteer = 1.0;
    if (this.keys["ArrowUp"] || this.keys["KeyW"]) inputDrive = 1.0;
    if (this.keys["ArrowDown"] || this.keys["KeyS"]) inputDrive = -1.0;

    const deadzone = 0.05;
    const isMoving = Math.abs(inputSteer) > deadzone || Math.abs(inputDrive) > deadzone;

    if (isMoving) {
      const turnRate = 2.4;
      this.rover.heading += inputSteer * turnRate * dt;

      const linearSpeed = 50.0;
      const dist = inputDrive * linearSpeed * dt;

      const proposedX = this.rover.x + Math.cos(this.rover.heading) * dist;
      const proposedY = this.rover.y + Math.sin(this.rover.heading) * dist;

      if (this.isTraversable(proposedX, proposedY)) {
        this.rover.x = proposedX;
        this.rover.y = proposedY;
      } else if (this.isTraversable(proposedX, this.rover.y)) {
        this.rover.x = proposedX;
      } else if (this.isTraversable(this.rover.x, proposedY)) {
        this.rover.y = proposedY;
      }

      // Check arrival in manual mode as well
      if (this.goalPoint && !this.surveyZone) {
        const distToGoal = Math.hypot(this.goalPoint.x - this.rover.x, this.goalPoint.y - this.rover.y) * this.mapData.resolution;
        if (distToGoal < 0.85) {
          this.onTargetReached();
        }
      }

      const now = performance.now();
      const moved = Math.hypot(this.rover.x - this.lastPathCalcPos.x, this.rover.y - this.lastPathCalcPos.y);
      if (moved > 10 || now - this.lastPathCalcTime > 500) {
        this.lastPathCalcPos = { x: this.rover.x, y: this.rover.y };
        this.lastPathCalcTime = now;
        this.calculateAStarPath();
      }
    }
  }

  onTargetReached() {
    this.autoNavigating = false;
    this.updateAutoDriveButton();
    applyJoystickInput(0, 0);

    // Snap rover exactly to goal
    if (this.goalPoint) {
      this.rover.x = this.goalPoint.x;
      this.rover.y = this.goalPoint.y;
    }

    // Play immediate audio arrival chime
    this.playArrivalChime();

    // Evaluate Environmental Sensor Telemetry
    const gas = Number(liveTelemetry.gas_reading) || 0;
    const gasStatus = Number(liveTelemetry.gas_status) || 0;
    const temp = Number(liveTelemetry.temperature) || 0;

    let rating = "GOOD";
    let color = "#00e676";
    let fill = "rgba(0, 230, 118, 0.35)";
    let label = "SAFE ZONE";

    if (gas >= 350 || gasStatus === 1 || temp >= 45) {
      rating = "BAD";
      color = "#ff4d4d";
      fill = "rgba(255, 77, 77, 0.38)";
      label = "HAZARD ZONE";
    } else if (gas >= 180 || temp >= 35) {
      rating = "NORMAL";
      color = "#ffc107";
      fill = "rgba(255, 193, 7, 0.35)";
      label = "CAUTION ZONE";
    }

    this.pathStatus = `🎯 TARGET REACHED — ${label}`;
    setBadge("lidar_map", `at target (${rating})`, "live");

    // Update HUD Chip with pulsing status
    if (this.hudStatus) {
      this.hudStatus.textContent = `🎯 TARGET REACHED — [ ${label} ]`;
      this.hudStatus.className = `hud-chip arrived-${rating.toLowerCase()}`;
    }

    // Create the marked environmental survey area on map
    this.surveyZone = {
      x: this.goalPoint.x,
      y: this.goalPoint.y,
      rating: rating,
      color: color,
      fill: fill,
      label: label,
      gas: gas,
      temp: temp,
      gasStatus: gasStatus,
      radius: 36,
      timestamp: Date.now()
    };

    if (this.btnViewReport) this.btnViewReport.style.display = "inline-flex";

    // Show floating toast with countdown
    this.showToast("🎯 TARGET REACHED!", `Survey Area Marked (${label}). Opening Report in 3s…`, 3200);

    // After 3 seconds, show report ready toast and automatically display documented report modal
    setTimeout(() => {
      this.showToast("📋 Inspection Report Ready", "Inspection report compiled. Review documented data.", 6000);
      this.openInspectionReport();
    }, 3000);
  }

  openInspectionReport() {
    if (!this.reportOverlay) return;
    const timeStr = new Date().toLocaleString();
    const timeEl = document.getElementById("reportTimestamp");
    if (timeEl) timeEl.textContent = timeStr;

    const gas = Number(liveTelemetry.gas_reading) || 0;
    const gasStatus = Number(liveTelemetry.gas_status) || 0;
    const temp = Number(liveTelemetry.temperature) || 0;
    const hum = Number(liveTelemetry.humidity) || 0;
    const leftDist = liveTelemetry.left_distance !== null ? Number(liveTelemetry.left_distance).toFixed(1) : "--";
    const rightDist = liveTelemetry.right_distance !== null ? Number(liveTelemetry.right_distance).toFixed(1) : "--";
    const batt = liveTelemetry.battery !== null ? Number(liveTelemetry.battery).toFixed(0) : "--";
    const lora = liveTelemetry.lora !== null ? Number(liveTelemetry.lora).toFixed(0) : "--";
    const halow = liveTelemetry.wifihalow !== null ? Number(liveTelemetry.wifihalow).toFixed(0) : "--";
    const roverStatus = liveTelemetry.status || "OPERATIONAL";

    let rating = "GOOD";
    if (gas >= 350 || gasStatus === 1 || temp >= 45) {
      rating = "BAD";
    } else if (gas >= 180 || temp >= 35) {
      rating = "NORMAL";
    }

    const card = document.getElementById("reportRatingCard");
    const badge = document.getElementById("reportRatingBadge");
    const title = document.getElementById("reportRatingTitle");
    const desc = document.getElementById("reportRatingDesc");

    if (card && badge && title && desc) {
      card.className = "report-rating-card " + (rating === "GOOD" ? "rating-good" : rating === "NORMAL" ? "rating-normal" : "rating-bad");
      if (rating === "GOOD") {
        badge.textContent = "SAFE / CLEAR";
        title.textContent = "Safe Environmental Status (Zone Approved)";
        desc.textContent = "All atmospheric gas levels, temperature, and humidity are within acceptable underground mining safety guidelines.";
      } else if (rating === "NORMAL") {
        badge.textContent = "MODERATE / CAUTION";
        title.textContent = "Caution Advised (Elevated Parameters)";
        desc.textContent = "Mildly elevated atmospheric gas or temperature detected. Protective equipment recommended; continuous telemetry monitoring active.";
      } else {
        badge.textContent = "HAZARD DETECTED";
        title.textContent = "Hazardous Environment (Entry Prohibited)";
        desc.textContent = "Dangerous combustible gas concentration or critical thermal threshold exceeded. Automatic containment warning dispatched.";
      }
    }

    // Stats
    const sPos = document.getElementById("repStartPos");
    const gPos = document.getElementById("repGoalPos");
    const dTrav = document.getElementById("repDistance");
    const nMode = document.getElementById("repMode");
    if (sPos) sPos.textContent = this.startPoint ? `(${this.startPoint.x}, ${this.startPoint.y})` : "--";
    if (gPos) gPos.textContent = this.goalPoint ? `(${this.goalPoint.x}, ${this.goalPoint.y})` : "--";
    if (dTrav) dTrav.textContent = `${this.pathLengthMeters.toFixed(2)} m`;
    if (nMode) nMode.textContent = "AUTONOMOUS (A* GUIDED)";

    // Table rows
    const tbody = document.getElementById("reportBodyTelemetry");
    if (tbody) {
      const rows = [
        { name: "Combustible Gas Level", val: `${gas} ppm`, unit: "ppm", thresh: "< 200 ppm", status: gas >= 350 ? "hazard" : gas >= 200 ? "warn" : "safe", statusText: gas >= 350 ? "CRITICAL" : gas >= 200 ? "ELEVATED" : "NOMINAL" },
        { name: "Combustible Gas Alarm", val: gasStatus === 1 ? "ACTIVE ALARM" : "CLEAR", unit: "digital", thresh: "0 (Clear)", status: gasStatus === 1 ? "hazard" : "safe", statusText: gasStatus === 1 ? "ALARM" : "OK" },
        { name: "Ambient Temperature", val: `${temp.toFixed(1)} °C`, unit: "°C", thresh: "< 38.0 °C", status: temp >= 45 ? "hazard" : temp >= 38 ? "warn" : "safe", statusText: temp >= 45 ? "CRITICAL" : temp >= 38 ? "WARM" : "NOMINAL" },
        { name: "Relative Humidity", val: `${hum.toFixed(1)} %RH`, unit: "%RH", thresh: "30 - 80 %", status: (hum < 20 || hum > 85) ? "warn" : "safe", statusText: (hum < 20 || hum > 85) ? "WARNING" : "NORMAL" },
        { name: "Left Obstacle Clearance", val: `${leftDist} cm`, unit: "cm", thresh: "> 30 cm", status: (leftDist !== "--" && Number(leftDist) < 20) ? "warn" : "safe", statusText: (leftDist !== "--" && Number(leftDist) < 20) ? "CLOSE" : "CLEAR" },
        { name: "Right Obstacle Clearance", val: `${rightDist} cm`, unit: "cm", thresh: "> 30 cm", status: (rightDist !== "--" && Number(rightDist) < 20) ? "warn" : "safe", statusText: (rightDist !== "--" && Number(rightDist) < 20) ? "CLOSE" : "CLEAR" },
        { name: "Battery Capacity", val: `${batt}%`, unit: "%", thresh: "> 25%", status: (batt !== "--" && Number(batt) < 20) ? "hazard" : (batt !== "--" && Number(batt) < 35) ? "warn" : "safe", statusText: (batt !== "--" && Number(batt) < 20) ? "LOW" : "NOMINAL" },
        { name: "LoRa Mesh Link", val: `${lora}%`, unit: "%", thresh: "> 40%", status: (lora !== "--" && Number(lora) < 40) ? "warn" : "safe", statusText: (lora !== "--" && Number(lora) < 40) ? "WEAK" : "STABLE" },
        { name: "WiFi HaLow Signal", val: `${halow}%`, unit: "%", thresh: "> 40%", status: (halow !== "--" && Number(halow) < 40) ? "warn" : "safe", statusText: (halow !== "--" && Number(halow) < 40) ? "WEAK" : "STRONG" },
        { name: "System Operating Status", val: roverStatus, unit: "state", thresh: "OPERATIONAL", status: roverStatus === "EMERGENCY" ? "hazard" : "safe", statusText: roverStatus }
      ];

      tbody.innerHTML = rows.map(r => `
        <tr>
          <td><strong>${r.name}</strong></td>
          <td>${r.val}</td>
          <td><code>${r.unit}</code></td>
          <td style="color:var(--text-dim);">${r.thresh}</td>
          <td><span class="status-pill ${r.status}">${r.statusText}</span></td>
        </tr>
      `).join("");
    }

    this.reportOverlay.classList.add("open");
  }

  closeInspectionReport() {
    if (this.reportOverlay) this.reportOverlay.classList.remove("open");
  }

  showToast(title, subtitle, durationMs = 4000) {
    if (!this.reportToast) return;
    const tEl = document.getElementById("toastTitle");
    const sEl = document.getElementById("toastSub");
    if (tEl) tEl.textContent = title;
    if (sEl) sEl.textContent = subtitle;
    this.reportToast.classList.add("show");
    clearTimeout(this._toastTimer);
    this._toastTimer = setTimeout(() => {
      this.reportToast.classList.remove("show");
    }, durationMs);
  }

  exportReportTxt() {
    const timeStr = new Date().toISOString();
    const gas = Number(liveTelemetry.gas_reading) || 0;
    const gasStatus = Number(liveTelemetry.gas_status) || 0;
    const temp = Number(liveTelemetry.temperature) || 0;
    const hum = Number(liveTelemetry.humidity) || 0;
    const leftDist = liveTelemetry.left_distance !== null ? Number(liveTelemetry.left_distance).toFixed(1) : "--";
    const rightDist = liveTelemetry.right_distance !== null ? Number(liveTelemetry.right_distance).toFixed(1) : "--";
    const batt = liveTelemetry.battery !== null ? Number(liveTelemetry.battery).toFixed(0) : "--";
    const lora = liveTelemetry.lora !== null ? Number(liveTelemetry.lora).toFixed(0) : "--";
    const halow = liveTelemetry.wifihalow !== null ? Number(liveTelemetry.wifihalow).toFixed(0) : "--";
    const roverStatus = liveTelemetry.status || "OPERATIONAL";

    let rating = "GOOD (SAFE)";
    if (gas >= 350 || gasStatus === 1 || temp >= 45) rating = "BAD (HAZARD)";
    else if (gas >= 180 || temp >= 35) rating = "NORMAL (CAUTION)";

    const report = `========================================================================
           DRILLPULSE MINE ROVER - MISSION INSPECTION REPORT
========================================================================
Generated: ${new Date().toLocaleString()} (${timeStr})
Vehicle: DrillPulse Exploration Rover Unit 01
Safety Assessment: [ ${rating} ]

1. NAVIGATION & MISSION OVERVIEW
------------------------------------------------------------------------
Start Coordinates (px)   : (${this.startPoint ? this.startPoint.x : "--"}, ${this.startPoint ? this.startPoint.y : "--"})
Target Coordinates (px)  : (${this.goalPoint ? this.goalPoint.x : "--"}, ${this.goalPoint ? this.goalPoint.y : "--"})
Total Traversal Distance : ${this.pathLengthMeters.toFixed(2)} meters
Navigation Mode          : Autonomous Obstacle-Avoiding Path (A* Algorithm)
Survey Status            : Target Reached & Survey Zone Deployed

2. ATMOSPHERIC & TELEMETRY READINGS
------------------------------------------------------------------------
Parameter                  Value         Threshold        Condition
------------------------------------------------------------------------
Combustible Gas Level      : ${gas} ppm      (< 200 ppm)      ${gas >= 350 ? "[CRITICAL]" : gas >= 200 ? "[ELEVATED]" : "[NOMINAL]"}
Gas Alarm Sensor           : ${gasStatus === 1 ? "ACTIVE ALARM" : "CLEAR"}    (0 Clear)        ${gasStatus === 1 ? "[ALARM]" : "[OK]"}
Ambient Temperature        : ${temp.toFixed(1)} °C      (< 38 °C)        ${temp >= 45 ? "[CRITICAL]" : temp >= 38 ? "[WARM]" : "[NOMINAL]"}
Relative Humidity          : ${hum.toFixed(1)} %RH     (30 - 80%)       ${(hum < 20 || hum > 85) ? "[WARNING]" : "[NORMAL]"}
Left Ultrasonic Clearance  : ${leftDist} cm     (> 30 cm)        ${(leftDist !== "--" && Number(leftDist) < 20) ? "[CLOSE]" : "[CLEAR]"}
Right Ultrasonic Clearance : ${rightDist} cm    (> 30 cm)        ${(rightDist !== "--" && Number(rightDist) < 20) ? "[CLOSE]" : "[CLEAR]"}
Battery State of Charge    : ${batt}%          (> 25%)          ${(batt !== "--" && Number(batt) < 20) ? "[LOW]" : "[NOMINAL]"}
LoRa Long-Range Signal     : ${lora}%          (> 40%)          ${(lora !== "--" && Number(lora) < 40) ? "[WEAK]" : "[STABLE]"}
WiFi HaLow Signal          : ${halow}%         (> 40%)          ${(halow !== "--" && Number(halow) < 40) ? "[WEAK]" : "[STRONG]"}
System Health              : ${roverStatus}                     [ACTIVE]
------------------------------------------------------------------------

3. SUMMARY & RECOMMENDATIONS
------------------------------------------------------------------------
${rating.startsWith("BAD") ? "CRITICAL WARNING: Atmospheric gas or thermal limits exceeded. Entry strictly prohibited." : rating.startsWith("NORMAL") ? "NOTICE: Elevated telemetry parameters observed. Wear standard protective gear." : "CONFIRMED: Target survey area is clear and stable for personnel operations."}
========================================================================
END OF REPORT
`;
    const blob = new Blob([report], { type: "text/plain;charset=utf-8" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `mission_report_${Date.now()}.txt`;
    a.click();
    URL.revokeObjectURL(url);
  }

  exportReportJson() {
    const data = {
      report_id: `DP-INSPECT-${Date.now()}`,
      timestamp: new Date().toISOString(),
      zone_assessment: {
        rating: this.surveyZone ? this.surveyZone.rating : "UNKNOWN",
        label: this.surveyZone ? this.surveyZone.label : "UNKNOWN"
      },
      navigation: {
        start_point_px: this.startPoint,
        goal_point_px: this.goalPoint,
        path_distance_meters: Number(this.pathLengthMeters.toFixed(2)),
        mode: "AUTONOMOUS"
      },
      telemetry: { ...liveTelemetry }
    };
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `mission_report_${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
  }

  renderLoop(time) {
    const dt = Math.min(0.1, (time - this.lastFrameTime) / 1000);
    this.lastFrameTime = time;

    this.updateRoverPhysics(dt);
    this.render();

    requestAnimationFrame((t) => this.renderLoop(t));
  }

  render() {
    if (!this.mapLoaded || !this.mapData) return;

    const w = this.canvas.width / (window.devicePixelRatio || 1);
    const h = this.canvas.height / (window.devicePixelRatio || 1);

    this.ctx.clearRect(0, 0, w, h);

    if (this.hudStatus && !this.surveyZone) this.hudStatus.textContent = `Status: ${this.pathStatus}`;
    if (this.hudDistance) this.hudDistance.textContent = `Distance: ${this.pathLengthMeters.toFixed(1)} m`;
    if (this.hudCoords) {
      const rx = (this.rover.x * this.mapData.resolution).toFixed(1);
      const ry = (this.rover.y * this.mapData.resolution).toFixed(1);
      this.hudCoords.textContent = `Rover: ${rx}m, ${ry}m`;
    }

    this.ctx.save();
    this.ctx.translate(this.offsetX, this.offsetY);
    this.ctx.scale(this.scale, this.scale);

    // 1. Draw stylized LiDAR map background image
    this.ctx.drawImage(this.mapImage, 0, 0);

    // 2. Draw Obstacle-avoiding guidance path
    if (this.currentPath && this.currentPath.length >= 2) {
      this.drawGuidancePath();
    }

    // 3. Draw Start Marker
    if (this.startPoint) {
      this.drawStartMarker(this.startPoint.x, this.startPoint.y);
    }

    // 4. Draw Goal / Target Marker
    if (this.goalPoint) {
      this.drawGoalMarker(this.goalPoint.x, this.goalPoint.y);
    }

    // 5. Draw Arrival Survey Zone (if target reached)
    if (this.surveyZone) {
      this.drawSurveyZone(this.surveyZone);
    }

    // 6. Draw Rover Pointer
    this.drawRoverPointer(this.rover.x, this.rover.y, this.rover.heading);

    this.ctx.restore();
  }

  drawGuidancePath() {
    const pts = this.currentPath;
    const now = Date.now();

    this.ctx.save();
    this.ctx.beginPath();
    this.ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) {
      this.ctx.lineTo(pts[i].x, pts[i].y);
    }
    this.ctx.lineWidth = 6 / this.scale;
    this.ctx.strokeStyle = "rgba(79, 216, 224, 0.4)";
    this.ctx.shadowColor = "#4fd8e0";
    this.ctx.shadowBlur = 14;
    this.ctx.lineCap = "round";
    this.ctx.lineJoin = "round";
    this.ctx.stroke();

    this.ctx.beginPath();
    this.ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) {
      this.ctx.lineTo(pts[i].x, pts[i].y);
    }
    this.ctx.lineWidth = 2.5 / this.scale;
    this.ctx.strokeStyle = "#e0fcff";
    this.ctx.shadowBlur = 0;
    this.ctx.setLineDash([8 / this.scale, 5 / this.scale]);
    this.ctx.lineDashOffset = -(now / 20) % (13 / this.scale);
    this.ctx.stroke();
    this.ctx.restore();
  }

  drawStartMarker(x, y) {
    const r = 7 / this.scale;
    this.ctx.save();
    this.ctx.beginPath();
    this.ctx.arc(x, y, r * 1.6, 0, Math.PI * 2);
    this.ctx.strokeStyle = "rgba(107, 255, 176, 0.4)";
    this.ctx.lineWidth = 2 / this.scale;
    this.ctx.stroke();

    this.ctx.beginPath();
    this.ctx.arc(x, y, r, 0, Math.PI * 2);
    this.ctx.fillStyle = "#6bffb0";
    this.ctx.shadowColor = "#6bffb0";
    this.ctx.shadowBlur = 10;
    this.ctx.fill();

    this.ctx.font = `bold ${Math.max(9, 11 / this.scale)}px sans-serif`;
    this.ctx.fillStyle = "#6bffb0";
    this.ctx.textAlign = "center";
    this.ctx.fillText("START", x, y - r - 4 / this.scale);
    this.ctx.restore();
  }

  drawGoalMarker(x, y) {
    const now = Date.now();
    const pulse = 1.0 + Math.sin(now / 250) * 0.15;
    const r = (9 * pulse) / this.scale;

    this.ctx.save();
    this.ctx.beginPath();
    this.ctx.arc(x, y, r * 1.5, 0, Math.PI * 2);
    this.ctx.strokeStyle = "rgba(255, 107, 107, 0.45)";
    this.ctx.lineWidth = 2 / this.scale;
    this.ctx.stroke();

    this.ctx.beginPath();
    this.ctx.arc(x, y, r, 0, Math.PI * 2);
    this.ctx.fillStyle = "#ff6b6b";
    this.ctx.shadowColor = "#ff6b6b";
    this.ctx.shadowBlur = 12;
    this.ctx.fill();

    this.ctx.beginPath();
    this.ctx.moveTo(x - r * 1.8, y);
    this.ctx.lineTo(x + r * 1.8, y);
    this.ctx.moveTo(x, y - r * 1.8);
    this.ctx.lineTo(x + r * 1.8, y);
    this.ctx.strokeStyle = "rgba(255, 255, 255, 0.85)";
    this.ctx.lineWidth = 1.5 / this.scale;
    this.ctx.stroke();

    this.ctx.font = `bold ${Math.max(9, 11 / this.scale)}px sans-serif`;
    this.ctx.fillStyle = "#ff8c8c";
    this.ctx.textAlign = "center";
    this.ctx.fillText("TARGET", x, y - r * 1.5 - 4 / this.scale);
    this.ctx.restore();
  }

  drawSurveyZone(zone) {
    if (!zone) return;
    const now = Date.now();
    const elapsed = (now - zone.timestamp) / 1000;
    const pulse = 1.0 + Math.sin(now / 200) * 0.08;
    const r = (zone.radius * pulse) / this.scale;
    const x = zone.x;
    const y = zone.y;

    this.ctx.save();

    // 1. Concentric ripple waves
    for (let i = 1; i <= 3; i++) {
      const wavePhase = (elapsed * 1.2 + i * 0.33) % 1.0;
      const waveR = (zone.radius * (1.0 + wavePhase * 1.5)) / this.scale;
      const alpha = Math.max(0, 0.5 * (1.0 - wavePhase));
      this.ctx.beginPath();
      this.ctx.arc(x, y, waveR, 0, Math.PI * 2);
      this.ctx.strokeStyle = zone.color;
      this.ctx.globalAlpha = alpha;
      this.ctx.lineWidth = 2.0 / this.scale;
      this.ctx.stroke();
    }
    this.ctx.globalAlpha = 1.0;

    // 2. Broad environmental gradient fill
    const grad = this.ctx.createRadialGradient(x, y, 0, x, y, r * 1.2);
    grad.addColorStop(0, zone.fill);
    grad.addColorStop(0.7, zone.fill);
    grad.addColorStop(1, "rgba(0,0,0,0)");
    this.ctx.beginPath();
    this.ctx.arc(x, y, r * 1.2, 0, Math.PI * 2);
    this.ctx.fillStyle = grad;
    this.ctx.shadowColor = zone.color;
    this.ctx.shadowBlur = 24;
    this.ctx.fill();

    // 3. Prominent curved boundary border with glowing dash
    this.ctx.beginPath();
    this.ctx.arc(x, y, r, 0, Math.PI * 2);
    this.ctx.strokeStyle = zone.color;
    this.ctx.lineWidth = 3.0 / this.scale;
    this.ctx.setLineDash([8 / this.scale, 5 / this.scale]);
    this.ctx.lineDashOffset = -now / 35;
    this.ctx.stroke();
    this.ctx.setLineDash([]);

    // 4. Rotating radar scanner arc
    const sweepAngle = (elapsed * 2.5) % (Math.PI * 2);
    this.ctx.beginPath();
    this.ctx.moveTo(x, y);
    this.ctx.arc(x, y, r, sweepAngle - Math.PI / 5, sweepAngle);
    this.ctx.closePath();
    this.ctx.fillStyle = zone.color;
    this.ctx.globalAlpha = 0.40;
    this.ctx.fill();
    this.ctx.globalAlpha = 1.0;

    // 5. Central glowing beacon
    this.ctx.beginPath();
    this.ctx.arc(x, y, 6 / this.scale, 0, Math.PI * 2);
    this.ctx.fillStyle = "#ffffff";
    this.ctx.shadowColor = zone.color;
    this.ctx.shadowBlur = 14;
    this.ctx.fill();

    // 6. Floating two-line curved badge directly above the survey zone
    const line1 = `📍 SURVEY ZONE: ${zone.label}`;
    const line2 = `Gas: ${zone.gas} ppm  |  Temp: ${Number(zone.temp).toFixed(1)}°C  |  ${zone.gasStatus ? 'ALARM' : 'NORMAL'}`;

    const fontSize1 = Math.max(10, 11.5 / this.scale);
    const fontSize2 = Math.max(9, 10 / this.scale);
    this.ctx.font = `bold ${fontSize1}px sans-serif`;
    const w1 = this.ctx.measureText(line1).width;
    this.ctx.font = `${fontSize2}px monospace`;
    const w2 = this.ctx.measureText(line2).width;

    const badgeW = Math.max(w1, w2) + 20 / this.scale;
    const badgeH = 34 / this.scale;
    const badgeX = x - badgeW / 2;
    const badgeY = y - r - 12 / this.scale - badgeH;

    // Badge background box
    this.ctx.beginPath();
    this.ctx.roundRect(badgeX, badgeY, badgeW, badgeH, 8 / this.scale);
    this.ctx.fillStyle = "rgba(6, 12, 20, 0.94)";
    this.ctx.strokeStyle = zone.color;
    this.ctx.lineWidth = 1.8 / this.scale;
    this.ctx.shadowColor = zone.color;
    this.ctx.shadowBlur = 16;
    this.ctx.fill();
    this.ctx.stroke();

    // Line 1: Title
    this.ctx.font = `bold ${fontSize1}px sans-serif`;
    this.ctx.fillStyle = zone.color;
    this.ctx.textAlign = "center";
    this.ctx.textBaseline = "top";
    this.ctx.fillText(line1, x, badgeY + 5 / this.scale);

    // Line 2: Telemetry values
    this.ctx.font = `${fontSize2}px monospace`;
    this.ctx.fillStyle = "#e0e6ed";
    this.ctx.fillText(line2, x, badgeY + 18 / this.scale);

    this.ctx.restore();
  }

  drawRoverPointer(x, y, heading) {
    const size = 11 / this.scale;
    this.ctx.save();
    this.ctx.translate(x, y);
    this.ctx.rotate(heading);

    const grad = this.ctx.createRadialGradient(size, 0, 2, size * 2.8, 0, size * 3.5);
    grad.addColorStop(0, "rgba(79, 216, 224, 0.45)");
    grad.addColorStop(1, "rgba(79, 216, 224, 0.0)");
    this.ctx.beginPath();
    this.ctx.moveTo(size * 0.8, 0);
    this.ctx.arc(size * 0.8, 0, size * 3.5, -Math.PI / 5, Math.PI / 5);
    this.ctx.closePath();
    this.ctx.fillStyle = grad;
    this.ctx.fill();

    this.ctx.beginPath();
    this.ctx.moveTo(size * 1.4, 0);
    this.ctx.lineTo(-size * 0.9, -size * 0.8);
    this.ctx.lineTo(-size * 0.4, 0);
    this.ctx.lineTo(-size * 0.9, size * 0.8);
    this.ctx.closePath();

    this.ctx.fillStyle = "#4fd8e0";
    this.ctx.shadowColor = "#4fd8e0";
    this.ctx.shadowBlur = 14;
    this.ctx.fill();

    this.ctx.strokeStyle = "#ffffff";
    this.ctx.lineWidth = 1.5 / this.scale;
    this.ctx.stroke();

    this.ctx.beginPath();
    this.ctx.arc(0, 0, size * 0.3, 0, Math.PI * 2);
    this.ctx.fillStyle = "#ffffff";
    this.ctx.fill();

    this.ctx.restore();
  }
}
class BinaryMinHeap {
  constructor(compare) {
    this.data = [];
    this.compare = compare || ((a, b) => a - b);
  }
  size() { return this.data.length; }
  push(item) {
    this.data.push(item);
    this.bubbleUp(this.data.length - 1);
  }
  pop() {
    if (this.data.length === 0) return null;
    const top = this.data[0];
    const bottom = this.data.pop();
    if (this.data.length > 0) {
      this.data[0] = bottom;
      this.sinkDown(0);
    }
    return top;
  }
  bubbleUp(n) {
    const element = this.data[n];
    while (n > 0) {
      const parentN = Math.floor((n + 1) / 2) - 1;
      const parent = this.data[parentN];
      if (this.compare(element, parent) >= 0) break;
      this.data[parentN] = element;
      this.data[n] = parent;
      n = parentN;
    }
  }
  sinkDown(n) {
    const length = this.data.length;
    const element = this.data[n];
    while (true) {
      const child2N = (n + 1) * 2;
      const child1N = child2N - 1;
      let swap = null;
      if (child1N < length) {
        const child1 = this.data[child1N];
        if (this.compare(child1, element) < 0) swap = child1N;
      }
      if (child2N < length) {
        const child2 = this.data[child2N];
        if (this.compare(child2, swap === null ? element : this.data[child1N]) < 0) swap = child2N;
      }
      if (swap === null) break;
      this.data[n] = this.data[swap];
      this.data[swap] = element;
      n = swap;
    }
  }
}

let lidarMapManager = null;
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", () => {
    lidarMapManager = new LidarMapManager();
  });
} else {
  lidarMapManager = new LidarMapManager();
}
