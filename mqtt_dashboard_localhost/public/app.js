const els = {
  mqttBadge: document.getElementById("mqttBadge"),
  configInfo: document.getElementById("configInfo"),
  readinessInfo: document.getElementById("readinessInfo"),
  sessionStatus: document.getElementById("sessionStatus"),
  summaryInfo: document.getElementById("summaryInfo"),
  nodesTableBody: document.querySelector("#nodesTable tbody"),
  routeTableBody: document.querySelector("#routeTable tbody"),
  paramTableBody: document.querySelector("#paramTable tbody"),
  rangeTableBody: document.querySelector("#rangeTable tbody"),
  allMqttTableBody: document.querySelector("#allMqttTable tbody"),
  startBtn: document.getElementById("startBtn"),
  stopBtn: document.getElementById("stopBtn"),
  resetBtn: document.getElementById("resetBtn"),
  fillDefaultBtn: document.getElementById("fillDefaultBtn"),
  autoLabelBtn: document.getElementById("autoLabelBtn"),
  copyLastBtn: document.getElementById("copyLastBtn"),
  navButtons: Array.from(document.querySelectorAll(".nav-btn[data-page]")),
  pages: Array.from(document.querySelectorAll(".page[data-page]")),
  form: {
    label: document.getElementById("label"),
    testType: document.getElementById("testType"),
    payloadProfile: document.getElementById("payloadProfile"),
    sf: document.getElementById("sf"),
    bwKHz: document.getElementById("bwKHz"),
    cr: document.getElementById("cr"),
    distanceM: document.getElementById("distanceM"),
    trialDurationMin: document.getElementById("trialDurationMin"),
    iterasi: document.getElementById("iterasi")
  }
};

let appConfig = null;
let lastState = null;
let refreshInFlight = false;
let activePage = "overview";
let historyEventsCache = [];
let historyFetchInFlight = false;
const FORM_CACHE_KEY = "loraDashboardFormDraftV1";

const readinessLabelMap = {
  mqtt_connected: "MQTT",
  packet_flow_recent: "Aliran Paket",
  route_diag_recent: "Route Diagnostic",
  gateway_status_recent: "Status Gateway",
  latency_signal_recent: "Sinyal Latensi"
};

function fmtTime(ts) {
  if (!ts) {
    return "-";
  }
  // Format waktu Makassar (WITA)
  return new Intl.DateTimeFormat("id-ID", {
    dateStyle: "short",
    timeStyle: "medium",
    timeZone: "Asia/Makassar",
  }).format(new Date(ts));
}

function fmtNum(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  return Number(value).toFixed(digits);
}

function fmtDurationMs(ms) {
  if (!Number.isFinite(Number(ms)) || Number(ms) < 0) {
    return "-";
  }
  const totalSec = Math.floor(Number(ms) / 1000);
  const mm = Math.floor(totalSec / 60);
  const ss = totalSec % 60;
  return `${String(mm).padStart(2, "0")}:${String(ss).padStart(2, "0")}`;
}

function fmtDurationMinFromMs(ms) {
  if (!Number.isFinite(Number(ms)) || Number(ms) <= 0) {
    return "-";
  }
  return String(Math.max(1, Math.round(Number(ms) / 60000)));
}

function fmtRouteStampLow32(value) {
  if (value === null || value === undefined) {
    return "-";
  }
  const n = Number(value);
  if (!Number.isFinite(n) || n <= 0) {
    return "-";
  }
  return String(Math.trunc(n));
}

function computeTrialLatencyAvg(summary) {
  const nodes = Array.isArray(summary?.nodes) ? summary.nodes : [];
  let sum = 0;
  let count = 0;
  for (const row of nodes) {
    if (row.latencyAvg !== null && row.latencyAvg !== undefined && Number.isFinite(Number(row.latencyAvg))) {
      sum += Number(row.latencyAvg);
      count += 1;
    }
  }
  if (count === 0) {
    return null;
  }
  return sum / count;
}

function computeTrialHopAvg(summary) {
  const nodes = Array.isArray(summary?.nodes) ? summary.nodes : [];
  let sum = 0;
  let count = 0;
  for (const row of nodes) {
    if (row.hopAvg !== null && row.hopAvg !== undefined && Number.isFinite(Number(row.hopAvg))) {
      sum += Number(row.hopAvg);
      count += 1;
    }
  }
  if (count === 0) {
    return null;
  }
  return sum / count;
}

function esc(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

function formatDateForLabel(date) {
  const yyyy = date.getFullYear();
  const mm = String(date.getMonth() + 1).padStart(2, "0");
  const dd = String(date.getDate()).padStart(2, "0");
  const hh = String(date.getHours()).padStart(2, "0");
  const mi = String(date.getMinutes()).padStart(2, "0");
  const ss = String(date.getSeconds()).padStart(2, "0");
  return `${yyyy}${mm}${dd}-${hh}${mi}${ss}`;
}

function cleanLabelToken(value) {
  return String(value || "")
    .trim()
    .replace(/\s+/g, "")
    .replace(/\//g, "-")
    .replace(/[^A-Za-z0-9._-]/g, "");
}

function buildAutoLabel() {
  const scenario = cleanLabelToken(els.form.testType.value);
  const profile = cleanLabelToken(els.form.payloadProfile.value);
  const sf = cleanLabelToken(els.form.sf.value) || "X";
  const bw = cleanLabelToken(els.form.bwKHz.value) || "X";
  const cr = cleanLabelToken(els.form.cr.value) || "X";
  const dist = cleanLabelToken(els.form.distanceM.value) || "X";
  const dur = cleanLabelToken(els.form.trialDurationMin.value) || "X";
  const stamp = formatDateForLabel(new Date());
  const tokens = [];

  if (scenario) {
    tokens.push(scenario);
  }
  if (profile) {
    tokens.push(profile);
  }
  tokens.push(`SF${sf}`, `BW${bw}`, `CR${cr}`, `D${dist}`, `T${dur}m`, stamp);
  return tokens.join("_");
}

function writeFormDraftToCache() {
  const draft = {
    label: els.form.label.value,
    testType: els.form.testType.value,
    payloadProfile: els.form.payloadProfile.value,
    sf: els.form.sf.value,
    bwKHz: els.form.bwKHz.value,
    cr: els.form.cr.value,
    distanceM: els.form.distanceM.value,
    trialDurationMin: els.form.trialDurationMin.value
  };
  try {
    localStorage.setItem(FORM_CACHE_KEY, JSON.stringify(draft));
  } catch (error) {
    console.error(error);
  }
}

function loadFormDraftFromCache() {
  try {
    const raw = localStorage.getItem(FORM_CACHE_KEY);
    if (!raw) {
      return;
    }
    const draft = JSON.parse(raw);
    if (!draft || typeof draft !== "object") {
      return;
    }
    els.form.label.value = draft.label || "";
    els.form.testType.value = draft.testType || "";
    els.form.payloadProfile.value = draft.payloadProfile || "";
    els.form.sf.value = draft.sf || "";
    els.form.bwKHz.value = draft.bwKHz || "";
    els.form.cr.value = draft.cr || "";
    els.form.distanceM.value = draft.distanceM || "";
    els.form.trialDurationMin.value = draft.trialDurationMin || "5";
  } catch (error) {
    console.error(error);
  }
}

async function getJson(url, options = {}) {
  const response = await fetch(url, options);
  if (!response.ok) {
    const text = await response.text();
    throw new Error(text || `HTTP ${response.status}`);
  }
  return response.json();
}

async function postJson(url, body) {
  return getJson(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {})
  });
}

async function refreshHistoryEvents() {
  if (historyFetchInFlight) {
    return;
  }
  historyFetchInFlight = true;
  try {
    const payload = await getJson("/api/history/all_mqtt.json");
    historyEventsCache = Array.isArray(payload?.events) ? payload.events : [];
    if (lastState && activePage === "history") {
      renderAllMqttTable(lastState);
    }
  } catch (error) {
    console.error(error);
  } finally {
    historyFetchInFlight = false;
  }
}

function renderConfig(config) {
  if (!config) {
    els.configInfo.textContent = "Loading...";
    return;
  }
  els.configInfo.innerHTML = [
    `MQTT URI: ${esc(config.mqttUri)}`,
    `Data Prefix: ${esc(config.topicDataPrefix)}`,
    `Route Topic: ${esc(config.topicDiagnostic)}`,
    `Status Topic: ${esc(config.topicStatus)}`,
    `Interval default node: ${esc(config.nodeSendIntervalMs)} ms`,
    `Durasi trial default: ${esc(Math.round(Number(config.manualDefaultDurationMs || 300000) / 60000))} menit`
  ].join("<br>");
}

function renderMqttBadge(mqtt) {
  const connected = Boolean(mqtt?.connected);
  els.mqttBadge.textContent = connected ? "MQTT: ONLINE" : "MQTT: OFFLINE";
  els.mqttBadge.classList.toggle("online", connected);
  els.mqttBadge.classList.toggle("offline", !connected);
}

function renderReadiness(state) {
  const readiness = state.readiness;
  if (!readiness || !readiness.checks) {
    els.readinessInfo.innerHTML = "Readiness belum tersedia.";
    return;
  }

  const rows = Object.entries(readiness.checks).map(([key, check]) => {
    const statusClass = check.ok ? "ok" : "fail";
    const statusText = check.ok ? "OK" : "WARN";
    const label = readinessLabelMap[key] || key;
    return `<div><span class="${statusClass}">[${statusText}]</span> ${esc(label)}: ${esc(check.detail)}</div>`;
  });
  const startHint = readiness.canStartTrial
    ? "<div class=\"ok\">Trial bisa dimulai (MQTT connected).</div>"
    : "<div class=\"fail\">Trial belum siap (MQTT belum connected).</div>";
  els.readinessInfo.innerHTML = `${startHint}${rows.join("")}`;
}

function renderSummary(state) {
  const active = state.activeSession;
  const sessions = state.sessions || [];
  const latest = active || sessions[sessions.length - 1] || null;
  const totalMqttRows = Number(state.allMqttCount || 0);

  if (!latest) {
    els.summaryInfo.innerHTML = "<span class=\"small\">Belum ada trial.</span>";
    els.sessionStatus.textContent = `Trial aktif: tidak ada | Data MQTT tercatat: ${totalMqttRows}`;
    return;
  }

  const activeText = active
    ? `Trial aktif: ${active.label} (mulai ${fmtTime(active.startedAt)})`
    : "Trial aktif: tidak ada";
  els.sessionStatus.textContent = `${activeText} | Data MQTT tercatat: ${totalMqttRows}`;

  const targetDurationMs = Number(latest.targetDurationMs || 0);
  const hasTarget = Number.isFinite(targetDurationMs) && targetDurationMs > 0;
  const elapsedMs = latest.durationMs || 0;
  const remainingMs = hasTarget ? Math.max(0, targetDurationMs - elapsedMs) : null;

  els.summaryInfo.innerHTML = `
    <div class="small">
      ID: <b>${esc(latest.id)}</b><br>
      Label: <b>${esc(latest.label)}</b><br>
      SF/BW/CR: <b>${esc(latest.sf || "-")} / ${esc(latest.bwKHz || "-")} / ${esc(latest.cr || "-")}</b><br>
      Jarak: <b>${esc(latest.distanceM || "-")} m</b><br>
      Durasi Target: <b>${hasTarget ? fmtDurationMs(targetDurationMs) : "-"}</b><br>
      Sisa Waktu: <b>${active && hasTarget ? fmtDurationMs(remainingMs) : "-"}</b><br>
      Durasi: <b>${fmtNum((latest.durationMs || 0) / 1000, 2)} s</b><br>
      Total RX/Expected: <b>${esc(latest.total?.received ?? 0)} / ${esc(latest.total?.expected ?? 0)}</b><br>
      PDR/PLR Total: <b>${fmtNum(latest.total?.pdr)}% / ${fmtNum(latest.total?.plr)}%</b><br>
      Route OK/Fail: <b>${esc(latest.route?.success ?? 0)} / ${esc(latest.route?.fail ?? 0)}</b>
    </div>
  `;
}

function renderNodesTable(state) {
  const sessions = state.sessions || [];
  const allSessions = sessions.slice().reverse();
  let html = "";

  for (const session of allSessions) {
    const nodes = session.nodes || [];
    if (nodes.length > 0) {
      html += nodes.map(row => `
      <tr>
        <td>${esc(session.iterasi || "-")}</td>
        <td>${esc(row.node)}</td>
        <td>${esc(row.received || 0)} / ${esc(row.expected || 0)}</td>
        <td>${fmtNum(row.pdr)}</td>
        <td>${fmtNum(row.plr)}</td>
        <td>${fmtNum(row.latencyAvg)} <span class="small">(${esc(row.expectedMethod === "sequence" ? "seq" : "time")})</span></td>
      </tr>
      `).join("");
    }
  }

  if (!html) {
    els.nodesTableBody.innerHTML = "<tr><td colspan=\"6\" class=\"small\">Belum ada data node.</td></tr>";
    return;
  }

  els.nodesTableBody.innerHTML = html;
}

function renderRouteTable(state) {
  const rows = (state.routeEvents || []).slice().reverse();
  if (rows.length === 0) {
    els.routeTableBody.innerHTML = "<tr><td colspan=\"10\" class=\"small\">Belum ada route diagnostic.</td></tr>";
    return;
  }

  els.routeTableBody.innerHTML = rows
    .map((row) => {
      const trial = row.trial || {};
      const rowPath = (row.payload && Array.isArray(row.payload.route_path))
        ? row.payload.route_path
        : null;
      const rp = getRoutePathInfo(state, row.node, rowPath);
      // Gunakan nilai asli dari backend (hopCount)
      const hopDisplay = row.hops ?? "-";
      return `
      <tr>
        <td>${esc(fmtTime(row.ts))}</td>
        <td>${esc(row.node || "-")}</td>
        <td>${esc(row.target ?? "-")}</td>
        <td>${esc(String(hopDisplay))}</td>
        <td class="route-path">${rp.html}</td>
        <td>${esc(fmtRouteStampLow32(row.rreqAt))}</td>
        <td>${esc(fmtRouteStampLow32(row.rrepAt))}</td>
        <td>${esc(row.discoveryMs ?? "-")}</td>
        <td>${esc(row.retries ?? "-")}</td>
        <td class="${row.success ? "ok" : "fail"}">${row.success ? "SUCCESS" : "FAILED"}</td>
      </tr>
    `
    })
    .join("");
}

function routePathInfoFromArray(path) {
  const cleanPath = Array.isArray(path)
    ? path.filter((item) => item !== null && item !== undefined && String(item).trim() !== "").map((item) => String(item).trim())
    : [];
  if (cleanPath.length === 0) {
    return { html: '<span class="small">-</span>', hops: null, path: [] };
  }
  const html = cleanPath.map((n) => `<span class="route-node">${esc(n)}</span>`).join(" -> ");
  return { html, hops: cleanPath.length - 1, path: cleanPath };
}

function getRoutePathInfo(state, nodeName, preferredPath = null) {
  if (Array.isArray(preferredPath) && preferredPath.length > 0) {
    return routePathInfoFromArray(preferredPath);
  }
  // Cari event terakhir dari node ini yang memiliki route_path
  const latestByNode = state.latestByNode || {};
  const nodeEvent = latestByNode[nodeName];
  if (nodeEvent && nodeEvent.payload && Array.isArray(nodeEvent.payload.route_path)) {
    return routePathInfoFromArray(nodeEvent.payload.route_path);
  }
  return { html: '<span class="small">-</span>', hops: null, path: [] };
}

function renderParameterTable(state) {
  const sessions = state.sessions || [];
  const rows = sessions.slice().reverse().slice(0, 300);

  if (rows.length === 0) {
    els.paramTableBody.innerHTML = "<tr><td colspan=\"9\" class=\"small\">Belum ada data trial parameter.</td></tr>";
    return;
  }

  els.paramTableBody.innerHTML = rows
    .map((row) => {
      const latencyAvg = computeTrialLatencyAvg(row);
      return `
      <tr>
        <td>${esc(row.sf || "-")}</td>
        <td>${esc(row.bwKHz || "-")}</td>
        <td>${esc(row.cr || "-")}</td>
        <td>${esc(row.distanceM || "-")}</td>
        <td>${fmtNum(row.total?.expected, 0)}</td>
        <td>${fmtNum(row.total?.received, 0)}</td>
        <td>${fmtNum(row.total?.pdr)}</td>
        <td>${fmtNum(row.total?.plr)}</td>
        <td>${fmtNum(latencyAvg)}</td>
      </tr>
    `;
    })
    .join("");
}

function renderRangeTable(state) {
  const sessions = state.sessions || [];
  const allRows = sessions.slice().reverse();
  const rows = allRows
    .filter((row) => String(row.distanceM || "").trim() || String(row.expectedHop || "").trim())
    .slice(0, 300);

  if (rows.length === 0) {
    // Jika belum ada trial tapi sudah ada data node, tampilkan berdasarkan latestByNode
    const latestByNode = state.latestByNode || {};
    const nodeNames = Object.keys(latestByNode).filter(n => n !== "GATEWAY");
    if (nodeNames.length === 0) {
      els.rangeTableBody.innerHTML = "<tr><td colspan=\"6\" class=\"small\">Belum ada data jangkauan.</td></tr>";
      return;
    }
    els.rangeTableBody.innerHTML = nodeNames.map(nodeName => {
      const evt = latestByNode[nodeName];
      const rp = getRoutePathInfo(state, nodeName);
      // Hop otomatis dari jalur route
      const hops = evt?.hops ?? "-";
      return `
      <tr>
        <td>${esc(nodeName)}</td>
        <td>-</td>
        <td>${esc(String(hops))}</td>
        <td class="route-path">${rp.html}</td>
        <td>-</td>
        <td class="ok">AKTIF</td>
      </tr>`;
    }).join("");
    return;
  }

  els.rangeTableBody.innerHTML = rows
    .map((row) => {
      const nodeList = row.nodes || [];
      if (nodeList.length === 0) {
        return `
        <tr>
          <td>-</td>
          <td>${esc(row.distanceM || "-")}</td>
          <td>-</td>
          <td class="route-path">-</td>
          <td>-</td>
          <td class="fail">KOSONG</td>
        </tr>`;
      }

      return nodeList.map(n => {
        const success = Number(n.received || 0) > 0;
        const rp = getRoutePathInfo(state, n.node);
        const hops = row.expectedHop || "-";
        const routePathHtml = rp.html || "-";

        return `
        <tr>
          <td>${esc(n.node)}</td>
          <td>${esc(row.distanceM || "-")}</td>
          <td>${esc(String(hops))}</td>
          <td class="route-path">${routePathHtml}</td>
          <td>${fmtNum(n.pdr)}</td>
          <td class="${success ? "ok" : "fail"}">${success ? "BERHASIL" : "GAGAL"}</td>
        </tr>
        `;
      }).join("");
    })
    .join("");
}

function renderAllMqttTable(state) {
  const rows = historyEventsCache.slice().reverse();
  if (rows.length === 0) {
    els.allMqttTableBody.innerHTML = "<tr><td colspan=\"19\" class=\"small\">Belum ada data MQTT.</td></tr>";
    return;
  }

  els.allMqttTableBody.innerHTML = rows
    .map((row) => {
      const trial = row.trial || {};
      const successText = row.success === true ? "SUCCESS" : row.success === false ? "FAILED" : "-";
      const routePathArr = (row.payload && Array.isArray(row.payload.route_path))
        ? row.payload.route_path : [];
      const routePath = routePathArr.length > 0 ? routePathArr.join(" -> ") : "-";
      // Gunakan nilai asli dari backend (hopCount)
      const hopDisplay = row.hops ?? "-";
      const delayDisplay = row.delayMs ?? row.latencyMs ?? "-";
      return `
      <tr>
        <td>${esc(fmtTime(row.ts))}</td>
        <td>${esc(row.kind || "-")}</td>
        <td>${esc(row.topic || "-")}</td>
        <td>${esc(row.node || "-")}</td>
        <td>${esc(trial.trialCode || "-")}</td>
        <td>${esc(trial.label || "-")}</td>
        <td>${esc(trial.sf || "-")}</td>
        <td>${esc(trial.bwKHz || "-")}</td>
        <td>${esc(trial.cr || "-")}</td>
        <td>${esc(trial.distanceM || "-")}</td>
        <td>${esc(fmtDurationMinFromMs(trial.targetDurationMs))}</td>
        <td>${esc(String(hopDisplay))}</td>
        <td class="route-path">${esc(routePath)}</td>
        <td>${esc(delayDisplay)}</td>
        <td>${esc(fmtRouteStampLow32(row.rreqAt))}</td>
        <td>${esc(fmtRouteStampLow32(row.rrepAt))}</td>
        <td>${esc(row.discoveryMs ?? "-")}</td>
        <td>${esc(row.retries ?? "-")}</td>
        <td class="${row.success === true ? "ok" : row.success === false ? "fail" : ""}">${esc(successText)}</td>
        <td>${esc(row.rssi ?? "-")}</td>
      </tr>
    `;
    })
    .join("");
}

function setActivePage(pageName, syncHash = true) {
  const availablePages = new Set(els.pages.map((page) => page.dataset.page));
  const next = availablePages.has(pageName) ? pageName : "overview";
  activePage = next;

  for (const button of els.navButtons) {
    button.classList.toggle("active", button.dataset.page === next);
  }

  for (const page of els.pages) {
    page.classList.toggle("active", page.dataset.page === next);
  }

  if (syncHash) {
    const hash = `#${next}`;
    if (window.location.hash !== hash) {
      window.location.hash = hash;
    }
  }

  if (lastState) {
    renderState(lastState);
  }
  if (next === "history") {
    refreshHistoryEvents();
  }
}

function resolvePageFromHash() {
  const hash = String(window.location.hash || "").replace("#", "").trim();
  return hash || "overview";
}

function setupNavigation() {
  if (!els.navButtons.length || !els.pages.length) {
    return;
  }

  for (const button of els.navButtons) {
    button.addEventListener("click", () => {
      const targetPage = button.dataset.page || "overview";
      setActivePage(targetPage, true);
    });
  }

  window.addEventListener("hashchange", () => {
    setActivePage(resolvePageFromHash(), false);
  });

  setActivePage(resolvePageFromHash(), false);
}

function renderState(state) {
  lastState = state;
  renderMqttBadge(state.mqtt);
  renderReadiness(state);
  renderSummary(state);
  const hasActiveTrial = Boolean(state.activeSession);
  els.startBtn.disabled = hasActiveTrial;
  els.stopBtn.disabled = !hasActiveTrial;
  if (activePage === "monitoring") {
    renderNodesTable(state);
    renderRouteTable(state);
    renderParameterTable(state);
    renderRangeTable(state);
  }
  if (activePage === "history") {
    renderAllMqttTable(state);
  }
}

async function refreshState() {
  if (refreshInFlight) {
    return;
  }
  refreshInFlight = true;
  try {
    const state = await getJson("/api/state");
    renderState(state);
    if (activePage === "history") {
      await refreshHistoryEvents();
    }
  } catch (error) {
    console.error(error);
  } finally {
    refreshInFlight = false;
  }
}

function sessionPayloadFromForm() {
  return {
    label: els.form.label.value.trim(),
    sf: els.form.sf.value.trim(),
    bwKHz: els.form.bwKHz.value.trim(),
    cr: els.form.cr.value.trim(),
    expectedHop: "",
    distanceM: els.form.distanceM.value.trim(),
    durationMinutes: els.form.trialDurationMin.value.trim(),
    iterasi: els.form.iterasi.value.trim()
  };
}

function validateTrialPayload(payload) {
  const required = ["sf", "bwKHz", "cr", "distanceM"];
  for (const key of required) {
    if (!payload[key]) {
      return `Field ${key} wajib diisi.`;
    }
  }
  const durationMinutes = Number(payload.durationMinutes);
  if (!Number.isFinite(durationMinutes) || durationMinutes <= 0) {
    return "Field durationMinutes harus angka > 0.";
  }
  return "";
}

async function onStartClick() {
  const payload = sessionPayloadFromForm();
  if (!payload.label) {
    payload.label = buildAutoLabel();
    els.form.label.value = payload.label;
  }
  const validationError = validateTrialPayload(payload);
  if (validationError) {
    alert(validationError);
    return;
  }

  try {
    await postJson("/api/session/start", payload);
    writeFormDraftToCache();
    await refreshState();
  } catch (error) {
    alert(`Gagal start trial: ${error.message}`);
  }
}

async function onStopClick() {
  try {
    await postJson("/api/session/stop", {});
    await refreshState();
  } catch (error) {
    alert(`Gagal stop trial: ${error.message}`);
  }
}

async function onResetClick() {
  const yes = confirm("Reset semua data MQTT monitor, route, dan riwayat trial?");
  if (!yes) {
    return;
  }
  try {
    await postJson("/api/reset", {});
    await refreshState();
  } catch (error) {
    alert(`Gagal reset: ${error.message}`);
  }
}

function onFillDefaultClick() {
  els.form.sf.value = "9";
  els.form.bwKHz.value = "125";
  els.form.cr.value = "4/5";
  els.form.distanceM.value = "200";
  els.form.trialDurationMin.value = "5";
  if (!els.form.testType.value) {
    els.form.testType.value = "S3_QOS";
  }
  writeFormDraftToCache();
}

function onAutoLabelClick() {
  els.form.label.value = buildAutoLabel();
  writeFormDraftToCache();
}

function onCopyLastClick() {
  const sessions = Array.isArray(lastState?.sessions) ? lastState.sessions : [];
  const latest = sessions.length > 0 ? sessions[sessions.length - 1] : null;
  if (!latest) {
    alert("Belum ada trial sebelumnya untuk disalin.");
    return;
  }
  els.form.label.value = latest.label || "";
  els.form.sf.value = latest.sf || "";
  els.form.bwKHz.value = latest.bwKHz || "";
    els.form.cr.value = latest.cr || "";
    els.form.distanceM.value = latest.distanceM || "";
    if (latest.targetDurationMs) {
      els.form.trialDurationMin.value = String(Math.max(1, Math.round(Number(latest.targetDurationMs) / 60000)));
    }
  writeFormDraftToCache();
}

function setupFormDraftSync() {
  const inputs = [
    els.form.label,
    els.form.testType,
    els.form.payloadProfile,
    els.form.sf,
    els.form.bwKHz,
    els.form.cr,
    els.form.distanceM,
    els.form.trialDurationMin
  ];
  for (const input of inputs) {
    input.addEventListener("change", writeFormDraftToCache);
    input.addEventListener("input", writeFormDraftToCache);
  }
}

function setupSocket() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${scheme}://${window.location.host}`);
  ws.addEventListener("message", () => {
    refreshState();
  });
  ws.addEventListener("close", () => {
    setTimeout(setupSocket, 2000);
  });
}

async function init() {
  try {
    appConfig = await getJson("/api/config");
    renderConfig(appConfig);
  } catch (error) {
    console.error(error);
    els.configInfo.textContent = `Gagal load config: ${error.message}`;
  }

  await refreshState();
  loadFormDraftFromCache();
  setupFormDraftSync();
  setupNavigation();
  setupSocket();
  setInterval(refreshState, 5000);
  setInterval(() => {
    if (lastState) {
      renderSummary(lastState);
    }
  }, 1000);

  els.startBtn.addEventListener("click", onStartClick);
  els.stopBtn.addEventListener("click", onStopClick);
  els.resetBtn.addEventListener("click", onResetClick);
  els.fillDefaultBtn.addEventListener("click", onFillDefaultClick);
  els.autoLabelBtn.addEventListener("click", onAutoLabelClick);
  els.copyLastBtn.addEventListener("click", onCopyLastClick);
}

init();


