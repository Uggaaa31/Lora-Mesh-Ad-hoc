require("dotenv").config();

const express = require("express");
const http = require("http");
const path = require("path");
const mqtt = require("mqtt");
const { WebSocketServer } = require("ws");

const PORT = Number.parseInt(process.env.PORT || "3001", 10);
const MAX_EVENTS = Number.parseInt(process.env.MAX_EVENTS || "20000", 10);
const MAX_ROUTE_EVENTS = Number.parseInt(process.env.MAX_ROUTE_EVENTS || "20000", 10);
const MAX_STATUS_EVENTS = Number.parseInt(process.env.MAX_STATUS_EVENTS || "5000", 10);
const MAX_SESSIONS = Number.parseInt(process.env.MAX_SESSIONS || "200", 10);
const READINESS_DATA_MAX_AGE_MS = Number.parseInt(process.env.READINESS_DATA_MAX_AGE_MS || "120000", 10);
const READINESS_DIAG_MAX_AGE_MS = Number.parseInt(process.env.READINESS_DIAG_MAX_AGE_MS || "300000", 10);
const READINESS_STATUS_MAX_AGE_MS = Number.parseInt(process.env.READINESS_STATUS_MAX_AGE_MS || "360000", 10);
const AUTO_DEFAULT_DURATION_MS = Number.parseInt(process.env.AUTO_DEFAULT_DURATION_MS || "300000", 10);
const AUTO_DEFAULT_REPEATS = Number.parseInt(process.env.AUTO_DEFAULT_REPEATS || "3", 10);
const AUTO_DEFAULT_COOLDOWN_MS = Number.parseInt(process.env.AUTO_DEFAULT_COOLDOWN_MS || "3000", 10);
const AUTO_DEFAULT_INACTIVITY_TIMEOUT_SEC = Number.parseInt(
  process.env.AUTO_DEFAULT_INACTIVITY_TIMEOUT_SEC || "30",
  10
);
const AUTO_DEFAULT_MAX_ROUTE_FAIL = Number.parseInt(process.env.AUTO_DEFAULT_MAX_ROUTE_FAIL || "3", 10);
const MANUAL_DEFAULT_DURATION_MS = Number.parseInt(process.env.MANUAL_DEFAULT_DURATION_MS || "300000", 10);
const CSV_DELIMITER = process.env.CSV_DELIMITER || ";";
const CSV_INCLUDE_BOM = process.env.CSV_INCLUDE_BOM !== "false";

const config = {
  mqttUri: process.env.MQTT_URI || "wss://mqtt.aistrack.site:443/",
  mqttRejectUnauthorized: process.env.MQTT_REJECT_UNAUTHORIZED !== "false",
  mqttUsername: process.env.MQTT_USERNAME || "",
  mqttPassword: process.env.MQTT_PASSWORD || "",
  nodeSendIntervalMs: Number.parseInt(process.env.NODE_SEND_INTERVAL_MS || "3000", 10),
  topicDataPrefix: process.env.MQTT_TOPIC_DATA_PREFIX || "lora/fms",
  topicStatus: process.env.MQTT_TOPIC_STATUS || "lora/fms/status",
  topicDiagnostic: process.env.MQTT_TOPIC_DIAGNOSTIC || "lora/fms/diagnostic/route",
  topicFatigueImu: process.env.MQTT_TOPIC_FATIGUE_IMU || "lora/fms/fatigue_detection/imu",
  topicSafetyCondition: process.env.MQTT_TOPIC_SAFETY_CONDITION || "lora/fms/safety/condition",
  topicBnoData: process.env.MQTT_TOPIC_BNO_DATA || "lora/fms/bno/data"
};

const nodeNameById = {
  0: "GATEWAY",
  1: "TRK-001",
  2: "TRK-002",
  3: "TRK-003",
  4: "lora_saenab",
  5: "lora_nailah"
};

let eventCounter = 0;
let sessionCounter = 0;

const state = {
  startedAt: Date.now(),
  mqtt: {
    connected: false,
    lastConnectAt: null,
    lastDisconnectAt: null,
    lastMessageAt: null,
    lastError: "",
    subscribedTopics: []
  },
  events: [],
  routeEvents: [],
  gatewayStatusEvents: [],
  latestByNode: {},
  sessions: [],
  activeSession: null,
  automation: {
    running: false,
    startedAt: null,
    queue: [],
    current: null,
    completedCount: 0,
    totalCount: 0,
    durationMs: AUTO_DEFAULT_DURATION_MS,
    repeats: AUTO_DEFAULT_REPEATS,
    cooldownMs: AUTO_DEFAULT_COOLDOWN_MS,
    inactivityTimeoutSec: AUTO_DEFAULT_INACTIVITY_TIMEOUT_SEC,
    maxRouteFail: AUTO_DEFAULT_MAX_ROUTE_FAIL,
    baseLabel: "",
    presetName: "",
    lastError: "",
    lastStopReason: "",
    nextTimer: null
  }
};

// Fixed interval: firmware menggunakan 3 detik per node (bukan adaptive TDMA)
function getFixedIntervalMs() {
  return config.nodeSendIntervalMs || 3000;
}

function boundedPush(arr, value, maxLen) {
  arr.push(value);
  if (arr.length > maxLen) {
    arr.splice(0, arr.length - maxLen);
  }
}

function safeParseJson(raw) {
  try {
    return JSON.parse(raw);
  } catch (error) {
    return null;
  }
}

function num(value) {
  if (typeof value === "number" && Number.isFinite(value)) {
    return value;
  }
  if (typeof value === "string") {
    const parsed = Number.parseFloat(value);
    if (Number.isFinite(parsed)) {
      return parsed;
    }
  }
  return null;
}

function resolveNodeNameById(nodeId) {
  const n = Number.parseInt(String(nodeId), 10);
  if (!Number.isFinite(n)) {
    return String(nodeId || "UNKNOWN");
  }
  return nodeNameById[n] || `NODE-${n}`;
}

function mkNodeStats() {
  return {
    received: 0,
    firstPacketAt: 0,
    lastPacketAt: 0,
    latencySum: 0,
    latencyCount: 0,
    latencyMin: Number.POSITIVE_INFINITY,
    latencyMax: 0,
    interArrivalSum: 0,
    interArrivalCount: 0,
    hopSum: 0,
    hopCount: 0,
    rssiSum: 0,
    rssiCount: 0,
    snrSum: 0,
    snrCount: 0
  };
}

function ensureNodeStats(map, nodeName) {
  if (!map[nodeName]) {
    map[nodeName] = mkNodeStats();
  }
  return map[nodeName];
}

function mkRouteHopStats() {
  return {
    attempts: 0,
    success: 0,
    fail: 0,
    discoveryMsSum: 0,
    discoveryMsCount: 0
  };
}

function ensureRouteHopStats(map, hop) {
  const key = String(hop);
  if (!map[key]) {
    map[key] = mkRouteHopStats();
  }
  return map[key];
}

function currentSessionDurationMs(session) {
  const start = session.firstPacketAt || session.startedAt;
  const end = session.endedAt || Date.now();
  return Math.max(0, end - start);
}

function summarizeSession(session) {
  const durationMs = currentSessionDurationMs(session);
  const now = Date.now();

  const nodeRows = Object.entries(session.nodeStats).map(([node, s]) => {
    const nodeDurationMs = s.firstPacketAt > 0
      ? Math.max(0, (session.endedAt || now) - s.firstPacketAt)
      : durationMs;
    // Tambahkan +1 karena paket pertama dihitung pada detik ke-0 (fencepost error)
    const expected = Math.max(1, Math.floor(nodeDurationMs / session.intervalMs) + 1);
    const pdr = Math.min(100, (s.received / expected) * 100);
    const plr = Math.max(0, 100 - pdr);
    let latencyAvg = s.latencyCount > 0 ? s.latencySum / s.latencyCount : null;

    if (!latencyAvg || latencyAvg === 0 || latencyAvg > 60000) {
      latencyAvg = s.interArrivalCount > 0 ? s.interArrivalSum / s.interArrivalCount : null;
    }
    const hopAvg = s.hopCount > 0 ? s.hopSum / s.hopCount : null;
    const rssiAvg = s.rssiCount > 0 ? s.rssiSum / s.rssiCount : null;
    const snrAvg = s.snrCount > 0 ? s.snrSum / s.snrCount : null;

    // Cek apakah node masih aktif (mengirim dalam 30 detik terakhir)
    // Jika trial sudah berhenti, gunakan waktu terakhir sesi
    const lastActive = s.lastPacketAt || session.startedAt;
    const isStale = (session.endedAt) ? false : (now - lastActive > 30000);

    return {
      node,
      received: s.received,
      expected,
      pdr,
      plr,
      latencyAvg,
      latencyMin: Number.isFinite(s.latencyMin) ? s.latencyMin : null,
      latencyMax: s.latencyCount > 0 ? s.latencyMax : null,
      hopAvg,
      rssiAvg,
      snrAvg,
      isStale
    };
  });

  let totalReceived = 0;
  let totalExpected = 0;
  for (const row of nodeRows) {
    // HANYA hitung ke total jika node TIDAK STALE (mati)
    if (!row.isStale) {
      totalReceived += row.received;
      totalExpected += row.expected;
    }
  }

  const totalPdr = totalExpected > 0 ? Math.min(100, (totalReceived / totalExpected) * 100) : 0;
  const totalPlr = Math.max(0, 100 - totalPdr);

  const routeByHop = {};
  for (const [hop, entry] of Object.entries(session.route.byHop)) {
    routeByHop[hop] = {
      attempts: entry.attempts,
      success: entry.success,
      fail: entry.fail,
      avgDiscoveryMs: entry.discoveryMsCount > 0 ? entry.discoveryMsSum / entry.discoveryMsCount : null
    };
  }

  return {
    id: session.id,
    trialCode: session.trialCode,
    label: session.label,
    scenario: session.scenario,
    sf: session.sf,
    bwKHz: session.bwKHz,
    cr: session.cr,
    expectedHop: session.expectedHop,
    distanceM: session.distanceM,
    iterasi: session.iterasi,
    note: session.note,
    startedAt: session.startedAt,
    endedAt: session.endedAt,
    status: session.status || "running",
    stopReason: session.stopReason || "",
    durationMs,
    targetDurationMs: session.targetDurationMs || null,
    remainingMs:
      !session.endedAt && session.targetDurationMs
        ? Math.max(0, session.targetDurationMs - durationMs)
        : null,
    intervalMs: session.intervalMs,
    nodes: nodeRows,
    total: {
      received: totalReceived,
      expected: totalExpected,
      pdr: totalPdr,
      plr: totalPlr
    },
    route: {
      attempts: session.route.attempts,
      success: session.route.success,
      fail: session.route.fail,
      byHop: routeByHop
    }
  };
}

function toCsvRow(values) {
  return values
    .map((value) => {
      const str = value === null || value === undefined ? "" : String(value);
      if (str.includes(CSV_DELIMITER) || str.includes("\"") || str.includes("\n")) {
        return `"${str.replace(/"/g, "\"\"")}"`;
      }
      return str;
    })
    .join(CSV_DELIMITER);
}

function formatDateTime(timestamp) {
  if (!timestamp) {
    return "";
  }
  // Gunakan format Makassar untuk CSV export
  return new Intl.DateTimeFormat("id-ID", {
    year: "numeric",
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
    timeZone: "Asia/Makassar",
  }).format(new Date(timestamp)).replace(/\//g, "-");
}

function withCsvEncoding(csvText) {
  if (!CSV_INCLUDE_BOM) {
    return csvText;
  }
  return `\ufeff${csvText}`;
}

function formatDateForCode(ts) {
  const d = new Date(ts);
  const yyyy = d.getFullYear();
  const mm = String(d.getMonth() + 1).padStart(2, "0");
  const dd = String(d.getDate()).padStart(2, "0");
  const hh = String(d.getHours()).padStart(2, "0");
  const mi = String(d.getMinutes()).padStart(2, "0");
  const ss = String(d.getSeconds()).padStart(2, "0");
  return `${yyyy}${mm}${dd}-${hh}${mi}${ss}`;
}

function buildTrialCode(id, ts) {
  return `TRIAL-${formatDateForCode(ts)}-${String(id).padStart(4, "0")}`;
}

function sanitizePositiveInt(value, fallbackValue) {
  const n = Number.parseInt(String(value), 10);
  if (!Number.isFinite(n) || n <= 0) {
    return fallbackValue;
  }
  return n;
}

function isPacketEventKind(kind) {
  return kind === "sensor_data" || kind === "fatigue_imu" || kind === "safety_condition" || kind === "vehicle_telemetry";
}

function sessionsToCsv(summaries) {
  const header = [
    "id_trial",
    "kode_trial",
    "label",
    "mode_trial",
    "sf",
    "bw_khz",
    "cr",
    "hop_target",
    "jarak_m",
    "mulai",
    "selesai",
    "durasi_detik",
    "status",
    "alasan_stop",
    "node",
    "paket_terima_rx",
    "paket_expected",
    "retries_estimasi",
    "pdr_persen",
    "plr_persen",
    "latency_avg_ms",
    "latency_min_ms",
    "latency_max_ms",
    "hop_avg",
    "rssi_avg",
    "snr_avg",
    "route_attempt",
    "route_success",
    "route_fail",
    "note"
  ];

  const rows = [toCsvRow(header)];
  for (const summary of summaries) {
    for (const node of summary.nodes) {
      rows.push(
        toCsvRow([
          summary.id,
          summary.trialCode,
          summary.label,
          summary.scenario,
          summary.sf,
          summary.bwKHz,
          summary.cr,
          summary.expectedHop,
          summary.distanceM,
          formatDateTime(summary.startedAt),
          formatDateTime(summary.endedAt),
          (summary.durationMs / 1000).toFixed(2),
          summary.status || "",
          summary.stopReason || "",
          node.node,
          node.received,
          node.expected,
          Math.max(0, Number(node.expected || 0) - Number(node.received || 0)),
          node.pdr.toFixed(2),
          node.plr.toFixed(2),
          node.latencyAvg !== null ? node.latencyAvg.toFixed(2) : "",
          node.latencyMin !== null ? node.latencyMin.toFixed(2) : "",
          node.latencyMax !== null ? node.latencyMax.toFixed(2) : "",
          node.hopAvg !== null ? node.hopAvg.toFixed(2) : "",
          node.rssiAvg !== null ? node.rssiAvg.toFixed(2) : "",
          node.snrAvg !== null ? node.snrAvg.toFixed(2) : "",
          summary.route.attempts,
          summary.route.success,
          summary.route.fail,
          summary.note || ""
        ])
      );
    }
  }
  return rows.join("\n");
}

function toTrialMeta(session) {
  if (!session) {
    return null;
  }
  return {
    id: session.id,
    trialCode: session.trialCode,
    label: session.label,
    sf: session.sf || "",
    bwKHz: session.bwKHz || "",
    cr: session.cr || "",
    expectedHop: session.expectedHop || "",
    distanceM: session.distanceM || "",
    iterasi: session.iterasi || "1",
    note: session.note || "",
    intervalMs: getFixedIntervalMs(),
    targetDurationMs: session.targetDurationMs || null,
    startedAt: session.startedAt || 0
  };
}

function getAllMqttEvents() {
  return [...state.events, ...state.routeEvents, ...state.gatewayStatusEvents].sort((a, b) => {
    const dt = (a.ts || 0) - (b.ts || 0);
    if (dt !== 0) {
      return dt;
    }
    return (a.id || 0) - (b.id || 0);
  });
}

function computeSummaryLatencyAvg(summary) {
  const nodeRows = Array.isArray(summary?.nodes) ? summary.nodes : [];
  let sum = 0;
  let count = 0;
  for (const row of nodeRows) {
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

function allMqttEventsToCsv(allEvents) {
  const header = [
    "waktu",
    "event_id",
    "jenis_event",
    "topic",
    "node",
    "node_id",
    "trial_code",
    "trial_label",
    "iterasi",
    "sf",
    "bw_khz",
    "cr",
    "hop_target",
    "jarak_m",
    "durasi_trial_menit",
    "hop",
    "route_path",
    "latency_ms",
    "rreq_at",
    "rrep_at",
    "discovery_ms",
    "retries",
    "success",
    "rssi",
    "payload_json"
  ];

  const rows = [toCsvRow(header)];
  for (const event of allEvents) {
    const trial = event.trial || {};
    const successValue = event.success === true ? 1 : event.success === false ? 0 : "";
    const routePath = (event.payload && Array.isArray(event.payload.route_path))
      ? event.payload.route_path.join(" -> ")
      : "";
    // Hop otomatis dari route_path, fallback ke event.hops
    const hopDisplay = (event.payload && Array.isArray(event.payload.route_path) && event.payload.route_path.length > 0)
      ? (event.payload.route_path.length - 1)
      : (event.hops ?? "");

    rows.push(
      toCsvRow([
        formatDateTime(event.ts),
        event.id ?? "",
        event.kind || "",
        event.topic || "",
        event.node || "",
        event.nodeId ?? "",
        trial.trialCode || "",
        trial.label || "",
        trial.iterasi || "1",
        trial.sf || "",
        trial.bwKHz || "",
        trial.cr || "",
        trial.expectedHop || "",
        trial.distanceM || "",
        trial.targetDurationMs ? Math.max(1, Math.round(Number(trial.targetDurationMs) / 60000)) : "",
        hopDisplay,
        routePath,
        event.latencyMs ?? "",
        event.rreqAt ?? "",
        event.rrepAt ?? "",
        event.discoveryMs ?? "",
        event.retries ?? "",
        successValue,
        event.rssi ?? "",
        JSON.stringify(event.payload || {})
      ])
    );
  }
  return rows.join("\n");
}

function scenario4ToCsv(summaries) {
  const header = [
    "trial_id",
    "trial_code",
    "label",
    "sf",
    "bw_khz",
    "cr",
    "jarak_m",
    "paket_dikirim",
    "paket_diterima",
    "mulai",
    "selesai",
    "durasi_detik",
    "pdr_total_persen",
    "plr_total_persen",
    "latency_rata2_semua_node_ms",
    "route_attempt",
    "route_success",
    "route_fail"
  ];

  const rows = [toCsvRow(header)];
  for (const summary of summaries) {
    const latencyAvg = computeSummaryLatencyAvg(summary);
    rows.push(
      toCsvRow([
        summary.id,
        summary.trialCode,
        summary.label,
        summary.sf || "",
        summary.bwKHz || "",
        summary.cr || "",
        summary.distanceM || "",
        summary.total?.expected ?? 0,
        summary.total?.received ?? 0,
        formatDateTime(summary.startedAt),
        formatDateTime(summary.endedAt),
        (summary.durationMs / 1000).toFixed(2),
        summary.total?.pdr !== undefined ? Number(summary.total.pdr).toFixed(2) : "",
        summary.total?.plr !== undefined ? Number(summary.total.plr).toFixed(2) : "",
        latencyAvg !== null ? latencyAvg.toFixed(2) : "",
        summary.route?.attempts ?? 0,
        summary.route?.success ?? 0,
        summary.route?.fail ?? 0
      ])
    );
  }
  return rows.join("\n");
}

function scenario3ToCsv(summaries) {
  const header = [
    "trial_id",
    "trial_code",
    "label",
    "iterasi",
    "sf",
    "bw_khz",
    "cr",
    "jarak_m",
    "mulai",
    "selesai",
    "durasi_detik",
    "node",
    "paket_masuk",
    "pdr_persen",
    "plr_persen",
    "delay_ms"
  ];

  const rows = [toCsvRow(header)];
  for (const summary of summaries) {
    const nodeList = Array.isArray(summary.nodes) ? summary.nodes : [];
    for (const node of nodeList) {
      rows.push(
        toCsvRow([
          summary.id,
          summary.trialCode,
          summary.label,
          summary.iterasi || "1",
          summary.sf || "",
          summary.bwKHz || "",
          summary.cr || "",
          summary.distanceM || "",
          formatDateTime(summary.startedAt),
          formatDateTime(summary.endedAt),
          (summary.durationMs / 1000).toFixed(2),
          node.node,
          node.received,
          node.pdr.toFixed(2),
          node.plr.toFixed(2),
          node.latencyAvg !== null ? node.latencyAvg.toFixed(2) : ""
        ])
      );
    }
  }
  return rows.join("\n");
}

function scenario5ToCsv(summaries) {
  const header = [
    "trial_id",
    "trial_code",
    "label",
    "sf",
    "bw_khz",
    "cr",
    "mulai",
    "selesai",
    "durasi_detik",
    "jarak_m",
    "node",
    "hop_dilalui",
    "jalur_route",
    "status_pengiriman"
  ];

  const rows = [toCsvRow(header)];
  for (const summary of summaries) {
    const nodeList = Array.isArray(summary.nodes) ? summary.nodes : [];
    if (nodeList.length === 0) {
      rows.push(
        toCsvRow([
          summary.id,
          summary.trialCode,
          summary.label,
          summary.sf || "",
          summary.bwKHz || "",
          summary.cr || "",
          formatDateTime(summary.startedAt),
          formatDateTime(summary.endedAt),
          (summary.durationMs / 1000).toFixed(2),
          summary.distanceM || "",
          "-",
          "-",
          "-",
          "GAGAL"
        ])
      );
      continue;
    }

    for (const node of nodeList) {
      const nodeName = node.node;
      const latestEvt = state.latestByNode[nodeName];
      const routePathArr = (latestEvt && latestEvt.payload && Array.isArray(latestEvt.payload.route_path))
        ? latestEvt.payload.route_path
        : [];
      const routePath = routePathArr.length > 0 ? routePathArr.join(" -> ") : "";
      const hopDisplay = routePathArr.length > 0 ? (routePathArr.length - 1) : (node.hopAvg !== null ? Math.round(node.hopAvg) : "-");
      const successText = (node.received || 0) > 0 ? "BERHASIL" : "GAGAL";

      rows.push(
        toCsvRow([
          summary.id,
          summary.trialCode,
          summary.label,
          summary.sf || "",
          summary.bwKHz || "",
          summary.cr || "",
          formatDateTime(summary.startedAt),
          formatDateTime(summary.endedAt),
          (summary.durationMs / 1000).toFixed(2),
          summary.distanceM || "",
          nodeName,
          hopDisplay,
          routePath,
          successText
        ])
      );
    }
  }
  return rows.join("\n");
}

function packetEventsToCsv(events) {
  const header = [
    "waktu",
    "id_trial",
    "kode_trial",
    "label_trial",
    "sf",
    "bw_khz",
    "cr",
    "hop_target",
    "jarak_m",
    "event_id",
    "kind",
    "topic",
    "node",
    "node_id",
    "hop",
    "route_path",
    "latency_ms",
    "payload_json"
  ];
  const rows = [toCsvRow(header)];
  for (const event of events) {
    if (!isPacketEventKind(event.kind)) {
      continue;
    }
    const trial = event.trial || {};
    const routePath = (event.payload && Array.isArray(event.payload.route_path))
      ? event.payload.route_path.join(" -> ")
      : "";
    // Hop otomatis dari route_path, fallback ke event.hops
    const hopDisplay = (event.payload && Array.isArray(event.payload.route_path) && event.payload.route_path.length > 0)
      ? (event.payload.route_path.length - 1)
      : (event.hops ?? "");

    rows.push(
      toCsvRow([
        formatDateTime(event.ts),
        trial.id || "",
        trial.trialCode || "",
        trial.label || "",
        trial.sf || "",
        trial.bwKHz || "",
        trial.cr || "",
        trial.expectedHop || "",
        trial.distanceM || "",
        event.id,
        event.kind,
        event.topic,
        event.node || "",
        event.nodeId ?? "",
        hopDisplay,
        routePath,
        event.latencyMs ?? "",
        JSON.stringify(event.payload || {})
      ])
    );
  }
  return rows.join("\n");
}

function routeEventsToCsv(routeEvents) {
  const header = [
    "waktu",
    "id_trial",
    "kode_trial",
    "label_trial",
    "sf",
    "bw_khz",
    "cr",
    "hop_target",
    "jarak_m",
    "event_id",
    "node",
    "node_id",
    "target",
    "hops",
    "route_path",
    "retries",
    "durasi_discovery_ms",
    "success",
    "rssi",
    "payload_json"
  ];
  const rows = [toCsvRow(header)];
  for (const event of routeEvents) {
    const trial = event.trial || {};
    const routePath = (event.payload && Array.isArray(event.payload.route_path))
      ? event.payload.route_path.join(" -> ")
      : "";
    // Hop otomatis dari route_path, fallback ke event.hops
    const hopDisplay = (event.payload && Array.isArray(event.payload.route_path) && event.payload.route_path.length > 0)
      ? (event.payload.route_path.length - 1)
      : (event.hops ?? "");

    rows.push(
      toCsvRow([
        formatDateTime(event.ts),
        trial.id || "",
        trial.trialCode || "",
        trial.label || "",
        trial.sf || "",
        trial.bwKHz || "",
        trial.cr || "",
        trial.expectedHop || "",
        trial.distanceM || "",
        event.id,
        event.node || "",
        event.nodeId ?? "",
        event.target ?? "",
        hopDisplay,
        routePath,
        event.retries ?? "",
        event.discoveryMs ?? "",
        event.success ? 1 : 0,
        event.rssi ?? "",
        JSON.stringify(event.payload || {})
      ])
    );
  }
  return rows.join("\n");
}

function findLastByPredicate(arr, predicate) {
  for (let i = arr.length - 1; i >= 0; i--) {
    if (predicate(arr[i])) {
      return arr[i];
    }
  }
  return null;
}

function computeReadiness() {
  const now = Date.now();
  const lastPacketEvent = findLastByPredicate(state.events, (e) => isPacketEventKind(e.kind));
  const lastLatencyEvent = findLastByPredicate(
    state.events,
    (e) => isPacketEventKind(e.kind) && e.latencyMs !== null && e.latencyMs !== undefined
  );
  const lastRouteEvent = state.routeEvents.length > 0 ? state.routeEvents[state.routeEvents.length - 1] : null;
  const lastStatusEvent = state.gatewayStatusEvents.length > 0
    ? state.gatewayStatusEvents[state.gatewayStatusEvents.length - 1]
    : null;

  const hasRecentPacket = !!lastPacketEvent && now - lastPacketEvent.ts <= READINESS_DATA_MAX_AGE_MS;
  const hasRecentRouteDiag = !!lastRouteEvent && now - lastRouteEvent.ts <= READINESS_DIAG_MAX_AGE_MS;
  const hasRecentGatewayStatus = !!lastStatusEvent && now - lastStatusEvent.ts <= READINESS_STATUS_MAX_AGE_MS;
  const hasRecentLatencySignal = !!lastLatencyEvent && now - lastLatencyEvent.ts <= READINESS_DATA_MAX_AGE_MS;

  const checks = {
    mqtt_connected: {
      ok: state.mqtt.connected,
      detail: state.mqtt.connected ? "MQTT terhubung" : "MQTT belum terhubung"
    },
    packet_flow_recent: {
      ok: hasRecentPacket,
      detail: lastPacketEvent
        ? `Paket terakhir ${Math.round((now - lastPacketEvent.ts) / 1000)} detik lalu`
        : "Belum ada paket data masuk"
    },
    route_diag_recent: {
      ok: hasRecentRouteDiag,
      detail: lastRouteEvent
        ? `Diagnostic terakhir ${Math.round((now - lastRouteEvent.ts) / 1000)} detik lalu`
        : "Belum ada diagnostic route"
    },
    gateway_status_recent: {
      ok: hasRecentGatewayStatus,
      detail: lastStatusEvent
        ? `Status gateway terakhir ${Math.round((now - lastStatusEvent.ts) / 1000)} detik lalu`
        : "Belum ada status gateway"
    },
    latency_signal_recent: {
      ok: hasRecentLatencySignal,
      detail: lastLatencyEvent
        ? `Latency terakhir ${Math.round((now - lastLatencyEvent.ts) / 1000)} detik lalu`
        : "Belum ada payload dengan latency_ms"
    }
  };

  return {
    checkedAt: now,
    canStartTrial: state.mqtt.connected,
    checks
  };
}

function broadcast(payload) {
  const message = JSON.stringify(payload);
  wss.clients.forEach((client) => {
    if (client.readyState === 1) {
      client.send(message);
    }
  });
}

function dataTopicMatches(topic) {
  const prefix = `${config.topicDataPrefix}/`;
  if (!topic.startsWith(prefix)) {
    return false;
  }
  if (topic === config.topicStatus) {
    return false;
  }
  if (topic === config.topicDiagnostic) {
    return false;
  }
  if (topic === config.topicFatigueImu) {
    return false;
  }
  if (topic === config.topicSafetyCondition) {
    return false;
  }
  if (topic === config.topicBnoData) {
    return false;
  }
  return true;
}

function normalizeEvent(topic, payloadRaw) {
  const payload = safeParseJson(payloadRaw);
  const now = Date.now();

  if (topic === config.topicStatus) {
    return {
      id: ++eventCounter,
      kind: "gateway_status",
      topic,
      ts: now,
      payload: payload || { raw: payloadRaw }
    };
  }

  if (topic === config.topicDiagnostic) {
    const hop = num(payload?.hops);
    return {
      id: ++eventCounter,
      kind: "route_diagnostic",
      topic,
      ts: now,
      node: payload?.node_name || resolveNodeNameById(payload?.node_asal),
      nodeId: payload?.node_asal ?? null,
      target: payload?.target_rute ?? null,
      success: num(payload?.success) === 1,
      hops: hop !== null ? hop : null,
      retries: num(payload?.retries),
      rreqAt: num(payload?.rreq_at),
      rrepAt: num(payload?.rrep_at),
      discoveryMs: num(payload?.discovery_ms),
      rssi: num(payload?.rssi),
      payload: payload || { raw: payloadRaw }
    };
  }

  if (topic === config.topicFatigueImu) {
    const nodeId = payload?.nodeId;
    let latencyMs = num(payload?.latency_ms);
    let rawEpoch = payload?.epoch ?? payload?.ts;
    if (latencyMs === null && rawEpoch) {
      const tsLow32 = num(rawEpoch);
      const nowLow32 = now % 4294967296; // 2^32
      let tsDiff = nowLow32 - tsLow32;
      if (tsDiff < 0) tsDiff += 4294967296;
      if (tsDiff >= 0 && tsDiff < 60000) latencyMs = tsDiff;
    }
    return {
      id: ++eventCounter,
      kind: "fatigue_imu",
      topic,
      ts: now,
      node: resolveNodeNameById(nodeId),
      nodeId: nodeId ?? null,
      latencyMs: latencyMs,
      hops: num(payload?.route_hops) ?? num(payload?.hopCount),
      rssi: num(payload?.rssi),
      snr: num(payload?.snr),
      discoveryMs: num(payload?.route_disc_ms),
      payload: payload || { raw: payloadRaw }
    };
  }

  if (topic === config.topicSafetyCondition) {
    const nodeId = payload?.nodeId;
    let latencyMs = num(payload?.latency_ms);
    let rawEpoch = payload?.epoch ?? payload?.ts;
    if (latencyMs === null && rawEpoch) {
      const tsLow32 = num(rawEpoch);
      const nowLow32 = now % 4294967296;
      let tsDiff = nowLow32 - tsLow32;
      if (tsDiff < 0) tsDiff += 4294967296;
      if (tsDiff >= 0 && tsDiff < 60000) latencyMs = tsDiff;
    }
    return {
      id: ++eventCounter,
      kind: "safety_condition",
      topic,
      ts: now,
      node: resolveNodeNameById(nodeId),
      nodeId: nodeId ?? null,
      latencyMs: latencyMs,
      hops: num(payload?.route_hops) ?? num(payload?.hopCount),
      rssi: num(payload?.rssi),
      snr: num(payload?.snr),
      discoveryMs: num(payload?.route_disc_ms),
      payload: payload || { raw: payloadRaw }
    };
  }

  if (topic === config.topicBnoData) {
    const nodeId = payload?.nodeId;
    let latencyMs = num(payload?.latency_ms);
    let rawEpoch = payload?.epoch ?? payload?.ts;
    if (latencyMs === null && rawEpoch) {
      const tsLow32 = num(rawEpoch);
      const nowLow32 = now % 4294967296;
      let tsDiff = nowLow32 - tsLow32;
      if (tsDiff < 0) tsDiff += 4294967296;
      if (tsDiff >= 0 && tsDiff < 60000) latencyMs = tsDiff;
    }
    return {
      id: ++eventCounter,
      kind: "vehicle_telemetry",
      topic,
      ts: now,
      node: nodeId !== undefined ? resolveNodeNameById(nodeId) : "lora_nailah",
      nodeId: nodeId ?? null,
      latencyMs: latencyMs,
      hops: num(payload?.route_hops) ?? num(payload?.hopCount),
      rssi: num(payload?.rssi),
      snr: num(payload?.snr),
      discoveryMs: num(payload?.route_disc_ms),
      payload: payload || { raw: payloadRaw }
    };
  }

  if (dataTopicMatches(topic)) {
    const node = topic.replace(`${config.topicDataPrefix}/`, "");
    let latencyMs = num(payload?.latency_ms);
    let rawEpoch = payload?.epoch ?? payload?.ts;
    if (latencyMs === null && rawEpoch) {
      const tsLow32 = num(rawEpoch);
      const nowLow32 = now % 4294967296;
      let tsDiff = nowLow32 - tsLow32;
      if (tsDiff < 0) tsDiff += 4294967296;
      if (tsDiff >= 0 && tsDiff < 60000) latencyMs = tsDiff;
    }
    return {
      id: ++eventCounter,
      kind: "sensor_data",
      topic,
      ts: now,
      node,
      nodeId: payload?.nodeId ?? null,
      latencyMs: latencyMs,
      hops: num(payload?.route_hops) ?? num(payload?.hopCount),
      rssi: num(payload?.rssi),
      snr: num(payload?.snr),
      discoveryMs: num(payload?.route_disc_ms),
      payload: payload || { raw: payloadRaw }
    };
  }

  return {
    id: ++eventCounter,
    kind: "unknown",
    topic,
    ts: now,
    payload: payload || { raw: payloadRaw }
  };
}

function updateSessionWithEvent(session, event) {
  if (!session || session.endedAt) {
    return;
  }

  if (event.kind === "route_diagnostic") {
    session.lastRouteEventAt = event.ts;
    session.route.attempts += 1;
    if (event.success) {
      session.route.success += 1;
    } else {
      session.route.fail += 1;
    }

    const hopKey = event.hops !== null ? event.hops : 0;
    const hopStats = ensureRouteHopStats(session.route.byHop, hopKey);
    hopStats.attempts += 1;
    if (event.success) {
      hopStats.success += 1;
    } else {
      hopStats.fail += 1;
    }
    if (event.discoveryMs !== null) {
      hopStats.discoveryMsSum += event.discoveryMs;
      hopStats.discoveryMsCount += 1;
    }
    return;
  }

  const packetKinds = new Set(["sensor_data", "fatigue_imu", "safety_condition", "vehicle_telemetry"]);
  if (!packetKinds.has(event.kind) || !event.node) {
    return;
  }
  session.lastPacketAt = event.ts;
  if (!session.firstPacketAt) {
    session.firstPacketAt = event.ts;
  }

  const nodeStats = ensureNodeStats(session.nodeStats, event.node);
  if (!nodeStats.firstPacketAt) {
    nodeStats.firstPacketAt = event.ts;
  } else if (nodeStats.lastPacketAt > 0) {
    const diff = event.ts - nodeStats.lastPacketAt;
    if (diff > 0 && diff < 120000) { // maksimal 120 detik agar logic
      nodeStats.interArrivalSum += diff;
      nodeStats.interArrivalCount += 1;
    }
  }

  nodeStats.received += 1;
  nodeStats.lastPacketAt = event.ts;

  if (event.latencyMs !== null && event.latencyMs >= 0 && event.latencyMs < 60000) {
    nodeStats.latencySum += event.latencyMs;
    nodeStats.latencyCount += 1;
    if (event.latencyMs < nodeStats.latencyMin) {
      nodeStats.latencyMin = event.latencyMs;
    }
    if (event.latencyMs > nodeStats.latencyMax) {
      nodeStats.latencyMax = event.latencyMs;
    }
  }

  if (event.hops !== null) {
    nodeStats.hopSum += event.hops;
    nodeStats.hopCount += 1;
  }

  if (event.rssi !== null && event.rssi !== undefined) {
    nodeStats.rssiSum += event.rssi;
    nodeStats.rssiCount += 1;
  }

  if (event.snr !== null && event.snr !== undefined) {
    nodeStats.snrSum += event.snr;
    nodeStats.snrCount += 1;
  }
}

function handleIncomingEvent(event) {
  state.mqtt.lastMessageAt = event.ts;
  if (state.activeSession) {
    event.trial = toTrialMeta(state.activeSession);
  }

  if (event.kind === "route_diagnostic") {
    boundedPush(state.routeEvents, event, MAX_ROUTE_EVENTS);
  } else if (event.kind === "gateway_status") {
    boundedPush(state.gatewayStatusEvents, event, MAX_STATUS_EVENTS);
  } else {
    boundedPush(state.events, event, MAX_EVENTS);
    if (event.node) {
      state.latestByNode[event.node] = event;
    }
  }

  if (state.activeSession) {
    updateSessionWithEvent(state.activeSession, event);
  }

  broadcast({ type: "event", event });
}

function buildAutomationProgress() {
  return {
    running: state.automation.running,
    startedAt: state.automation.startedAt,
    completedCount: state.automation.completedCount,
    totalCount: state.automation.totalCount,
    remainingCount: state.automation.queue.length,
    current: state.automation.current,
    config: {
      durationMs: state.automation.durationMs,
      repeats: state.automation.repeats,
      cooldownMs: state.automation.cooldownMs,
      inactivityTimeoutSec: state.automation.inactivityTimeoutSec,
      maxRouteFail: state.automation.maxRouteFail,
      baseLabel: state.automation.baseLabel,
      presetName: state.automation.presetName
    },
    lastError: state.automation.lastError,
    lastStopReason: state.automation.lastStopReason
  };
}

function resetAutomationState() {
  if (state.automation.nextTimer) {
    clearTimeout(state.automation.nextTimer);
  }
  state.automation.running = false;
  state.automation.startedAt = null;
  state.automation.queue = [];
  state.automation.current = null;
  state.automation.completedCount = 0;
  state.automation.totalCount = 0;
  state.automation.durationMs = AUTO_DEFAULT_DURATION_MS;
  state.automation.repeats = AUTO_DEFAULT_REPEATS;
  state.automation.cooldownMs = AUTO_DEFAULT_COOLDOWN_MS;
  state.automation.inactivityTimeoutSec = AUTO_DEFAULT_INACTIVITY_TIMEOUT_SEC;
  state.automation.maxRouteFail = AUTO_DEFAULT_MAX_ROUTE_FAIL;
  state.automation.baseLabel = "";
  state.automation.presetName = "";
  state.automation.lastError = "";
  state.automation.lastStopReason = "";
  state.automation.nextTimer = null;
}

function startSession(input, options = {}) {
  if (state.activeSession) {
    return null;
  }

  const intervalMs = config.nodeSendIntervalMs;
  const startedAt = Date.now();
  const trialId = ++sessionCounter;
  const targetDurationMsRaw = Number.parseInt(String(options.targetDurationMs ?? ""), 10);
  const targetDurationMs = Number.isFinite(targetDurationMsRaw) && targetDurationMsRaw > 0
    ? targetDurationMsRaw
    : null;
  const session = {
    id: trialId,
    trialCode: buildTrialCode(trialId, startedAt),
    label: input.label || `Trial-${trialId}`,
    scenario: "combined_2_5",
    sf: input.sf || "",
    bwKHz: input.bwKHz || "",
    cr: input.cr || "",
    expectedHop: input.expectedHop || "",
    distanceM: input.distanceM || "",
    iterasi: input.iterasi || "1",
    note: "",
    intervalMs: Number.isFinite(intervalMs) && intervalMs > 0 ? intervalMs : config.nodeSendIntervalMs,
    startedAt,
    endedAt: null,
    status: "running",
    stopReason: "",
    startedBy: options.startedBy || "manual",
    autoCaseLabel: options.autoCaseLabel || "",
    targetDurationMs,
    lastPacketAt: null,
    nodeStats: {},
    route: {
      attempts: 0,
      success: 0,
      fail: 0,
      byHop: {}
    }
  };

  state.activeSession = session;
  state.sessions.push(session);
  if (state.sessions.length > MAX_SESSIONS) {
    state.sessions.splice(0, state.sessions.length - MAX_SESSIONS);
  }

  return session;
}

function stopSession(reason = "manual_stop") {
  if (!state.activeSession) {
    return null;
  }
  const endedAt = Date.now();
  state.activeSession.endedAt = endedAt;
  state.activeSession.stopReason = reason;
  if (reason === "duration_elapsed") {
    state.activeSession.status = "completed";
  } else if (reason === "auto_fail_route") {
    state.activeSession.status = "failed_route";
  } else if (reason === "auto_fail_inactive") {
    state.activeSession.status = "failed_inactive";
  } else if (reason === "automation_stopped") {
    state.activeSession.status = "aborted";
  } else {
    state.activeSession.status = "manual_stopped";
  }

  const summary = summarizeSession(state.activeSession);
  state.activeSession = null;
  onSessionStopped(summary);
  return summary;
}

function buildAutomationQueue(payload) {
  const queue = [];
  const matrix = Array.isArray(payload.matrix) ? payload.matrix : [];
  const repeats = sanitizePositiveInt(payload.repeats, AUTO_DEFAULT_REPEATS);
  const baseLabel = String(payload.baseLabel || "AUTO").trim();
  for (let i = 0; i < matrix.length; i++) {
    const item = matrix[i];
    for (let r = 1; r <= repeats; r++) {
      const itemLabel = String(item.label || `Case-${i + 1}`).trim();
      queue.push({
        label: `${baseLabel} | ${itemLabel} | Rep-${r}`,
        sf: String(item.sf || ""),
        bwKHz: String(item.bwKHz || ""),
        cr: String(item.cr || ""),
        expectedHop: String(item.expectedHop || ""),
        distanceM: String(item.distanceM || ""),
        matrixIndex: i + 1,
        repeatIndex: r,
        totalRepeats: repeats
      });
    }
  }
  return queue;
}

function startNextAutomationTrial() {
  if (!state.automation.running) {
    return null;
  }
  if (state.activeSession) {
    return null;
  }
  if (state.automation.queue.length === 0) {
    state.automation.running = false;
    state.automation.current = null;
    state.automation.lastStopReason = "queue_completed";
    broadcast({ type: "automation_completed", automation: buildAutomationProgress() });
    return null;
  }

  const nextItem = state.automation.queue.shift();
  const session = startSession(nextItem, {
    startedBy: "automation",
    autoCaseLabel: nextItem.label
  });
  if (!session) {
    state.automation.lastError = "Gagal memulai trial otomatis berikutnya.";
    state.automation.running = false;
    broadcast({ type: "automation_error", automation: buildAutomationProgress() });
    return null;
  }

  state.automation.current = {
    sessionId: session.id,
    trialCode: session.trialCode,
    label: session.label,
    sf: session.sf,
    bwKHz: session.bwKHz,
    cr: session.cr,
    expectedHop: session.expectedHop,
    distanceM: session.distanceM,
    startedAt: session.startedAt,
    endsAt: session.startedAt + state.automation.durationMs,
    matrixIndex: nextItem.matrixIndex,
    repeatIndex: nextItem.repeatIndex,
    totalRepeats: nextItem.totalRepeats
  };

  broadcast({ type: "automation_trial_started", automation: buildAutomationProgress() });
  return session;
}

function onSessionStopped(summary) {
  if (!summary || !state.automation.running) {
    return;
  }

  state.automation.completedCount += 1;
  state.automation.current = null;

  if (state.automation.queue.length === 0) {
    state.automation.running = false;
    state.automation.lastStopReason = "queue_completed";
    broadcast({ type: "automation_completed", automation: buildAutomationProgress(), summary });
    return;
  }

  if (state.automation.nextTimer) {
    clearTimeout(state.automation.nextTimer);
  }
  state.automation.nextTimer = setTimeout(() => {
    state.automation.nextTimer = null;
    startNextAutomationTrial();
  }, state.automation.cooldownMs);
  broadcast({ type: "automation_waiting_next", automation: buildAutomationProgress(), summary });
}

function stopAutomation(reason = "manual_stop") {
  if (!state.automation.running) {
    return buildAutomationProgress();
  }
  state.automation.running = false;
  state.automation.lastStopReason = reason;
  if (state.automation.nextTimer) {
    clearTimeout(state.automation.nextTimer);
    state.automation.nextTimer = null;
  }
  if (state.activeSession && state.activeSession.startedBy === "automation") {
    stopSession("automation_stopped");
  }
  broadcast({ type: "automation_stopped", automation: buildAutomationProgress() });
  return buildAutomationProgress();
}

function startAutomation(payload) {
  if (state.automation.running) {
    return { error: "Otomatisasi sudah berjalan." };
  }
  if (state.activeSession) {
    return { error: "Masih ada trial aktif. Stop trial aktif dulu." };
  }

  const queue = buildAutomationQueue(payload);
  if (queue.length === 0) {
    return { error: "Matrix trial kosong." };
  }

  const durationMs = sanitizePositiveInt(payload.durationMs, AUTO_DEFAULT_DURATION_MS);
  const repeats = sanitizePositiveInt(payload.repeats, AUTO_DEFAULT_REPEATS);
  const cooldownMs = sanitizePositiveInt(payload.cooldownMs, AUTO_DEFAULT_COOLDOWN_MS);
  const inactivityTimeoutSec = sanitizePositiveInt(
    payload.inactivityTimeoutSec,
    AUTO_DEFAULT_INACTIVITY_TIMEOUT_SEC
  );
  const maxRouteFail = Number.parseInt(String(payload.maxRouteFail), 10);
  const safeMaxRouteFail = Number.isFinite(maxRouteFail) ? Math.max(0, maxRouteFail) : AUTO_DEFAULT_MAX_ROUTE_FAIL;

  state.automation.running = true;
  state.automation.startedAt = Date.now();
  state.automation.queue = queue;
  state.automation.current = null;
  state.automation.completedCount = 0;
  state.automation.totalCount = queue.length;
  state.automation.durationMs = durationMs;
  state.automation.repeats = repeats;
  state.automation.cooldownMs = cooldownMs;
  state.automation.inactivityTimeoutSec = inactivityTimeoutSec;
  state.automation.maxRouteFail = safeMaxRouteFail;
  state.automation.baseLabel = String(payload.baseLabel || "AUTO").trim();
  state.automation.presetName = String(payload.presetName || "").trim();
  state.automation.lastError = "";
  state.automation.lastStopReason = "";

  startNextAutomationTrial();
  return { automation: buildAutomationProgress() };
}

function automationTick() {
  if (!state.automation.running || !state.activeSession) {
    return;
  }
  const now = Date.now();
  const session = state.activeSession;
  const elapsedMs = now - session.startedAt;

  if (session.route.fail > state.automation.maxRouteFail) {
    stopSession("auto_fail_route");
    return;
  }

  const lastPacketAt = session.lastPacketAt || session.startedAt;
  if (now - lastPacketAt > state.automation.inactivityTimeoutSec * 1000) {
    stopSession("auto_fail_inactive");
    return;
  }

  if (elapsedMs >= state.automation.durationMs) {
    stopSession("duration_elapsed");
  }
}

function manualSessionTick() {
  if (!state.activeSession) {
    return;
  }
  const session = state.activeSession;
  if (session.startedBy !== "manual") {
    return;
  }
  if (!session.targetDurationMs) {
    return;
  }
  const elapsedMs = Date.now() - session.startedAt;
  if (elapsedMs >= session.targetDurationMs) {
    const summary = stopSession("duration_elapsed");
    if (summary) {
      broadcast({ type: "session_stopped", summary });
    }
  }
}

const app = express();
const server = http.createServer(app);
const wss = new WebSocketServer({ server });

app.use(express.json({ limit: "1mb" }));
app.use(express.static(path.join(__dirname, "public")));

function sendCsv(res, filename, csvContent) {
  res.setHeader("Content-Type", "text/csv; charset=utf-8");
  res.setHeader("Content-Disposition", `attachment; filename="${filename}"`);
  res.send(withCsvEncoding(csvContent));
}

app.get("/api/config", (req, res) => {
  res.json({
    mqttUri: config.mqttUri,
    topicDataPrefix: config.topicDataPrefix,
    topicStatus: config.topicStatus,
    topicDiagnostic: config.topicDiagnostic,
    topicFatigueImu: config.topicFatigueImu,
    topicSafetyCondition: config.topicSafetyCondition,
    topicBnoData: config.topicBnoData,
    nodeSendIntervalMs: config.nodeSendIntervalMs,
    autoDefaults: {
      durationMs: AUTO_DEFAULT_DURATION_MS,
      repeats: AUTO_DEFAULT_REPEATS,
      cooldownMs: AUTO_DEFAULT_COOLDOWN_MS,
      inactivityTimeoutSec: AUTO_DEFAULT_INACTIVITY_TIMEOUT_SEC,
      maxRouteFail: AUTO_DEFAULT_MAX_ROUTE_FAIL
    },
    manualDefaultDurationMs: MANUAL_DEFAULT_DURATION_MS
  });
});

app.get("/api/state", (req, res) => {
  const sessionSummaries = state.sessions.map((s) => summarizeSession(s));
  res.json({
    startedAt: state.startedAt,
    mqtt: state.mqtt,
    readiness: computeReadiness(),
    latestByNode: state.latestByNode,
    latestEvents: state.events,
    routeEvents: state.routeEvents,
    gatewayStatusEvents: state.gatewayStatusEvents,
    allMqttCount: state.events.length + state.routeEvents.length + state.gatewayStatusEvents.length,
    activeSession: state.activeSession ? summarizeSession(state.activeSession) : null,
    sessions: sessionSummaries,
    automation: buildAutomationProgress()
  });
});

app.get("/api/history/all_mqtt.json", (req, res) => {
  const allMqttEvents = getAllMqttEvents();
  res.json({
    total: allMqttEvents.length,
    events: allMqttEvents
  });
});

app.post("/api/session/start", (req, res) => {
  if (state.automation.running) {
    res.status(409).json({ error: "Otomatisasi sedang berjalan. Stop otomatisasi dulu." });
    return;
  }

  const payload = req.body || {};
  const requiredKeys = ["label", "sf", "bwKHz", "cr", "distanceM"];
  for (const key of requiredKeys) {
    const value = String(payload[key] || "").trim();
    if (!value) {
      res.status(400).json({ error: `Field wajib diisi: ${key}` });
      return;
    }
    payload[key] = value;
  }

  const durationMinutes = Number.parseFloat(String(payload.durationMinutes || ""));
  const targetDurationMs = Number.isFinite(durationMinutes) && durationMinutes > 0
    ? Math.round(durationMinutes * 60 * 1000)
    : MANUAL_DEFAULT_DURATION_MS;

  const session = startSession(payload, { startedBy: "manual", targetDurationMs });
  if (!session) {
    res.status(409).json({ error: "Trial aktif sudah ada. Stop dulu sebelum start trial baru." });
    return;
  }
  const summary = summarizeSession(session);
  broadcast({ type: "session_started", summary });
  res.json(summary);
});

app.post("/api/session/stop", (req, res) => {
  const summary = stopSession("manual_stop");
  if (!summary) {
    res.status(409).json({ error: "Tidak ada trial aktif." });
    return;
  }
  broadcast({ type: "session_stopped", summary });
  res.json(summary);
});

app.post("/api/automation/start", (req, res) => {
  const payload = req.body || {};
  const matrix = Array.isArray(payload.matrix) ? payload.matrix : [];
  if (matrix.length === 0) {
    res.status(400).json({ error: "Matrix trial kosong." });
    return;
  }

  for (let i = 0; i < matrix.length; i++) {
    const item = matrix[i] || {};
    const requiredKeys = ["label", "sf", "bwKHz", "cr", "distanceM"];
    for (const key of requiredKeys) {
      if (!String(item[key] || "").trim()) {
        res.status(400).json({ error: `Matrix item ${i + 1} field wajib diisi: ${key}` });
        return;
      }
    }
  }

  const result = startAutomation(payload);
  if (result.error) {
    res.status(409).json(result);
    return;
  }
  res.json(result);
});

app.post("/api/automation/stop", (req, res) => {
  const automation = stopAutomation("manual_stop");
  res.json({ automation });
});

app.post("/api/reset", (req, res) => {
  stopAutomation("reset");
  if (state.activeSession) {
    stopSession("reset");
  }

  state.events = [];
  state.routeEvents = [];
  state.gatewayStatusEvents = [];
  state.latestByNode = {};
  state.sessions = [];
  state.activeSession = null;
  resetAutomationState();
  broadcast({ type: "reset" });
  res.json({ ok: true });
});

app.get("/api/export/sessions.csv", (req, res) => {
  const summaries = state.sessions.map((s) => summarizeSession(s));
  const csv = sessionsToCsv(summaries);
  sendCsv(res, "lora_test_trials.csv", csv);
});

app.get("/api/export/summary.csv", (req, res) => {
  const summaries = state.sessions.map((s) => summarizeSession(s));
  const csv = sessionsToCsv(summaries);
  sendCsv(res, "summary_per_trial.csv", csv);
});

app.get("/api/export/raw_packets.csv", (req, res) => {
  const csv = packetEventsToCsv(state.events);
  sendCsv(res, "raw_packets.csv", csv);
});

app.get("/api/export/route_events.csv", (req, res) => {
  const csv = routeEventsToCsv(state.routeEvents);
  sendCsv(res, "route_discovery_events.csv", csv);
});

app.get("/api/export/all_data.csv", (req, res) => {
  const csv = allMqttEventsToCsv(getAllMqttEvents());
  sendCsv(res, "all_data_mqtt.csv", csv);
});

app.get("/api/export/pengujian2_route.csv", (req, res) => {
  const csv = routeEventsToCsv(state.routeEvents);
  sendCsv(res, "pengujian_2_route_discovery_aodv.csv", csv);
});

app.get("/api/export/pengujian3_qos.csv", (req, res) => {
  const summaries = state.sessions.map((s) => summarizeSession(s));
  const csv = scenario3ToCsv(summaries);
  sendCsv(res, "pengujian_3_qos_dan_latensi.csv", csv);
});

app.get("/api/export/pengujian4_parameter.csv", (req, res) => {
  const summaries = state.sessions.map((s) => summarizeSession(s));
  const csv = scenario4ToCsv(summaries);
  sendCsv(res, "pengujian_4_pengaruh_parameter_lora.csv", csv);
});

app.get("/api/export/pengujian5_jarak.csv", (req, res) => {
  const summaries = state.sessions.map((s) => summarizeSession(s));
  const csv = scenario5ToCsv(summaries);
  sendCsv(res, "pengujian_5_jangkauan_multihop.csv", csv);
});

wss.on("connection", (socket) => {
  socket.send(
    JSON.stringify({
      type: "hello",
      message: "websocket connected",
      now: Date.now()
    })
  );
});

const mqttOptions = {
  clientId: `lora-dashboard-${Math.random().toString(16).slice(2, 10)}`,
  reconnectPeriod: 2000,
  connectTimeout: 10000,
  keepalive: 60,
  clean: true,
  rejectUnauthorized: config.mqttRejectUnauthorized
};

if (config.mqttUsername) {
  mqttOptions.username = config.mqttUsername;
}
if (config.mqttPassword) {
  mqttOptions.password = config.mqttPassword;
}

const mqttClient = mqtt.connect(config.mqttUri, mqttOptions);

const subscribeTopics = [
  `${config.topicDataPrefix}/+`,
  config.topicStatus,
  config.topicDiagnostic,
  config.topicFatigueImu,
  config.topicSafetyCondition,
  config.topicBnoData
];

mqttClient.on("connect", () => {
  state.mqtt.connected = true;
  state.mqtt.lastConnectAt = Date.now();
  state.mqtt.lastError = "";
  mqttClient.subscribe(subscribeTopics, { qos: 1 }, (error) => {
    if (error) {
      state.mqtt.lastError = `Subscribe error: ${error.message}`;
    } else {
      state.mqtt.subscribedTopics = subscribeTopics;
    }
  });
  broadcast({ type: "mqtt_connected", mqtt: state.mqtt });
});

mqttClient.on("reconnect", () => {
  broadcast({ type: "mqtt_reconnect" });
});

mqttClient.on("close", () => {
  state.mqtt.connected = false;
  state.mqtt.lastDisconnectAt = Date.now();
  broadcast({ type: "mqtt_closed", mqtt: state.mqtt });
});

mqttClient.on("error", (error) => {
  state.mqtt.lastError = error.message;
  broadcast({ type: "mqtt_error", error: error.message });
});

mqttClient.on("message", (topic, payloadBuffer) => {
  const payloadRaw = payloadBuffer.toString("utf8");
  const event = normalizeEvent(topic, payloadRaw);
  handleIncomingEvent(event);
});

setInterval(automationTick, 1000);
setInterval(manualSessionTick, 1000);

server.listen(PORT, () => {
  console.log(`Localhost dashboard ready: http://localhost:${PORT}`);
  console.log(`MQTT broker target: ${config.mqttUri}`);
});
