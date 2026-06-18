#include "fms_mqtt.h"
#include "esp_err.h"
#include <Arduino.h>
#include <functional>
#include <stdio.h>
#include <string.h>

// Extern debug flags from fms.ino
extern bool DEBUG_ALL;
extern bool DEBUG_NETWORK;
extern void lcdQueueIncomingChatMessage(const char *msg, size_t len);

#if USE_MQTT_PURE

// ================= GLOBAL INSTANCE =================
FMSMqttClient fmsMqtt;

// ================= STATIC EVENT HANDLER BRIDGE =================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  fmsMqtt._handleNativeEvent(event_id, event_data);
}

// ================= CONSTRUCTOR =================
FMSMqttClient::FMSMqttClient()
    : _mqtt(nullptr), _isConnected(false), _isRunning(false), _mqttCallback(nullptr),
      _onCommandReceived(nullptr), _publishCount(0), _failCount(0),
      _reconnectCount(0), _lastWifiRSSILog(0), _lastRSSI(0) {
  memset(_vehicleId, 0, sizeof(_vehicleId));
  memset(_deviceId, 0, sizeof(_deviceId));
  memset(_clientId, 0, sizeof(_clientId));
  memset(_topicData, 0, sizeof(_topicData));
  memset(_topicLog, 0, sizeof(_topicLog));
  memset(_topicConfig, 0, sizeof(_topicConfig));
  memset(_topicConfigSet, 0, sizeof(_topicConfigSet));
  memset(_topicCommand, 0, sizeof(_topicCommand));
  memset(_topicSerial, 0, sizeof(_topicSerial));
  memset(_topicChat, 0, sizeof(_topicChat));
}

// ================= PUBLIC: begin =================
void FMSMqttClient::begin(const char *vehicleId, const char *deviceId) {
  strncpy(_vehicleId, vehicleId, sizeof(_vehicleId) - 1);
  strncpy(_deviceId, deviceId, sizeof(_deviceId) - 1);

  // Build topics
  buildTopics();

  // Create Client ID
  snprintf(_clientId, sizeof(_clientId), "FMS-%s-%lu", _vehicleId,
           (unsigned long)random(1000, 9999));

  // ENSURE EVENT LOOP IS READY
  esp_err_t loop_err = esp_event_loop_create_default();
  if (loop_err != ESP_OK && loop_err != ESP_ERR_INVALID_STATE) {
    if (DEBUG_NETWORK || DEBUG_ALL) {
      Serial.printf(
          "[MQTT] Critical Error: Failed to create event loop (0x%x)\n",
          loop_err);
    }
  }

  // Configure Native MQTT (Industrial Tuning)
  esp_mqtt_client_config_t mqtt_cfg = {};
  mqtt_cfg.broker.address.uri = MQTT_URI;
  mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
  mqtt_cfg.credentials.client_id = _clientId;
  mqtt_cfg.session.keepalive = MQTT_KEEPALIVE_S;
  mqtt_cfg.network.reconnect_timeout_ms = MQTT_RECONNECT_MS;
  mqtt_cfg.network.timeout_ms = MQTT_NETWORK_TIMEOUT_MS;
  mqtt_cfg.network.refresh_connection_after_ms = 30 * 60 * 1000;
  mqtt_cfg.task.stack_size = MQTT_TASK_STACK_SIZE;
  mqtt_cfg.buffer.size = MQTT_BUFFER_SIZE;
  mqtt_cfg.buffer.out_size = MQTT_BUFFER_SIZE;

  _mqtt = esp_mqtt_client_init(&mqtt_cfg);

  if (_mqtt == nullptr) {
    if (DEBUG_NETWORK || DEBUG_ALL) {
      Serial.println("[MQTT] ERROR: Failed to initialize MQTT handle!");
    }
    return;
  }

  // Register events
  esp_mqtt_client_register_event(_mqtt, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                 mqtt_event_handler, NULL);

  if (DEBUG_NETWORK || DEBUG_ALL) {
    Serial.println("[MQTT] Client initialized (start deferred).");
  }
}

// ================= PUBLIC: loop =================
void FMSMqttClient::loop() {
  unsigned long now = millis();

  // Log WiFi signal strength periodically (every 30s)
  if (now - _lastWifiRSSILog >= 30000 || _lastWifiRSSILog == 0) {
    logWifiSignalStrength();
    _lastWifiRSSILog = now;
  }
}

// ================= PUBLIC: isConnected =================
bool FMSMqttClient::isConnected() { return _isConnected; }

bool FMSMqttClient::isRunning() const { return _isRunning; }

bool FMSMqttClient::startClient() {
  if (_mqtt == nullptr) {
    return false;
  }
  if (_isRunning) {
    return true;
  }

  esp_err_t err = esp_mqtt_client_start(_mqtt);
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
    _isRunning = true;
    if (DEBUG_NETWORK || DEBUG_ALL) {
      Serial.println("[MQTT] Client task started.");
    }
    return true;
  }

  if (DEBUG_NETWORK || DEBUG_ALL) {
    Serial.printf("[MQTT] ERROR: start failed (0x%x)\n", (unsigned)err);
  }
  return false;
}

void FMSMqttClient::stopClient() {
  if (_mqtt == nullptr || !_isRunning) {
    _isConnected = false;
    return;
  }

  esp_err_t err = esp_mqtt_client_stop(_mqtt);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE && (DEBUG_NETWORK || DEBUG_ALL)) {
    Serial.printf("[MQTT] WARN: stop returned 0x%x\n", (unsigned)err);
  }
  _isRunning = false;
  _isConnected = false;
}

// ================= PUBLIC: connect =================
bool FMSMqttClient::connect() {
  if (_isConnected) {
    return true;
  }
  if (_mqtt == nullptr) {
    return false;
  }
  if (!_isRunning) {
    return startClient();
  }
  return esp_mqtt_client_reconnect(_mqtt) == ESP_OK;
}

// ================= PUBLIC: disconnect =================
void FMSMqttClient::disconnect() {
  stopClient();
}

// ================= PUBLIC: setCallback =================
void FMSMqttClient::setCallback(void (*callback)(char *, uint8_t *,
                                                 unsigned int)) {
  _mqttCallback = callback;
}

// ================= PUBLIC: setCommandCallback =================
void FMSMqttClient::setCommandCallback(CommandCallback cb) {
  _onCommandReceived = cb;
}

// ================= PUBLIC: subscribeConfig =================
void FMSMqttClient::subscribeConfig() {
  if (_isConnected && _mqtt) {
    esp_mqtt_client_subscribe(_mqtt, _topicConfigSet, 1);
    Serial.printf("[MQTT] Subscribed to config: %s\n", _topicConfigSet);
  }
}

// ================= PUBLIC: subscribeCommand =================
void FMSMqttClient::subscribeCommand() {
  if (_isConnected && _mqtt) {
    esp_mqtt_client_subscribe(_mqtt, _topicCommand, 1);
    Serial.printf("[MQTT] Subscribed to command: %s\n", _topicCommand);
  }
}

// ================= INTERNAL: handleNativeEvent =================
void FMSMqttClient::_handleNativeEvent(int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    _isConnected = true;
    _reconnectCount++;
    if (DEBUG_NETWORK || DEBUG_ALL) {
      Serial.println("âœ… MQTT CONNECTED (WSS)");
    }
    
    // Subscribe to topics
    subscribeConfig();
    subscribeCommand(); 
    
    esp_mqtt_client_subscribe(_mqtt, _topicChat, MQTT_CHAT_QOS);
    Serial.printf("[MQTT] Subscribed to chat: %s\n", _topicChat);

    // Subscribe to Serial Remote topic (MASTER)
    esp_mqtt_client_subscribe(_mqtt, _topicSerial, 1);
    Serial.printf("[MQTT] Subscribed to serial remote: %s\n", _topicSerial);
    break;

  case MQTT_EVENT_DISCONNECTED:
    _isConnected = false;
    Serial.println("âš ï¸ MQTT DISCONNECTED");
    break;

  case MQTT_EVENT_DATA: {
    if (event == nullptr || event->topic == nullptr || event->topic_len <= 0 ||
        event->data == nullptr || event->data_len <= 0) {
      break;
    }

    auto topicEquals = [&](const char *expected) -> bool {
      size_t expectedLen = strlen(expected);
      return event->topic_len == (int)expectedLen &&
             strncmp(event->topic, expected, expectedLen) == 0;
    };
    auto topicEndsWith = [&](const char *suffix) -> bool {
      size_t suffixLen = strlen(suffix);
      if (event->topic_len < (int)suffixLen) {
        return false;
      }
      return strncmp(event->topic + event->topic_len - suffixLen, suffix,
                     suffixLen) == 0;
    };

    if (topicEquals(_topicChat)) {
      lcdQueueIncomingChatMessage(event->data, (size_t)event->data_len);
      Serial.println("[MQTT] Chat message received");
    } else if (topicEquals(_topicCommand) || topicEndsWith("/log/send")) {
      DynamicJsonDocument doc(768);
      DeserializationError err =
          deserializeJson(doc, event->data, (size_t)event->data_len);
      if (!err && doc.containsKey("cmd")) {
        const char *cmdStr = doc["cmd"] | "";
        if (cmdStr[0] != '\0') {
          Serial.printf("[MQTT] Remote Command Received: %s\n", cmdStr);
          if (_onCommandReceived) {
            _onCommandReceived(String(cmdStr));
          }
        }
      }
    } else if (topicEquals(_topicSerial)) {
      // â”€â”€â”€ MQTT Serial Remote â”€â”€â”€
      // Payload is plain text, not JSON.
      char cmdBuf[64];
      size_t len = (size_t)event->data_len;
      if (len > sizeof(cmdBuf) - 1) len = sizeof(cmdBuf) - 1;
      memcpy(cmdBuf, event->data, len);
      cmdBuf[len] = '\0';

      String cmdStr = String(cmdBuf);
      cmdStr.trim();

      if (cmdStr.length() > 0) {
        Serial.printf("[MQTT] Remote Serial Command: %s\n", cmdStr.c_str());
        if (_onCommandReceived) {
          _onCommandReceived(cmdStr);
        }
      }
    } else if (topicEquals("fms/web")) {
      DynamicJsonDocument doc(512);
      DeserializationError err =
          deserializeJson(doc, event->data, (size_t)event->data_len);
      if (!err) {
        const char *action = doc["action"] | "";
        if (strcmp(action, "sync_geo") == 0 ||
            strcmp(action, "sync_geo_edit") == 0 ||
            strcmp(action, "sync_geo_hapus") == 0) {
          Serial.println("[MQTT] Received global sync_geo trigger");
          if (_onCommandReceived) {
            _onCommandReceived("sync_geo");
          }
        }
      }
    } else if (topicEquals(_topicConfigSet)) {
      if (_mqttCallback) {
        _mqttCallback(_topicConfigSet, (uint8_t *)event->data,
                      event->data_len);
      }
    }
    break;
  }

  case MQTT_EVENT_ERROR:
    Serial.println("âŒ MQTT EVENT ERROR");
    if (event && event->error_handle && (DEBUG_NETWORK || DEBUG_ALL)) {
      esp_mqtt_error_codes_t *err = event->error_handle;
      Serial.printf(
          "[MQTT][ERR] type=%d tls=0x%x stack=0x%x cert=0x%x sock=%d\n",
          (int)err->error_type, (unsigned int)err->esp_tls_last_esp_err,
          (unsigned int)err->esp_tls_stack_err,
          (unsigned int)err->esp_tls_cert_verify_flags,
          err->esp_transport_sock_errno);
    }
    break;

  default:
    break;
  }
}

// ================= PRIVATE: logWifiSignalStrength =================
void FMSMqttClient::logWifiSignalStrength() {
  if (WiFi.status() != WL_CONNECTED) {
    if (DEBUG_NETWORK || DEBUG_ALL) {
      Serial.printf("[WiFi] Not connected (status=%d)\n", (int)WiFi.status());
    }
    return;
  }

  int8_t rssi = WiFi.RSSI();
  _lastRSSI = rssi;

  const char *quality;
  if (rssi > -50) {
    quality = "Excellent";
  } else if (rssi > -60) {
    quality = "Good";
  } else if (rssi > -70) {
    quality = "Fair";
  } else {
    quality = "Poor - may cause issues!";
  }

  if (DEBUG_NETWORK || DEBUG_ALL) {
    String ssid = WiFi.SSID();
    String ip = WiFi.localIP().toString();
    Serial.printf("[WiFi] SSID: %s | IP: %s | RSSI: %d dBm (%s)\n",
                  ssid.c_str(), ip.c_str(), rssi, quality);
  }
}

// ================= PUBLIC: publishData =================
bool FMSMqttClient::publishData(const TelemetryData &data) {
  if (!_isConnected || _mqtt == nullptr)
    return false;
  String payload = buildDataPayload(data);
  int msg_id =
      esp_mqtt_client_publish(_mqtt, _topicData, payload.c_str(), 0, 1, 0);
  if (msg_id >= 0) {
    _publishCount++;
    return true;
  }
  _failCount++;
  return false;
}

// ================= PUBLIC: publishLog =================
bool FMSMqttClient::publishLog(const FMSLogEntry *logs, uint8_t count) {
  if (count == 0 || !_isConnected || _mqtt == nullptr)
    return false;
  String payload = buildLogPayload(logs, count);
  int msg_id =
      esp_mqtt_client_publish(_mqtt, _topicLog, payload.c_str(), 0, 1, 0);
  if (msg_id >= 0) {
    _publishCount++;
    return true;
  }
  _failCount++;
  return false;
}

bool FMSMqttClient::publishLogEntry(const FMSLogEntry &entry) {
  return publishLog(&entry, 1);
}

// ================= PUBLIC: publishConfig =================
bool FMSMqttClient::publishConfig(const FMSConfig &config) {
  if (!_isConnected || _mqtt == nullptr)
    return false;
  String payload = buildConfigPayload(config);
  int msg_id =
      esp_mqtt_client_publish(_mqtt, _topicConfig, payload.c_str(), 0, 1, 0);
  if (msg_id >= 0) {
    _publishCount++;
    return true;
  }
  _failCount++;
  return false;
}

// ================= PUBLIC: publishResponse =================
bool FMSMqttClient::publishResponse(const String &response) {
  if (!_isConnected || _mqtt == nullptr)
    return false;

  // Respon dikirim ke topik LOG, bukan topik COMMAND (mencegah loop)
  // Format log harian akan otomatis menangkap ini
  int msg_id =
      esp_mqtt_client_publish(_mqtt, _topicLog, response.c_str(), 0, 1, 0);
  return msg_id >= 0;
}

bool FMSMqttClient::publishToTopic(const char *topic, const String &payload,
                                   uint8_t qos, bool retain) {
  if (!_isConnected || _mqtt == nullptr || topic == nullptr || topic[0] == '\0') {
    return false;
  }

  int msg_id = esp_mqtt_client_publish(_mqtt, topic, payload.c_str(), 0, qos,
                                       retain ? 1 : 0);
  if (msg_id >= 0) {
    _publishCount++;
    return true;
  }

  _failCount++;
  return false;
}

// Statistics
uint32_t FMSMqttClient::getPublishCount() const { return _publishCount; }
uint32_t FMSMqttClient::getFailCount() const { return _failCount; }
uint32_t FMSMqttClient::getReconnectCount() const { return _reconnectCount; }

// ================= PRIVATE: buildTopics =================
void FMSMqttClient::buildTopics() {
  snprintf(_topicData, sizeof(_topicData), "%s/%s/%s", MQTT_TOPIC_BASE,
           _vehicleId, MQTT_TOPIC_DATA);
  snprintf(_topicLog, sizeof(_topicLog), "%s/%s/%s", MQTT_TOPIC_BASE,
           _vehicleId, MQTT_TOPIC_LOG);
  snprintf(_topicConfig, sizeof(_topicConfig), "%s/%s/%s", MQTT_TOPIC_BASE,
           _vehicleId, MQTT_TOPIC_CONFIG);
  snprintf(_topicConfigSet, sizeof(_topicConfigSet), "%s/%s/%s",
           MQTT_TOPIC_BASE, _vehicleId, MQTT_TOPIC_CONFIG_SET);
  snprintf(_topicCommand, sizeof(_topicCommand), "%s/%s/%s", MQTT_TOPIC_BASE,
           _vehicleId, MQTT_TOPIC_COMMAND);
  snprintf(_topicChat, sizeof(_topicChat), "%s/%s/%s", MQTT_TOPIC_BASE,
           _vehicleId, MQTT_TOPIC_CHAT);

  // Topic Serial khusus master: fms/master_nfc/serial
  snprintf(_topicSerial, sizeof(_topicSerial), "%s/%s", MQTT_TOPIC_MASTER,
           MQTT_TOPIC_SERIAL);
}

// ================= PAYLOAD BUILDERS (EXACT MATCH) =================
String FMSMqttClient::buildDataPayload(const TelemetryData &data) {
  DynamicJsonDocument doc(2304);
  doc["type"] = "data";
  doc["device_id"] = data.device_id;
  doc["seq"] = data.seq;
  doc["timestamp_ms"] = data.timestamp_ms;

  JsonObject datetime = doc.createNestedObject("datetime");
  datetime["rtc"] = data.datetime_rtc;
  datetime["gps"] = data.datetime_gps;
  datetime["best"] = data.datetime_best;
  datetime["rtc_valid"] = data.rtc_valid;
  datetime["gps_valid"] = data.gps_time_valid;

  JsonObject gps = doc.createNestedObject("gps");
  gps["lat"] = round(data.gps_lat * 1000000.0) / 1000000.0;
  gps["lon"] = round(data.gps_lon * 1000000.0) / 1000000.0;
  gps["speed_kph"] = round(data.gps_speed_kph * 10) / 10.0;
  gps["altitude_m"] = round(data.gps_altitude_m);
  gps["hdop"] = round(data.gps_hdop * 10) / 10.0;
  gps["satellites"] = data.gps_satellites;
  gps["valid"] = data.gps_valid;

  JsonObject geofence = doc.createNestedObject("geofence");
  geofence["id"] = data.geofence_id;
  geofence["name"] = data.geofence_name;

  // --- Barometer (Original structure + temp_status) ---
  JsonObject baro = doc.createNestedObject("barometer");
  baro["temp_c"] = round(data.baro_temp_c * 10) / 10.0;
  baro["temp_status"] = data.box_temp_status; // NEW: Health status
  baro["pressure_hpa"] = round(data.baro_pressure_hpa * 10) / 10.0;
  baro["alt_abs_m"] = round(data.baro_alt_abs_m * 10) / 10.0;
  baro["alt_rel_m"] = round(data.baro_alt_rel_m * 10) / 10.0;
  baro["vspeed_ms"] = round(data.baro_vspeed_ms * 100) / 100.0;
  baro["motion_state"] = data.baro_motion_state;

  // --- IMU (Original structure + new nested objects) ---
  JsonObject imu = doc.createNestedObject("imu");
  imu["ready"] = data.imu_ready;
  if (data.imu_ready) {
    JsonObject accel = imu.createNestedObject("accel");
    accel["x"] = round(data.imu_accel_x);
    accel["y"] = round(data.imu_accel_y);
    accel["z"] = round(data.imu_accel_z);
    JsonObject gyro = imu.createNestedObject("gyro");
    gyro["x"] = round(data.imu_gyro_x * 10) / 10.0;
    gyro["y"] = round(data.imu_gyro_y * 10) / 10.0;
    gyro["z"] = round(data.imu_gyro_z * 10) / 10.0;
    JsonObject mag = imu.createNestedObject("mag");
    mag["x"] = round(data.imu_mag_x);
    mag["y"] = round(data.imu_mag_y);
    mag["z"] = round(data.imu_mag_z);
    // NEW: Orientation (Euler Angles)
    JsonObject orientation = imu.createNestedObject("orientation");
    orientation["pitch"] = round(data.imu_pitch * 10) / 10.0;
    orientation["roll"] = round(data.imu_roll * 10) / 10.0;
    orientation["heading"] = round(data.imu_heading * 10) / 10.0;
    // NEW: Linear Acceleration (gravity-free)
    JsonObject lin_accel = imu.createNestedObject("lin_accel");
    lin_accel["x"] = round(data.imu_lin_acc_x);
    lin_accel["y"] = round(data.imu_lin_acc_y);
    lin_accel["z"] = round(data.imu_lin_acc_z);
    // NEW: Calibration Status
    JsonObject calib = imu.createNestedObject("calibration");
    calib["sys"] = data.imu_cal_sys;
    calib["gyro"] = data.imu_cal_gyro;
    calib["accel"] = data.imu_cal_accel;
    calib["mag"] = data.imu_cal_mag;
    // Keep heading at root for backward compatibility
    imu["heading"] = round(data.imu_heading * 10) / 10.0;
    imu["accel_rms"] = round(data.imu_accel_rms * 1000) / 1000.0;
  }

  JsonObject power = doc.createNestedObject("power");
  JsonObject ina0 = power.createNestedObject("ina0");
  for (int i = 0; i < 3; i++) {
    char chName[4];
    snprintf(chName, sizeof(chName), "ch%d", i + 1);
    JsonObject ch = ina0.createNestedObject(chName);
    float v = (i == 0)   ? data.ina0_ch1_voltage
              : (i == 1) ? data.ina0_ch2_voltage
                         : data.ina0_ch3_voltage;
    float a = (i == 0)   ? data.ina0_ch1_current
              : (i == 1) ? data.ina0_ch2_current
                         : data.ina0_ch3_current;
    ch["v"] = round(v * 100) / 100.0;
    ch["a"] = round(a * 1000) / 1000.0;
  }
  JsonObject ina1 = power.createNestedObject("ina1");
  for (int i = 0; i < 3; i++) {
    char chName[4];
    snprintf(chName, sizeof(chName), "ch%d", i + 1);
    JsonObject ch = ina1.createNestedObject(chName);
    float v = (i == 0)   ? data.ina1_ch1_voltage
              : (i == 1) ? data.ina1_ch2_voltage
                         : data.ina1_ch3_voltage;
    float a = (i == 0)   ? data.ina1_ch1_current
              : (i == 1) ? data.ina1_ch2_current
                         : data.ina1_ch3_current;
    ch["v"] = round(v * 100) / 100.0;
    ch["a"] = round(a * 1000) / 1000.0;
  }

  JsonObject fuel = doc.createNestedObject("fuel");
  fuel["freq_hz"] = round(data.fuel_frequency_hz);
  fuel["percent"] = round(data.fuel_percent * 10) / 10.0;
  fuel["volume_l"] = round(data.fuel_volume_l * 10) / 10.0;
  fuel["consumption_l"] = round(data.fuel_consumption_l * 100) / 100.0;
  fuel["rate_lph"] = round(data.fuel_rate_lph * 100) / 100.0;
  fuel["runtime_h"] = round(data.fuel_runtime_h * 10) / 10.0;
  fuel["stolen_l"] = round(data.fuel_stolen_l * 100) / 100.0;
  fuel["anomaly"] = data.fuel_anomaly;
  fuel["signal_ok"] = data.fuel_signal_ok;

  JsonObject nfc = doc.createNestedObject("nfc");
  nfc["last_uid"] = data.nfc_last_uid;
  nfc["last_tap_ms"] = data.nfc_last_tap_ms;
  nfc["tag_present"] = data.nfc_tag_present;

  JsonObject vehicle = doc.createNestedObject("vehicle");
  vehicle["engine_on"] = data.engine_on;
  vehicle["moving"] = data.vehicle_moving;

  JsonObject operatorInput = doc.createNestedObject("operator_input");
  operatorInput["lokasi_awal"] = data.lokasi_awal;
  operatorInput["lokasi_akhir"] = data.lokasi_akhir;
  operatorInput["jenis_muatan"] = data.jenis_muatan;
  operatorInput["status_trip"] = data.status_trip;

  JsonObject storage = doc.createNestedObject("storage");
  storage["status"] = data.sd_status;
  storage["free_mb"] = data.sd_free_mb;

  String payload;
  serializeJson(doc, payload);
  return payload;
}

String FMSMqttClient::buildLogPayload(const FMSLogEntry *logs, uint8_t count) {
  DynamicJsonDocument doc(2048);
  doc["type"] = "log";
  doc["device_id"] = _deviceId;
  doc["batch_size"] = count;
  JsonArray logsArray = doc.createNestedArray("logs");
  for (uint8_t i = 0; i < count; i++) {
    JsonObject entry = logsArray.createNestedObject();
    entry["ts"] = logs[i].timestamp_ms;
    entry["dt"] = logs[i].datetime;
    const char *lvl = "INFO";
    switch (logs[i].level) {
    case FMS_LOG_DEBUG:
      lvl = "DEBUG";
      break;
    case FMS_LOG_INFO:
      lvl = "INFO";
      break;
    case FMS_LOG_WARN:
      lvl = "WARN";
      break;
    case FMS_LOG_ERROR:
      lvl = "ERROR";
      break;
    case FMS_LOG_CRITICAL:
      lvl = "CRITICAL";
      break;
    }
    entry["level"] = lvl;
    entry["src"] = logs[i].source;
    entry["msg"] = logs[i].message;
    if (logs[i].extra[0] != '\0')
      entry["data"] = logs[i].extra;
  }
  String payload;
  serializeJson(doc, payload);
  return payload;
}

String FMSMqttClient::buildConfigPayload(const FMSConfig &config) {
  DynamicJsonDocument doc(1280);
  doc["type"] = "config";
  doc["device_id"] = config.device_id;
  doc["version"] = config.version;

  JsonObject fuel = doc.createNestedObject("fuel");
  fuel["freq_empty"] = config.fuel_freq_empty;
  fuel["freq_full"] = config.fuel_freq_full;
  fuel["tank_capacity"] = config.fuel_tank_capacity;

  JsonObject thresholds = doc.createNestedObject("thresholds");
  thresholds["engine_on_v"] = config.engine_on_voltage;
  thresholds["moving_kph"] = config.moving_speed_kph;
  thresholds["stopped_kph"] = config.stopped_speed_kph;
  thresholds["moving_accel_g"] = config.moving_accel_g;
  thresholds["stopped_accel_g"] = config.stopped_accel_g;

  JsonObject anomaly = doc.createNestedObject("anomaly");
  anomaly["siphon_pct"] = config.siphon_drop_pct;
  anomaly["slow_5m_pct"] = config.slow_theft_5m_pct;
  anomaly["slow_10m_pct"] = config.slow_theft_10m_pct;
  anomaly["sudden_pct"] = config.sudden_drop_pct;
  anomaly["excessive_pct"] = config.excessive_drop_pct;

  JsonObject intervals = doc.createNestedObject("intervals");
  intervals["data_ms"] = config.data_send_interval_ms;
  intervals["log_batch"] = config.log_batch_size;
  intervals["log_timeout_ms"] = config.log_send_timeout_ms;

  String payload;
  serializeJson(doc, payload);
  return payload;
}

bool FMSMqttClient::publishMaster(const String &payload) {
  return publishToTopic(MQTT_TOPIC_MASTER, payload);
}

bool FMSMqttClient::publishMasterData(const TelemetryData &data) {
  String payload = buildDataPayload(data);
  return publishMaster(payload);
}

#endif // USE_MQTT_PURE
