#ifndef FMS_MQTT_H
#define FMS_MQTT_H

#include "fms_protocol.h"
#include <Arduino.h>

#if USE_MQTT_PURE

#include <ArduinoJson.h>
#include <WiFi.h>
#include <functional>

// Native ESP-IDF MQTT
extern "C" {
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "mqtt_client.h"
}

#include "config_manager.h"
#include "log_manager.h"
#include "telemetry.h"

// ================= MQTT CONFIG =================
// Using WSS (WebSocket Secure) on port 443
#include "config.h"

#define MQTT_TOPIC_BASE "fms"
#ifndef MQTT_TOPIC_DATA
#define MQTT_TOPIC_DATA "data"
#endif
#define MQTT_TOPIC_LOG "log"
#define MQTT_TOPIC_CONFIG "config"
#define MQTT_TOPIC_CONFIG_SET "config/set"
#define MQTT_TOPIC_COMMAND "cmd"
#define MQTT_TOPIC_CHAT "chat"

// Topic khusus master
#define MQTT_TOPIC_MASTER "fms/master_nfc"
#define MQTT_TOPIC_SERIAL "serial"

#define MQTT_CHAT_QOS 1

// ================= TUNING PARAMETERS =================
#define MQTT_KEEPALIVE_S 60
#define MQTT_RECONNECT_MS 30000
#define MQTT_NETWORK_TIMEOUT_MS 15000
#define MQTT_TASK_STACK_SIZE 16384
#ifndef MQTT_BUFFER_SIZE
#define MQTT_BUFFER_SIZE 4096
#endif

// ================= FMS MQTT CLIENT CLASS =================
class FMSMqttClient {
public:
  typedef std::function<void(const String &cmd)> CommandCallback;

  FMSMqttClient();

  // Inisialisasi client Native ESP-IDF WSS
  void begin(const char *vehicleId, const char *deviceId);

  // Loop utama
  void loop();

  // Connection status
  bool isConnected();
  bool isRunning() const;
  bool startClient();
  void stopClient();
  bool connect();
  void disconnect();

  // Callback setter
  void setCallback(void (*callback)(char *, uint8_t *, unsigned int));
  void setCommandCallback(CommandCallback cb);

  // Subscribe functions
  void subscribeConfig();
  void subscribeCommand();

  // ================= PUBLISH FUNCTIONS =================
  bool publishData(const TelemetryData &data);
  bool publishLog(const FMSLogEntry *logs, uint8_t count);
  bool publishLogEntry(const FMSLogEntry &entry);
  bool publishConfig(const FMSConfig &config);
  bool publishResponse(const String &response);

  // Publish langsung ke /fms/master
  bool publishMaster(const String &payload);
  bool publishMasterData(const TelemetryData &data);

  bool publishToTopic(const char *topic, const String &payload, uint8_t qos = 1,
                      bool retain = false);

  // Payload Builders
  String buildDataPayload(const TelemetryData &data);
  String buildLogPayload(const FMSLogEntry *logs, uint8_t count);
  String buildConfigPayload(const FMSConfig &config);

  // ================= STATISTICS =================
  uint32_t getPublishCount() const;
  uint32_t getFailCount() const;
  uint32_t getReconnectCount() const;

  // Internal event handler bridge
  void _handleNativeEvent(int32_t event_id, void *event_data);

private:
  esp_mqtt_client_handle_t _mqtt;
  bool _isConnected;
  bool _isRunning;

  char _vehicleId[32];
  char _deviceId[32];
  char _clientId[64];

  // Topics
  char _topicData[128];
  char _topicLog[128];
  char _topicConfig[128];
  char _topicConfigSet[128];
  char _topicCommand[128];
  char _topicSerial[128];
  char _topicChat[128];

  void (*_mqttCallback)(char *, uint8_t *, unsigned int);
  CommandCallback _onCommandReceived;

  // Statistics
  uint32_t _publishCount;
  uint32_t _failCount;
  uint32_t _reconnectCount;

  // Helpers
  void buildTopics();
  void logWifiSignalStrength();

private:
  uint32_t _lastWifiRSSILog;
  int8_t _lastRSSI;
};

// ================= GLOBAL INSTANCE =================
extern FMSMqttClient fmsMqtt;

#endif // USE_MQTT_PURE

#endif // FMS_MQTT_H