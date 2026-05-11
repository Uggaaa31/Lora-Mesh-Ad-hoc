#include <SPI.h>
#include <WiFi.h>
#include <RH_RF95.h>
#include <time.h>
#include <sys/time.h>

#include <ArduinoJson.h>

#include "config.h"
#include "LoRa_Packet.h"
#include "AODV_Routing.h"
#include "WebConfig.h"

extern "C" {
  #include "esp_event.h"
  #include "mqtt_client.h"
  #include "esp_crt_bundle.h"
}

enum QueueItemKind : uint8_t {
    QUEUE_KIND_SENSOR = 1,
    QUEUE_KIND_FATIGUE_IMU = 2,
    QUEUE_KIND_SAFETY_CONDITION = 3,
    QUEUE_KIND_VEHICLE_TELEMETRY = 4
};

enum FatigueAlarmCode : uint8_t {
    FATIGUE_STATUS_NORMAL = 0,
    FATIGUE_STATUS_LELAH = 1,
    FATIGUE_STATUS_TERTIDUR = 2
};

struct MQTTQueueItem {
    uint8_t kind;
    uint8_t sourceNodeID;
    int8_t rssi;
    uint8_t hopCount;
    SensorDataPayload sensorData;
    ImuFatiguePayload fatigueImu;
    SafetyConditionPayload safetyCondition;
    VehicleTelemetryPayload vehicleTelemetry;
};

struct NodeStats {
    uint32_t packetsReceived;
    uint32_t latencySum;
    uint32_t latencyMin;
    uint32_t latencyMax;
    uint32_t latencyCount;
    char nodeName[16];

    NodeStats() {
        packetsReceived = 0;
        latencySum = 0;
        latencyMin = UINT32_MAX;
        latencyMax = 0;
        latencyCount = 0;
        memset(nodeName, 0, sizeof(nodeName));
    }
};

RH_RF95 rf95(LORA_CS_PIN, LORA_DIO0_PIN);
AODVRouting aodv(NODE_ID);
LoRaPacketHandler packetHandler;
static esp_mqtt_client_handle_t mqtt_client = nullptr;
QueueHandle_t mqttQueue = nullptr;
TaskHandle_t LoRaTaskHandle = nullptr;

bool wifiConnected = false;
volatile bool mqttConnected = false;
unsigned long lastWiFiCheck = 0;
unsigned long lastMqttReconnectAttempt = 0;
volatile unsigned long packetsReceivedTotal = 0;
volatile unsigned long packetsSentToServer = 0;

bool ntpSynced = false;
uint64_t ntpBaseEpochMs = 0;
unsigned long ntpBaseMillis = 0;
volatile bool timeSyncRequested = false;

volatile bool fatigueStatusPending = false;
volatile bool fatigueStatusReadyToSend = false;
volatile uint8_t pendingFatigueStatusCode = FATIGUE_STATUS_NORMAL;
uint8_t lastSentFatigueStatusCode = FATIGUE_STATUS_NORMAL;
bool hasSentFatigueStatus = false;
unsigned long lastFatigueRouteAttempt = 0;
uint32_t fatigueStatusSequence = 0;

NodeStats nodeStats[MAX_TRACKED_NODES];
unsigned long observationStartTime = 0;
unsigned long lastQoSPrint = 0;
uint8_t observationRound = 0;

LoRaRuntimeCfg gwRuntimeCfg;

void initLoRa();
void initWiFi();
void initMQTT();
void initNTP();
void checkWiFi();
void receivePackets();
void handleReceivedPacket(const LoRaPacket& packet);
void sendPacketCallback(const LoRaPacket& packet);
void enqueueSensorData(const SensorDataPayload& data, uint8_t src, int8_t rssi, uint8_t hops);
void enqueueFatigueImuData(const ImuFatiguePayload& data, uint8_t src, int8_t rssi, uint8_t hops);
void enqueueSafetyConditionData(const SafetyConditionPayload& data, uint8_t src, int8_t rssi, uint8_t hops);
void enqueueVehicleTelemetryData(const VehicleTelemetryPayload& data, uint8_t src, int8_t rssi, uint8_t hops);
void publishQueueItem(const MQTTQueueItem& item);
void publishSensorQueueItem(const MQTTQueueItem& item, const char* name, uint32_t latency, bool latencyValid);
void publishFatigueQueueItem(const MQTTQueueItem& item);
void publishSafetyConditionQueueItem(const MQTTQueueItem& item);
void publishVehicleTelemetryQueueItem(const MQTTQueueItem& item);
void sendTimeSyncPacket();
void sendPendingFatigueStatus();
void requestFatigueStatus(FatigueAlarmCode code);
void handleFatigueStatusMessage(const char* payload);
FatigueAlarmCode parseFatigueStatus(const String& input);
const char* fatigueAlarmText(FatigueAlarmCode code);
void handleSerialCLI();
void printQoSStats();
void printCSVStats();
void resetQoSStats();
void printStatus();
void updateNodeStats(uint8_t nodeId, const char* name, uint32_t latency, bool latencyValid);
const char* getNodeName(uint8_t nodeId);
uint32_t getExpectedPacketsForNode(uint8_t nodeId, unsigned long durationMs);
uint64_t getEpochMs64();
uint32_t getEpochMsLow32();
static void mqtt_event_handler(void*, esp_event_base_t, int32_t, void*);
void TaskLoRa(void*);

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("Gateway Firmware: LoRa + MQTT Bridge");
    Serial.println("========================================");

    gwRuntimeCfg = GWWebConfig::getConfig(
        LORA_SPREADING_FACTOR,
        LORA_BANDWIDTH,
        LORA_CODING_RATE,
        LORA_TX_POWER,
        LORA_USE_RFO,
        0
    );

    Serial.printf("Runtime: SF=%d BW=%lukHz CR=4/%d Pwr=%ddBm\n",
                  gwRuntimeCfg.sf,
                  (unsigned long)gwRuntimeCfg.bwKHz,
                  gwRuntimeCfg.cr,
                  gwRuntimeCfg.txPower);
    Serial.printf("WiFi target: %s\n", GWWebConfig::getWiFiSSID(gwRuntimeCfg));

    mqttQueue = xQueueCreate(20, sizeof(MQTTQueueItem));
    if (!mqttQueue) {
        Serial.println("ERROR: Failed to create MQTT queue");
        while (true) {}
    }

    initLoRa();
    aodv.begin();
    aodv.onSendPacket = sendPacketCallback;

    xTaskCreatePinnedToCore(TaskLoRa, "LoRaTask", 10000, nullptr, 1, &LoRaTaskHandle, 1);

    initWiFi();
    if (wifiConnected) {
        initMQTT();
        initNTP();
    }

    GWWebConfig::begin(WiFi.localIP().toString(), WiFi.softAPIP().toString(), gwRuntimeCfg);

    resetQoSStats();
    printStatus();
}

void loop() {
    unsigned long now = millis();

    if (now - lastWiFiCheck > 30000) {
        checkWiFi();
        lastWiFiCheck = now;
    }

    if (wifiConnected && mqtt_client == nullptr) {
        initMQTT();
        initNTP();
    }

    if (wifiConnected && mqtt_client != nullptr && !mqttConnected &&
        (now - lastMqttReconnectAttempt > 10000)) {
        esp_mqtt_client_reconnect(mqtt_client);
        lastMqttReconnectAttempt = now;
    }

    if (wifiConnected && mqttConnected) {
        MQTTQueueItem item;
        int count = 0;
        while (xQueueReceive(mqttQueue, &item, 0) == pdTRUE && count++ < 5) {
            publishQueueItem(item);
        }
    }

    static unsigned long lastTimeSync = 0;
    if (ntpSynced && now - lastTimeSync > TIMESYNC_INTERVAL_MS) {
        timeSyncRequested = true;
        lastTimeSync = now;
    }

    GWWebConfig::handle();

    if (now - lastQoSPrint > OBSERVATION_PERIOD_MS) {
        observationRound++;
        printQoSStats();
        printCSVStats();
        resetQoSStats();
        lastQoSPrint = now;
    }

    static unsigned long lastStatus = 0;
    if (now - lastStatus > 60000) {
        printStatus();
        lastStatus = now;
    }

    handleSerialCLI();
    delay(10);
}

void TaskLoRa(void*) {
    Serial.printf("[LoRa] Task on Core %d\n", xPortGetCoreID());

    for (;;) {
        aodv.update();
        receivePackets();

        if (timeSyncRequested) {
            timeSyncRequested = false;
            sendTimeSyncPacket();
        }

        if (fatigueStatusPending && fatigueStatusReadyToSend) {
            sendPendingFatigueStatus();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void initLoRa() {
    Serial.println("[LoRa] Initializing");
    pinMode(LORA_RST_PIN, OUTPUT);
    digitalWrite(LORA_RST_PIN, HIGH);
    pinMode(LORA_DIO0_PIN, INPUT);

    digitalWrite(LORA_RST_PIN, LOW);
    delay(10);
    digitalWrite(LORA_RST_PIN, HIGH);
    delay(10);

    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);

    if (!rf95.init()) {
        Serial.println("[LoRa] ERROR: init failed");
        while (true) {}
    }
    if (!rf95.setFrequency(LORA_FREQUENCY)) {
        Serial.println("[LoRa] ERROR: set frequency failed");
        while (true) {}
    }

    float bwHz = gwRuntimeCfg.bwKHz * 1000.0f;
    int8_t pwr = gwRuntimeCfg.useRFO
        ? constrain(gwRuntimeCfg.txPower, 0, 15)
        : constrain(gwRuntimeCfg.txPower, 2, 20);

    rf95.setTxPower(pwr, gwRuntimeCfg.useRFO);
    rf95.setSpreadingFactor(gwRuntimeCfg.sf);
    rf95.setSignalBandwidth(bwHz);
    rf95.setCodingRate4(gwRuntimeCfg.cr);
    rf95.setPreambleLength(LORA_PREAMBLE_LENGTH);

    Serial.printf("[LoRa] OK SF=%d BW=%.0fkHz CR=4/%d Pwr=%ddBm (%s)\n",
                  gwRuntimeCfg.sf,
                  bwHz / 1000.0f,
                  gwRuntimeCfg.cr,
                  pwr,
                  gwRuntimeCfg.useRFO ? "RFO" : "PA_BOOST");
}

void initWiFi() {
    const char* ssid = GWWebConfig::getWiFiSSID(gwRuntimeCfg);
    const char* password = GWWebConfig::getWiFiPassword(gwRuntimeCfg);

    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.softAP(GATEWAY_CONFIG_SSID);
    delay(200);

    Serial.printf("[WiFi] Config SSID: %s\n", GATEWAY_CONFIG_SSID);
    Serial.printf("[WiFi] Config URL : http://%s/\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("[WiFi] Connecting to %s\n", ssid);

    WiFi.begin(ssid, password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT) {
        delay(500);
        Serial.print(".");
    }

    wifiConnected = (WiFi.status() == WL_CONNECTED);
    if (wifiConnected) {
        Serial.printf("\n[WiFi] Connected: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] Connection failed");
    }
}

void initNTP() {
    if (!wifiConnected) {
        return;
    }

    Serial.println("[NTP] Syncing");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    time_t now = 0;
    unsigned long start = millis();
    while (now < 1000000000L && millis() - start < 10000) {
        delay(500);
        time(&now);
    }

    if (now > 1000000000L) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        ntpSynced = true;
        ntpBaseMillis = millis();
        ntpBaseEpochMs = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
        Serial.printf("[NTP] Synced epoch=%lu\n", (unsigned long)now);
    } else {
        Serial.println("[NTP] Failed");
    }
}

void initMQTT() {
    if (!wifiConnected || mqtt_client != nullptr) {
        return;
    }

    Serial.printf("[MQTT] Starting: %s\n", MQTT_URI);
    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = MQTT_URI;
    cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;

    mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, nullptr);
    esp_mqtt_client_start(mqtt_client);
}

static void mqtt_event_handler(void*, esp_event_base_t, int32_t event_id, void* event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqttConnected = true;
            Serial.println("[MQTT] Connected");
            esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_FATIGUE_STATUS, 1);
            Serial.printf("[MQTT] Subscribed: %s\n", MQTT_TOPIC_FATIGUE_STATUS);
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqttConnected = false;
            Serial.println("[MQTT] Disconnected");
            break;

        case MQTT_EVENT_DATA: {
            String topic;
            topic.reserve(event->topic_len + 1);
            for (int i = 0; i < event->topic_len; ++i) {
                topic += (char)event->topic[i];
            }

            String payload;
            payload.reserve(event->data_len + 1);
            for (int i = 0; i < event->data_len; ++i) {
                payload += (char)event->data[i];
            }

            Serial.printf("[MQTT RX] topic=%s payload=%s\n",
                          topic.c_str(),
                          payload.c_str());

            if (topic == MQTT_TOPIC_FATIGUE_STATUS) {
                handleFatigueStatusMessage(payload.c_str());
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            Serial.println("[MQTT] Error");
            if (event->error_handle) {
                Serial.printf("[MQTT] error_type=%d tls_err=0x%x sock_errno=%d\n",
                              event->error_handle->error_type,
                              event->error_handle->esp_tls_last_esp_err,
                              event->error_handle->esp_transport_sock_errno);
            }
            break;

        default:
            break;
    }
}

void checkWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiConnected) {
            wifiConnected = false;
            mqttConnected = false;
            Serial.println("[WiFi] Lost connection. Reconnecting.");
        }
        WiFi.reconnect();
    } else {
        wifiConnected = true;
    }
}

uint64_t getEpochMs64() {
    if (!ntpSynced) {
        return (uint64_t)millis();
    }

    return ntpBaseEpochMs + (uint64_t)(millis() - ntpBaseMillis);
}

uint32_t getEpochMsLow32() {
    return (uint32_t)getEpochMs64();
}

void sendTimeSyncPacket() {
    if (!ntpSynced) {
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, nullptr);

    TimeSyncPayload ts = {};
    ts.epochSeconds = (uint32_t)tv.tv_sec;
    ts.millisPart = (uint16_t)(tv.tv_usec / 1000);

    LoRaPacket packet = LoRaPacketHandler::createTimeSyncPacket(NODE_ID, ts);
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    int len = packetHandler.serializePacket(packet, buf, sizeof(buf));

    if (len > 0) {
        rf95.send(buf, len);
        rf95.waitPacketSent();
        rf95.setModeRx();
        Serial.printf("[TIMESYNC] epoch=%lu ms=%u\n",
                      (unsigned long)ts.epochSeconds,
                      ts.millisPart);
    }
}

void receivePackets() {
    if (!rf95.available()) {
        return;
    }

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
        packetsReceivedTotal++;
        LoRaPacket packet;
        if (packetHandler.deserializePacket(buf, len, packet)) {
            handleReceivedPacket(packet);
        } else {
            Serial.printf("[RX] Invalid packet len=%u RSSI=%d\n", len, rf95.lastRssi());
        }
    }
}

void handleReceivedPacket(const LoRaPacket& packet) {
    int8_t rssi = rf95.lastRssi();
    Serial.printf("[RX] Type=%s Src=%s Dst=%s NextHop=%u RSSI=%d\n",
                  packetHandler.getPacketTypeName(packet.header.packetType),
                  getNodeName(packet.header.sourceID),
                  getNodeName(packet.header.destinationID),
                  packet.header.nextHop,
                  rssi);

    switch (packet.header.packetType) {
        case PKT_TYPE_DATA:
            if (packet.header.destinationID == GATEWAY_ID &&
                packet.header.nextHop == NODE_ID) {
                if (packet.header.payloadLength != sizeof(SensorDataPayload)) {
                    Serial.println("[RX] Invalid sensor payload size");
                    break;
                }

                SensorDataPayload data;
                memcpy(&data, packet.payload, sizeof(data));
                enqueueSensorData(data, packet.header.sourceID, rssi, packet.header.hopCount);
            }
            break;

        case PKT_TYPE_FATIGUE_IMU:
            if (packet.header.destinationID == GATEWAY_ID &&
                packet.header.nextHop == NODE_ID) {
                if (packet.header.payloadLength != sizeof(ImuFatiguePayload)) {
                    Serial.println("[RX] Invalid fatigue IMU payload size");
                    break;
                }

                ImuFatiguePayload data;
                memcpy(&data, packet.payload, sizeof(data));
                if (data.packetType != PKT_TYPE_FATIGUE_IMU) {
                    Serial.println("[RX] Invalid fatigue IMU packetType in payload");
                    break;
                }
                Serial.printf("[gateway RX IMU] nodeId=%u ts=%lu pitch=%.2f roll=%.2f\n",
                              data.nodeId,
                              (unsigned long)data.ts,
                              data.pitch100 / 100.0f,
                              data.roll100 / 100.0f);
                enqueueFatigueImuData(data, packet.header.sourceID, rssi, packet.header.hopCount);
                if (packet.header.sourceID == FATIGUE_NODE_ID && fatigueStatusPending) {
                    fatigueStatusReadyToSend = true;
                }
            }
            break;

        case PKT_TYPE_SAFETY_CONDITION:
            if (packet.header.destinationID == GATEWAY_ID &&
                packet.header.nextHop == NODE_ID) {
                if (packet.header.payloadLength != sizeof(SafetyConditionPayload)) {
                    Serial.println("[RX] Invalid safety payload size");
                    break;
                }

                SafetyConditionPayload data;
                memcpy(&data, packet.payload, sizeof(data));
                if (data.packetType != PKT_TYPE_SAFETY_CONDITION) {
                    Serial.println("[RX] Invalid safety packetType in payload");
                    break;
                }
                Serial.printf("[gateway RX SAFETY] nodeId=%u ts=%lu flags=0x%02X\n",
                              data.nodeId,
                              (unsigned long)data.ts,
                              data.flags);
                enqueueSafetyConditionData(data, packet.header.sourceID, rssi, packet.header.hopCount);
            }
            break;

        case PKT_TYPE_FATIGUE_STATUS:
            // Gateway umumnya hanya mengirim packet ini (downlink), bukan menerima.
            Serial.println("[RX] FATIGUE_STATUS packet received on gateway (unexpected uplink)");
            break;

        case PKT_TYPE_VEHICLE_TELEMETRY:
            if (packet.header.destinationID == GATEWAY_ID &&
                packet.header.nextHop == NODE_ID) {
                if (packet.header.payloadLength != sizeof(VehicleTelemetryPayload)) {
                    Serial.println("[RX] Invalid vehicle telemetry payload size");
                    break;
                }

                VehicleTelemetryPayload data;
                memcpy(&data, packet.payload, sizeof(data));
                enqueueVehicleTelemetryData(data, packet.header.sourceID, rssi, packet.header.hopCount);
            }
            break;

        case PKT_TYPE_RREQ:
            aodv.handleRREQ(packet);
            break;

        case PKT_TYPE_RREP:
            aodv.handleRREP(packet);
            break;

        case PKT_TYPE_RERR:
            aodv.handleRERR(packet);
            break;

        case PKT_TYPE_HELLO:
            aodv.handleHello(packet);
            break;

        default:
            break;
    }
}

void sendPacketCallback(const LoRaPacket& packet) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    int len = packetHandler.serializePacket(packet, buf, sizeof(buf));

    delay(random(30, 150));
    if (len > 0) {
        rf95.send(buf, len);
        rf95.waitPacketSent();
        delay(10);
        rf95.setModeRx();
    }
}

void enqueueSensorData(const SensorDataPayload& data, uint8_t src, int8_t rssi, uint8_t hops) {
    if (!mqttQueue) {
        return;
    }

    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_SENSOR;
    item.sourceNodeID = src;
    item.rssi = rssi;
    item.hopCount = hops;
    item.sensorData = data;

    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("[MQTT] Queue full, sensor packet dropped");
    }
}

void enqueueFatigueImuData(const ImuFatiguePayload& data, uint8_t src, int8_t rssi, uint8_t hops) {
    if (!mqttQueue) {
        return;
    }

    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_FATIGUE_IMU;
    item.sourceNodeID = src;
    item.rssi = rssi;
    item.hopCount = hops;
    item.fatigueImu = data;

    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("[MQTT] Queue full, fatigue packet dropped");
    }
}

void enqueueSafetyConditionData(const SafetyConditionPayload& data, uint8_t src, int8_t rssi, uint8_t hops) {
    if (!mqttQueue) {
        return;
    }

    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_SAFETY_CONDITION;
    item.sourceNodeID = src;
    item.rssi = rssi;
    item.hopCount = hops;
    item.safetyCondition = data;

    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("[MQTT] Queue full, safety packet dropped");
    }
}

void enqueueVehicleTelemetryData(const VehicleTelemetryPayload& data, uint8_t src, int8_t rssi, uint8_t hops) {
    if (!mqttQueue) {
        return;
    }

    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_VEHICLE_TELEMETRY;
    item.sourceNodeID = src;
    item.rssi = rssi;
    item.hopCount = hops;
    item.vehicleTelemetry = data;

    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("[MQTT] Queue full, vehicle telemetry dropped");
    }
}

void publishQueueItem(const MQTTQueueItem& item) {
    if (!mqttConnected || !mqtt_client) {
        return;
    }

    const char* name = getNodeName(item.sourceNodeID);
    uint32_t txTimestamp = 0;
    if (item.kind == QUEUE_KIND_SENSOR) {
        txTimestamp = item.sensorData.txTimestamp;
    } else if (item.kind == QUEUE_KIND_FATIGUE_IMU) {
        txTimestamp = item.fatigueImu.ts;
    } else if (item.kind == QUEUE_KIND_SAFETY_CONDITION) {
        txTimestamp = item.safetyCondition.ts;
    } else if (item.kind == QUEUE_KIND_VEHICLE_TELEMETRY) {
        txTimestamp = item.vehicleTelemetry.txTimestamp;
    }

    uint32_t latency = 0;
    bool latencyValid = false;
    if (txTimestamp > 0) {
        uint32_t diff = getEpochMsLow32() - txTimestamp;
        if (diff < 60000UL) {
            latency = diff;
            latencyValid = true;
        }
    }

    updateNodeStats(item.sourceNodeID, name, latency, latencyValid);

    if (latencyValid) {
        Serial.printf("[LATENCY] %s -> %lu ms (hops=%u)\n",
                      name,
                      (unsigned long)latency,
                      item.hopCount);
    }

    if (item.kind == QUEUE_KIND_SENSOR) {
        publishSensorQueueItem(item, name, latency, latencyValid);
    } else if (item.kind == QUEUE_KIND_FATIGUE_IMU) {
        publishFatigueQueueItem(item);
    } else if (item.kind == QUEUE_KIND_SAFETY_CONDITION) {
        publishSafetyConditionQueueItem(item);
    } else if (item.kind == QUEUE_KIND_VEHICLE_TELEMETRY) {
        publishVehicleTelemetryQueueItem(item);
    }
}

void publishSensorQueueItem(const MQTTQueueItem& item, const char* name, uint32_t latency, bool latencyValid) {
    uint64_t rxEpochMs64 = getEpochMs64();
    uint64_t txEpochMs64 = latencyValid ? (rxEpochMs64 - latency) : 0;

    uint32_t eventTimestamp = 0;
    if (txEpochMs64 > 0) {
        eventTimestamp = (uint32_t)(txEpochMs64 / 1000ULL);
    } else if (ntpSynced) {
        // Node belum time-synced, pakai waktu penerimaan gateway sebagai fallback
        eventTimestamp = (uint32_t)(rxEpochMs64 / 1000ULL);
    } else {
        // NTP belum sync, millis() sebagai fallback agar tidak 0
        eventTimestamp = (uint32_t)(millis() / 1000UL);
    }

    StaticJsonDocument<512> doc;
    doc["nodeId"] = name;
    doc["timestamp"] = eventTimestamp;
    doc["hopCount"] = item.hopCount;
    doc["rssi"] = item.rssi;
    if (latencyValid) {
        doc["latency_ms"] = latency;
    }

    JsonObject gps = doc.createNestedObject("gps");
    gps["latitude"] = item.sensorData.gps.latitude;
    gps["longitude"] = item.sensorData.gps.longitude;
    gps["altitude"] = item.sensorData.gps.altitude;

    JsonObject imu = doc.createNestedObject("imu");
    JsonObject accel = imu.createNestedObject("accel");
    accel["x"] = item.sensorData.imu.accelX;
    accel["y"] = item.sensorData.imu.accelY;
    accel["z"] = item.sensorData.imu.accelZ;

    doc["battery"] = item.sensorData.batteryVoltage;

    String json;
    serializeJson(doc, json);

    char topic[96];
    snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_DATA, name);

    int id = esp_mqtt_client_publish(mqtt_client, topic, json.c_str(), 0, 1, 0);
    if (id != -1) {
        packetsSentToServer++;
        Serial.printf("[MQTT] Publish OK topic=%s\n", topic);
    } else {
        Serial.printf("[MQTT] Publish failed topic=%s\n", topic);
    }
}

void publishFatigueQueueItem(const MQTTQueueItem& item) {
    StaticJsonDocument<256> doc;
    doc["nodeId"] = item.fatigueImu.nodeId;
    doc["ts"] = item.fatigueImu.ts;
    doc["pitch"] = item.fatigueImu.pitch100 / 100.0f;
    doc["roll"] = item.fatigueImu.roll100 / 100.0f;
    doc["f_gx"] = item.fatigueImu.gx1000 / 1000.0f;
    doc["f_gy"] = item.fatigueImu.gy1000 / 1000.0f;
    doc["f_gz"] = item.fatigueImu.gz1000 / 1000.0f;

    char json[256];
    size_t len = serializeJson(doc, json, sizeof(json));

    int id = esp_mqtt_client_publish(
        mqtt_client,
        MQTT_TOPIC_FATIGUE_IMU,
        json,
        (int)len,
        1,
        0
    );

    if (id != -1) {
        packetsSentToServer++;
        Serial.printf("[gateway MQTT PUB IMU] topic=%s msg_id=%d payload=%s\n",
                      MQTT_TOPIC_FATIGUE_IMU,
                      id,
                      json);
    } else {
        Serial.printf("[MQTT] Publish failed topic=%s\n", MQTT_TOPIC_FATIGUE_IMU);
    }
}

void publishSafetyConditionQueueItem(const MQTTQueueItem& item) {
    StaticJsonDocument<256> doc;
    const uint8_t flags = item.safetyCondition.flags;

    doc["nodeId"] = item.safetyCondition.nodeId;
    doc["ts"] = item.safetyCondition.ts;
    doc["gpsValid"] = (flags & FLAG_GPS_VALID) != 0;
    doc["drActive"] = (flags & FLAG_DR_ACTIVE) != 0;
    doc["rolloverRisk"] = (flags & FLAG_ROLLOVER_RISK) != 0;
    doc["rollover"] = (flags & FLAG_ROLLOVER) != 0;
    doc["harshBraking"] = (flags & FLAG_HARSH_BRAKE) != 0;
    doc["overspeed"] = (flags & FLAG_OVERSPEED) != 0;
    doc["flags"] = flags;

    char json[256];
    size_t len = serializeJson(doc, json, sizeof(json));

    int id = esp_mqtt_client_publish(
        mqtt_client,
        MQTT_TOPIC_SAFETY_CONDITION,
        json,
        (int)len,
        1,
        0
    );

    if (id != -1) {
        packetsSentToServer++;
        Serial.printf("[gateway MQTT PUB SAFETY] topic=%s msg_id=%d payload=%s\n",
                      MQTT_TOPIC_SAFETY_CONDITION,
                      id,
                      json);
    } else {
        Serial.printf("[MQTT] Publish failed topic=%s\n", MQTT_TOPIC_SAFETY_CONDITION);
    }
}

void publishVehicleTelemetryQueueItem(const MQTTQueueItem& item) {
    StaticJsonDocument<1024> doc;
    doc["lat"] = item.vehicleTelemetry.latitude;
    doc["lon"] = item.vehicleTelemetry.longitude;
    doc["headingDeg"] = item.vehicleTelemetry.headingDeg;
    doc["speedMps"] = item.vehicleTelemetry.speedMps;
    doc["drActive"] = item.vehicleTelemetry.drActive;
    doc["gpsValid"] = item.vehicleTelemetry.gpsValid;
    doc["gpsSpeed"] = item.vehicleTelemetry.gpsSpeed;
    doc["gpsHeading"] = item.vehicleTelemetry.gpsHeading;
    doc["hdop"] = item.vehicleTelemetry.hdop;
    doc["satellites"] = item.vehicleTelemetry.satellites;
    doc["accelForward"] = item.vehicleTelemetry.accelForward;
    doc["ax"] = item.vehicleTelemetry.ax;
    doc["ay"] = item.vehicleTelemetry.ay;
    doc["az"] = item.vehicleTelemetry.az;
    doc["mx"] = item.vehicleTelemetry.mx;
    doc["my"] = item.vehicleTelemetry.my;
    doc["mz"] = item.vehicleTelemetry.mz;
    doc["pitch"] = item.vehicleTelemetry.pitch;
    doc["yaw"] = item.vehicleTelemetry.yaw;
    doc["roll"] = item.vehicleTelemetry.roll;
    doc["dt"] = item.vehicleTelemetry.dt;

    JsonObject calibration = doc.createNestedObject("calibration");
    calibration["sys"] = item.vehicleTelemetry.calSys;
    calibration["gyro"] = item.vehicleTelemetry.calGyro;
    calibration["accel"] = item.vehicleTelemetry.calAccel;
    calibration["mag"] = item.vehicleTelemetry.calMag;

    doc["rollover"] = item.vehicleTelemetry.rollover;
    doc["rolloverRisk"] = item.vehicleTelemetry.rolloverRisk;
    doc["harshBraking"] = item.vehicleTelemetry.harshBraking;
    doc["overspeed"] = item.vehicleTelemetry.overspeed;
    doc["note"] = "OK";

    String json;
    serializeJson(doc, json);

    int id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_BNO_DATA, json.c_str(), 0, 0, 0);
    if (id != -1) {
        packetsSentToServer++;
        Serial.printf("[MQTT] Publish OK topic=%s\n", MQTT_TOPIC_BNO_DATA);
    } else {
        Serial.printf("[MQTT] Publish failed topic=%s\n", MQTT_TOPIC_BNO_DATA);
    }
}

void requestFatigueStatus(FatigueAlarmCode code) {
    // Jangan kirim ulang jika status akhir sama dengan status terakhir yang sudah dikirim.
    if (hasSentFatigueStatus && lastSentFatigueStatusCode == code) {
        if (fatigueStatusPending && pendingFatigueStatusCode != code) {
            Serial.printf("[STATUS] Cancel pending %s, keep last sent %s\n",
                          fatigueAlarmText((FatigueAlarmCode)pendingFatigueStatusCode),
                          fatigueAlarmText(code));
            fatigueStatusPending = false;
            fatigueStatusReadyToSend = false;
        }
        return;
    }

    if (fatigueStatusPending && pendingFatigueStatusCode == code) {
        return;
    }

    pendingFatigueStatusCode = code;
    fatigueStatusPending = true;
    fatigueStatusReadyToSend = false;
    Serial.printf("[STATUS] Pending LoRa status -> %s (wait next IMU uplink)\n",
                  fatigueAlarmText(code));
}

void handleFatigueStatusMessage(const char* payload) {
    String status;
    String raw(payload);
    raw.trim();

    if (raw.length() > 0 && raw[0] == '{') {
        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, raw) == DeserializationError::Ok &&
            doc["status"].is<const char*>()) {
            status = (const char*)doc["status"];
        }
    } else {
        status = raw;
    }

    status.trim();
    status.toUpperCase();
    Serial.printf("[gateway MQTT RX FATIGUE STATUS] raw=%s parsed=%s\n",
                  raw.c_str(),
                  status.c_str());
    requestFatigueStatus(parseFatigueStatus(status));
}

FatigueAlarmCode parseFatigueStatus(const String& input) {
    if (input == "LELAH") {
        return FATIGUE_STATUS_LELAH;
    }
    if (input == "TERTIDUR") {
        return FATIGUE_STATUS_TERTIDUR;
    }
    return FATIGUE_STATUS_NORMAL;
}

const char* fatigueAlarmText(FatigueAlarmCode code) {
    switch (code) {
        case FATIGUE_STATUS_LELAH:
            return "LELAH";
        case FATIGUE_STATUS_TERTIDUR:
            return "TERTIDUR";
        case FATIGUE_STATUS_NORMAL:
        default:
            return "NORMAL";
    }
}

void sendPendingFatigueStatus() {
    if (!fatigueStatusPending || !fatigueStatusReadyToSend) {
        return;
    }

    if (!aodv.hasRouteTo(FATIGUE_NODE_ID)) {
        unsigned long now = millis();
        if (now - lastFatigueRouteAttempt >= FATIGUE_STATUS_ROUTE_RETRY_MS) {
            lastFatigueRouteAttempt = now;
            Serial.printf("[LoRa] No route to %s. Starting discovery.\n", getNodeName(FATIGUE_NODE_ID));
            aodv.initiateRouteDiscovery(FATIGUE_NODE_ID);
        }
        // Wait next IMU uplink before trying downlink again (half-duplex friendly).
        fatigueStatusReadyToSend = false;
        return;
    }

    FatigueAlarmCode code = (FatigueAlarmCode)pendingFatigueStatusCode;
    FatigueStatusPayload payload = {};
    payload.packetType = PKT_TYPE_FATIGUE_STATUS;
    payload.targetNodeId = FATIGUE_NODE_ID;
    payload.status = (uint8_t)code;

    LoRaPacket packet = LoRaPacketHandler::createFatigueStatusPacket(
        NODE_ID,
        FATIGUE_NODE_ID,
        payload,
        ++fatigueStatusSequence
    );
    packet.header.nextHop = aodv.getNextHop(FATIGUE_NODE_ID);
    packet.header.checksum = LoRaPacketHandler::calculateChecksum(packet);

    sendPacketCallback(packet);
    fatigueStatusPending = false;
    fatigueStatusReadyToSend = false;
    hasSentFatigueStatus = true;
    lastSentFatigueStatusCode = (uint8_t)code;

    Serial.printf("[gateway TX FATIGUE STATUS] target=%s status=%s(%u)\n",
                  getNodeName(FATIGUE_NODE_ID),
                  fatigueAlarmText(code),
                  (unsigned)code);
}

void updateNodeStats(uint8_t nodeId, const char* name, uint32_t latency, bool latencyValid) {
    if (nodeId >= MAX_TRACKED_NODES) {
        return;
    }

    nodeStats[nodeId].packetsReceived++;
    strncpy(nodeStats[nodeId].nodeName, name, sizeof(nodeStats[nodeId].nodeName) - 1);
    nodeStats[nodeId].nodeName[sizeof(nodeStats[nodeId].nodeName) - 1] = '\0';

    if (latencyValid) {
        nodeStats[nodeId].latencySum += latency;
        nodeStats[nodeId].latencyCount++;
        if (latency < nodeStats[nodeId].latencyMin) {
            nodeStats[nodeId].latencyMin = latency;
        }
        if (latency > nodeStats[nodeId].latencyMax) {
            nodeStats[nodeId].latencyMax = latency;
        }
    }
}

uint32_t getExpectedPacketsForNode(uint8_t nodeId, unsigned long durationMs) {
    uint32_t interval = (nodeId == FATIGUE_NODE_ID)
        ? FATIGUE_NODE_SEND_INTERVAL_MS
        : DEFAULT_NODE_SEND_INTERVAL_MS;

    if (nodeId == VEHICLE_NODE_ID) {
        interval = VEHICLE_NODE_SEND_INTERVAL_MS;
    }

    return max(1UL, durationMs / interval);
}

void printQoSStats() {
    unsigned long durationMs = millis() - observationStartTime;

    Serial.println();
    Serial.println("========================================");
    Serial.printf("QoS Stats Round %u (duration=%lus)\n",
                  observationRound,
                  durationMs / 1000);
    Serial.println("Node         Rx   Exp   PDR%   PLR%   Lat(ms)");

    uint32_t totalRx = 0;
    uint32_t totalExp = 0;

    for (uint8_t i = 1; i < MAX_TRACKED_NODES; ++i) {
        uint32_t exp = getExpectedPacketsForNode(i, durationMs);
        uint32_t rx = nodeStats[i].packetsReceived;
        float pdr = min(100.0f, (exp > 0) ? (rx * 100.0f / exp) : 0.0f);
        float plr = 100.0f - pdr;
        uint32_t latAvg = nodeStats[i].latencyCount
            ? nodeStats[i].latencySum / nodeStats[i].latencyCount
            : 0;
        const char* name = nodeStats[i].nodeName[0] ? nodeStats[i].nodeName : getNodeName(i);

        Serial.printf("%-12s %4lu %5lu %6.1f %6.1f %8lu\n",
                      name,
                      (unsigned long)rx,
                      (unsigned long)exp,
                      pdr,
                      plr,
                      (unsigned long)latAvg);

        totalRx += rx;
        totalExp += exp;
    }

    float totalPdr = totalExp ? min(100.0f, totalRx * 100.0f / totalExp) : 0.0f;
    Serial.printf("TOTAL        %4lu %5lu %6.1f %6.1f\n",
                  (unsigned long)totalRx,
                  (unsigned long)totalExp,
                  totalPdr,
                  100.0f - totalPdr);
    Serial.printf("Route Discovery: OK=%u FAIL=%u\n",
                  aodv.routeDiscoverySuccess,
                  aodv.routeDiscoveryFail);
    Serial.println("========================================");

    if (mqttConnected && mqtt_client) {
        StaticJsonDocument<256> summary;
        summary["round"] = observationRound;
        summary["duration_s"] = durationMs / 1000;
        summary["PDR"] = totalPdr;
        summary["PLR"] = 100.0f - totalPdr;
        summary["routeOK"] = aodv.routeDiscoverySuccess;
        summary["routeFail"] = aodv.routeDiscoveryFail;

        String json;
        serializeJson(summary, json);
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_STATUS, json.c_str(), 0, 1, 0);
    }
}

void printCSVStats() {
    unsigned long durationMs = millis() - observationStartTime;

    Serial.println();
    Serial.println("--- CSV OUTPUT ---");
    Serial.printf("SF=%d,BW=%lu,CR=4/%d,ROUND=%u,DUR_S=%lu\n",
                  gwRuntimeCfg.sf,
                  (unsigned long)gwRuntimeCfg.bwKHz,
                  gwRuntimeCfg.cr,
                  observationRound,
                  (unsigned long)(durationMs / 1000));
    Serial.println("NODE,RX,EXP,PDR,PLR,LAT_AVG_MS,LAT_MIN_MS,LAT_MAX_MS,NTP_SYNCED");

    for (uint8_t i = 1; i < MAX_TRACKED_NODES; ++i) {
        uint32_t exp = getExpectedPacketsForNode(i, durationMs);
        uint32_t rx = nodeStats[i].packetsReceived;
        float pdr = min(100.0f, (exp > 0) ? (rx * 100.0f / exp) : 0.0f);
        uint32_t latAvg = nodeStats[i].latencyCount
            ? nodeStats[i].latencySum / nodeStats[i].latencyCount
            : 0;
        uint32_t latMin = nodeStats[i].latencyMin == UINT32_MAX
            ? 0
            : nodeStats[i].latencyMin;
        const char* name = nodeStats[i].nodeName[0] ? nodeStats[i].nodeName : getNodeName(i);

        Serial.printf("%s,%lu,%lu,%.1f,%.1f,%lu,%lu,%lu,%s\n",
                      name,
                      (unsigned long)rx,
                      (unsigned long)exp,
                      pdr,
                      100.0f - pdr,
                      (unsigned long)latAvg,
                      (unsigned long)latMin,
                      (unsigned long)nodeStats[i].latencyMax,
                      ntpSynced ? "YES" : "NO");
    }

    Serial.println("--- END CSV ---");
}

void resetQoSStats() {
    for (int i = 0; i < MAX_TRACKED_NODES; ++i) {
        nodeStats[i] = NodeStats();
    }
    observationStartTime = millis();
    Serial.println("[QoS] Counters reset");
}

void handleSerialCLI() {
    if (!Serial.available()) {
        return;
    }

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "STATS") {
        printQoSStats();
    } else if (cmd == "CSV") {
        printCSVStats();
    } else if (cmd == "RESET" || cmd == "START") {
        observationRound++;
        resetQoSStats();
        Serial.println("[CLI] New observation period started");
    } else if (cmd == "STATUS") {
        printStatus();
    } else if (cmd == "SYNC") {
        timeSyncRequested = true;
        Serial.println("[CLI] Time sync requested");
    } else if (cmd == "ROUTE") {
        Serial.printf("[ROUTE] OK=%u FAIL=%u\n",
                      aodv.routeDiscoverySuccess,
                      aodv.routeDiscoveryFail);
    } else if (cmd == "LELAH") {
        requestFatigueStatus(FATIGUE_STATUS_LELAH);
    } else if (cmd == "TERTIDUR") {
        requestFatigueStatus(FATIGUE_STATUS_TERTIDUR);
    } else if (cmd == "NORMAL") {
        requestFatigueStatus(FATIGUE_STATUS_NORMAL);
    } else if (cmd == "HELP") {
        Serial.println("STATS      - Print QoS table");
        Serial.println("CSV        - Print CSV stats");
        Serial.println("RESET      - Reset QoS counters");
        Serial.println("STATUS     - Print gateway status");
        Serial.println("SYNC       - Send time sync");
        Serial.println("ROUTE      - Print route discovery stats");
        Serial.println("LELAH      - Send LELAH status to lora_saenab");
        Serial.println("TERTIDUR   - Send TERTIDUR status to lora_saenab");
        Serial.println("NORMAL     - Send NORMAL status to lora_saenab");
    } else if (cmd.length() > 0) {
        Serial.printf("[CLI] Unknown command: %s\n", cmd.c_str());
    }
}

const char* getNodeName(uint8_t id) {
    switch (id) {
        case GATEWAY_ID:
            return "GATEWAY";
        case 1:
            return "TRK-001";
        case 2:
            return "TRK-002";
        case 3:
            return "TRK-003";
        case 4:
            return "lora_saenab";
        case 5:
            return "lora_nailah";
        case BROADCAST_ADDR:
            return "BROADCAST";
        default:
            return "UNKNOWN";
    }
}

void printStatus() {
    Serial.println();
    Serial.println("=== Gateway Status ===");
    Serial.printf("WiFi: %s | MQTT: %s | NTP: %s\n",
                  wifiConnected ? "OK" : "FAIL",
                  mqttConnected ? "OK" : "FAIL",
                  ntpSynced ? "SYNCED" : "NO");
    Serial.printf("WiFi Target: %s\n", GWWebConfig::getWiFiSSID(gwRuntimeCfg));
    Serial.printf("Queue: %u | RxTotal: %lu | Published: %lu\n",
                  (unsigned)uxQueueMessagesWaiting(mqttQueue),
                  packetsReceivedTotal,
                  packetsSentToServer);
    Serial.printf("QoS Round: %u | Obs Elapsed: %lus\n",
                  observationRound,
                  (unsigned long)((millis() - observationStartTime) / 1000));
    Serial.printf("Pending Fatigue Status: %s\n",
                  fatigueStatusPending ? fatigueAlarmText((FatigueAlarmCode)pendingFatigueStatusCode) : "NONE");
    Serial.printf("Route: OK=%u FAIL=%u\n",
                  aodv.routeDiscoverySuccess,
                  aodv.routeDiscoveryFail);
    Serial.println("======================");
}
