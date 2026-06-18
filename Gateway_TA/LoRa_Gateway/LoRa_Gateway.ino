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
    uint32_t sequenceNum;
    int8_t rssi;
    int8_t snr;
    uint8_t hopCount;
    uint8_t routePathLen;
    uint8_t routePath[MAX_ROUTE_PATH];
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
uint32_t gatewayAckSequence = 0;

NodeStats nodeStats[MAX_TRACKED_NODES];
unsigned long observationStartTime = 0;
unsigned long lastQoSPrint = 0;
uint8_t observationRound = 0;

LoRaRuntimeCfg gwRuntimeCfg;

// Route path sementara dari paket terakhir yang diterima (diisi di handleReceivedPacket)
static uint8_t  lastRxRoutePathLen = 0;
static uint8_t  lastRxRoutePath[MAX_ROUTE_PATH] = {};

void initLoRa();
void initWiFi();
void initMQTT();
void initNTP();
void checkWiFi();
bool connectToWiFiProfile(uint8_t profileIdx, unsigned long timeoutMs);
int8_t findBestScannedWiFiProfile();
void receivePackets();
void handleReceivedPacket(const LoRaPacket& packet);
void sendPacketCallback(const LoRaPacket& packet);
void enqueueSensorData(const SensorDataPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops);
void enqueueFatigueImuData(const ImuFatiguePayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops);
void enqueueSafetyConditionData(const SafetyConditionPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops);
void enqueueVehicleTelemetryData(const VehicleTelemetryPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops);
void publishQueueItem(const MQTTQueueItem& item);
void publishSensorQueueItem(const MQTTQueueItem& item, const char* name, uint32_t latency, bool latencyValid);
void publishFatigueQueueItem(const MQTTQueueItem& item, uint32_t latency, bool latencyValid);
void publishSafetyConditionQueueItem(const MQTTQueueItem& item, uint32_t latency, bool latencyValid);
void publishVehicleTelemetryQueueItem(const MQTTQueueItem& item, uint32_t latency, bool latencyValid);
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
bool computeLatencyFromTx(uint32_t txTimestamp, uint32_t& latencyOut);
bool shouldAckDataPacketType(uint8_t packetType);
void sendDataAckForPacket(const LoRaPacket& packet);
static void mqtt_event_handler(void*, esp_event_base_t, int32_t, void*);
void TaskLoRa(void*);

// Per-packet CSV logging: satu baris per paket yang diterima Gateway
// Format: EPOCH_MS,NODE_ID,NODE_NAME,TYPE,SEQ,RSSI,SNR,HOPS,LATENCY_MS,SF,BW_KHZ
void logPacketCSV(uint8_t srcId, const char* typeName, uint32_t seq,
                  int8_t rssi, int8_t snr, uint8_t hops,
                  uint32_t latencyMs, bool latencyValid) {
    uint64_t epoch = getEpochMs64();
    Serial.printf("[CSV] %lu,%u,%s,%s,%lu,%d,%d,%u,%s%lu,%d,%lu\n",
                  (unsigned long)(epoch & 0xFFFFFFFF),
                  srcId, getNodeName(srcId), typeName,
                  (unsigned long)seq,
                  rssi, snr, hops,
                  latencyValid ? "" : "~",
                  (unsigned long)latencyMs,
                  gwRuntimeCfg.sf,
                  (unsigned long)gwRuntimeCfg.bwKHz);
}

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

    // Register callback untuk START_TEST button di WebConfig UI
    GWWebConfig::setStartTestCallback([](uint8_t sf, uint32_t bwKHz) {
        StartTestPayload st;
        st.sf = sf;
        st.bwKHz = bwKHz;
        LoRaPacket pkt = LoRaPacketHandler::createStartTestPacket(GATEWAY_ID, st);
        sendPacketCallback(pkt);
        delay(500);
        sendPacketCallback(pkt);  // 2x untuk reliability
        Serial.printf("[WebUI] START_TEST broadcast: SF=%d BW=%lukHz\n", sf, (unsigned long)bwKHz);
    });

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
        while (xQueueReceive(mqttQueue, &item, 0) == pdTRUE && count++ < 15) {
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
    rf95.setCADTimeout(LORA_CAD_TIMEOUT_MS);

    Serial.printf("[LoRa] OK SF=%d BW=%.0fkHz CR=4/%d Pwr=%ddBm (%s)\n",
                  gwRuntimeCfg.sf,
                  bwHz / 1000.0f,
                  gwRuntimeCfg.cr,
                  pwr,
                  gwRuntimeCfg.useRFO ? "RFO" : "PA_BOOST");
}

bool connectToWiFiProfile(uint8_t profileIdx, unsigned long timeoutMs) {
    if (profileIdx >= WIFI_PROFILE_COUNT) {
        return false;
    }
    const WiFiProfile& profile = WIFI_PROFILES[profileIdx];
    if (profile.ssid == nullptr || profile.ssid[0] == '\0') {
        return false;
    }

    Serial.printf("[WiFi] Try profile #%u: %s\n", (unsigned)(profileIdx + 1), profile.ssid);
    WiFi.begin(profile.ssid, profile.password ? profile.password : "");
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        delay(300);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        gwRuntimeCfg.wifiProfileIdx = profileIdx;
        Serial.printf("[WiFi] Connected via profile #%u (%s)\n",
                      (unsigned)(profileIdx + 1),
                      profile.ssid);
        return true;
    }

    Serial.printf("[WiFi] Failed profile #%u (%s)\n",
                  (unsigned)(profileIdx + 1),
                  profile.ssid);
    return false;
}

int8_t findBestScannedWiFiProfile() {
    Serial.println("[WiFi] Scanning SSID sekitar...");
    const int found = WiFi.scanNetworks(false, true);
    if (found <= 0) {
        Serial.println("[WiFi] Tidak ada SSID terdeteksi.");
        WiFi.scanDelete();
        return -1;
    }

    int8_t bestProfile = -1;
    int32_t bestRssi = -127;

    for (int i = 0; i < found; ++i) {
        const String scannedSsid = WiFi.SSID(i);
        const int32_t scannedRssi = WiFi.RSSI(i);
        for (uint8_t p = 0; p < WIFI_PROFILE_COUNT; ++p) {
            if (scannedSsid.equals(WIFI_PROFILES[p].ssid) && scannedRssi > bestRssi) {
                bestRssi = scannedRssi;
                bestProfile = (int8_t)p;
            }
        }
    }
    WiFi.scanDelete();

    if (bestProfile >= 0) {
        Serial.printf("[WiFi] Best SSID terdeteksi: %s (RSSI=%ld)\n",
                      WIFI_PROFILES[(uint8_t)bestProfile].ssid,
                      (long)bestRssi);
    } else {
        Serial.println("[WiFi] Tidak ada SSID yang cocok dengan WIFI_PROFILES.");
    }
    return bestProfile;
}

void initWiFi() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.softAP(GATEWAY_CONFIG_SSID);
    delay(200);

    Serial.printf("[WiFi] Config SSID: %s\n", GATEWAY_CONFIG_SSID);
    Serial.printf("[WiFi] Config URL : http://%s/\n", WiFi.softAPIP().toString().c_str());
    
    const uint8_t configuredProfile = GWWebConfig::getWiFiProfileIndex(gwRuntimeCfg);
    bool connected = false;

    const int8_t bestProfile = findBestScannedWiFiProfile();
    if (bestProfile >= 0) {
        connected = connectToWiFiProfile((uint8_t)bestProfile, WIFI_TIMEOUT);
    }

    if (!connected) {
        connected = connectToWiFiProfile(configuredProfile, WIFI_TIMEOUT);
    }

    if (!connected) {
        for (uint8_t p = 0; p < WIFI_PROFILE_COUNT; ++p) {
            if (p == configuredProfile || (bestProfile >= 0 && p == (uint8_t)bestProfile)) {
                continue;
            }
            if (connectToWiFiProfile(p, WIFI_TIMEOUT)) {
                connected = true;
                break;
            }
        }
    }

    wifiConnected = connected;
    if (connected) {
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
    configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // Set ke UTC+8

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
        bool reconnected = false;
        const int8_t bestProfile = findBestScannedWiFiProfile();
        if (bestProfile >= 0) {
            reconnected = connectToWiFiProfile((uint8_t)bestProfile, WIFI_TIMEOUT);
        }
        if (!reconnected) {
            WiFi.reconnect();
        }
        wifiConnected = (WiFi.status() == WL_CONNECTED);
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

bool computeLatencyFromTx(uint32_t txTimestamp, uint32_t& latencyOut) {
    latencyOut = 0;
    if (!ntpSynced || txTimestamp == 0) {
        return false;
    }

    uint32_t diff = getEpochMsLow32() - txTimestamp;
    if (diff >= 60000UL) {
        return false;
    }

    latencyOut = diff;
    return true;
}

bool shouldAckDataPacketType(uint8_t packetType) {
    return packetType == PKT_TYPE_DATA ||
           packetType == PKT_TYPE_FATIGUE_IMU ||
           packetType == PKT_TYPE_SAFETY_CONDITION ||
           packetType == PKT_TYPE_VEHICLE_TELEMETRY;
}

void sendDataAckForPacket(const LoRaPacket& packet) {
    if (!DATA_ACK_ENABLE || !shouldAckDataPacketType(packet.header.packetType)) {
        return;
    }
    if (packet.header.destinationID != GATEWAY_ID || packet.header.nextHop != NODE_ID) {
        return;
    }
    if (!aodv.hasRouteTo(packet.header.sourceID)) {
        aodv.initiateRouteDiscovery(packet.header.sourceID);
        return;
    }

    AckPayload ack = {};
    ack.ackedPacketType = packet.header.packetType;
    ack.ackedSequence = packet.header.sequenceNum;

    LoRaPacket ackPacket = LoRaPacketHandler::createAckPacket(
        NODE_ID,
        packet.header.sourceID,
        ack,
        ++gatewayAckSequence
    );
    ackPacket.header.nextHop = aodv.getNextHop(packet.header.sourceID);
    ackPacket.header.checksum = LoRaPacketHandler::calculateChecksum(ackPacket);
    sendPacketCallback(ackPacket);
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

struct UplinkDedupEntry {
    uint8_t src;
    uint8_t type;
    uint32_t seq;
    uint32_t payloadHash;
    unsigned long ts;
    bool valid;
};

static UplinkDedupEntry uplinkDedupCache[96];
static uint8_t uplinkDedupIndex = 0;
static const unsigned long UPLINK_DEDUP_WINDOW_MS = 60000;

uint32_t hashPacketPayload(const LoRaPacket& packet) {
    uint32_t hash = 2166136261UL; // FNV-1a 32-bit
    for (uint16_t i = 0; i < packet.header.payloadLength && i < MAX_PAYLOAD_SIZE; i++) {
        hash ^= packet.payload[i];
        hash *= 16777619UL;
    }
    return hash;
}

bool isDuplicateUplinkPayload(const LoRaPacket& packet) {
    if (!shouldAckDataPacketType(packet.header.packetType) ||
        packet.header.destinationID != GATEWAY_ID ||
        packet.header.nextHop != NODE_ID ||
        packet.header.sequenceNum == 0) {
        return false;
    }

    unsigned long now = millis();
    uint32_t payloadHash = hashPacketPayload(packet);
    for (uint8_t i = 0; i < 96; i++) {
        UplinkDedupEntry& entry = uplinkDedupCache[i];
        if (!entry.valid) {
            continue;
        }
        if ((now - entry.ts) > UPLINK_DEDUP_WINDOW_MS) {
            entry.valid = false;
            continue;
        }
        if (entry.src == packet.header.sourceID &&
            entry.type == packet.header.packetType &&
            entry.seq == packet.header.sequenceNum &&
            entry.payloadHash == payloadHash) {
            Serial.printf("[DUP] Dropped payload src=%u seq=%lu type=0x%02X (ACK ulang)\n",
                          packet.header.sourceID,
                          (unsigned long)packet.header.sequenceNum,
                          packet.header.packetType);
            return true;
        }
    }

    uplinkDedupCache[uplinkDedupIndex] = {
        packet.header.sourceID,
        packet.header.packetType,
        packet.header.sequenceNum,
        payloadHash,
        now,
        true
    };
    uplinkDedupIndex = (uplinkDedupIndex + 1) % 96;
    return false;
}

void handleReceivedPacket(const LoRaPacket& packet) {
    // --- ARTIFICIAL TOPOLOGY FILTER ---
    // Mensimulasikan halangan fisik (Lantai 3 vs Lantai 1)
    // Jika paket datang secara langsung (hopCount == 0) dari TRK-003 (ID 3), kita buang!
    if (packet.header.sourceID == 3 && packet.header.hopCount == 0) {
        // Uncomment baris di bawah jika ingin melihat log saat paket diblokir
        // Serial.println("[FILTER] Dropping direct packet from TRK-003 to enforce multihop");
        return;
    }
    // ----------------------------------

    int8_t rssi = rf95.lastRssi();
    int8_t snr = rf95.lastSNR();
    Serial.printf("[RX] Type=%s Src=%s Dst=%s NextHop=%u RSSI=%d\n",
                  packetHandler.getPacketTypeName(packet.header.packetType),
                  getNodeName(packet.header.sourceID),
                  getNodeName(packet.header.destinationID),
                  packet.header.nextHop,
                  rssi);

    uint8_t ptype = packet.header.packetType;
    if (isDuplicateUplinkPayload(packet)) {
        sendDataAckForPacket(packet);
        return;
    }

    switch (ptype) {
        case PKT_TYPE_DATA:
            if (packet.header.destinationID == GATEWAY_ID &&
                packet.header.nextHop == NODE_ID) {
                if (packet.header.payloadLength != sizeof(SensorDataPayload)) {
                    Serial.printf("[RX] Invalid sensor payload size: got=%u expected=%u\n",
                                  packet.header.payloadLength, (unsigned)sizeof(SensorDataPayload));
                    break;
                }

                SensorDataPayload data;
                memcpy(&data, packet.payload, sizeof(data));
                lastRxRoutePathLen = packet.header.routePathLen;
                memcpy(lastRxRoutePath, packet.header.routePath, MAX_ROUTE_PATH);
                enqueueSensorData(data, packet.header.sourceID, packet.header.sequenceNum, rssi, snr, packet.header.hopCount);
                {
                  uint32_t lat = 0;
                  bool latValid = computeLatencyFromTx(data.txTimestamp, lat);
                  logPacketCSV(packet.header.sourceID, "DATA", packet.header.sequenceNum,
                              rssi, snr, packet.header.hopCount, lat, latValid);
                  updateNodeStats(packet.header.sourceID, getNodeName(packet.header.sourceID), lat, latValid);
                }
                sendDataAckForPacket(packet);
            }
            break;

        case PKT_TYPE_FATIGUE_IMU:
            if (packet.header.destinationID == GATEWAY_ID &&
                packet.header.nextHop == NODE_ID) {
                // Toleransi ukuran lama (tanpa buzzerActive=21) dan baru (dengan buzzerActive=22)
                const uint16_t oldSize = sizeof(ImuFatiguePayload) - 1; // tanpa buzzerActive
                const uint16_t newSize = sizeof(ImuFatiguePayload);     // dengan buzzerActive
                if (packet.header.payloadLength != oldSize && packet.header.payloadLength != newSize) {
                    Serial.printf("[RX] Invalid fatigue IMU payload size: got=%u expected=%u or %u\n",
                                  packet.header.payloadLength, oldSize, newSize);
                    break;
                }

                ImuFatiguePayload data;
                memset(&data, 0, sizeof(data)); // zero-init agar buzzerActive = 0 jika tidak ada
                memcpy(&data, packet.payload, packet.header.payloadLength);
                if (data.packetType != PKT_TYPE_FATIGUE_IMU) {
                    Serial.println("[RX] Invalid fatigue IMU packetType in payload");
                    break;
                }
                Serial.printf("[gateway RX IMU] nodeId=%u ts=%lu pitch=%.2f roll=%.2f buzzer=%s\n",
                              data.nodeId,
                              (unsigned long)data.ts,
                              data.pitch100 / 100.0f,
                              data.roll100 / 100.0f,
                              data.buzzerActive ? "ON" : "OFF");
                lastRxRoutePathLen = packet.header.routePathLen;
                memcpy(lastRxRoutePath, packet.header.routePath, MAX_ROUTE_PATH);
                enqueueFatigueImuData(data, packet.header.sourceID, packet.header.sequenceNum, rssi, snr, packet.header.hopCount);
                {
                  uint32_t lat = 0;
                  bool latValid = computeLatencyFromTx(data.ts, lat);
                  logPacketCSV(packet.header.sourceID, "FATIGUE_IMU", packet.header.sequenceNum,
                              rssi, snr, packet.header.hopCount, lat, latValid);
                  updateNodeStats(packet.header.sourceID, getNodeName(packet.header.sourceID), lat, latValid);
                }
                if (packet.header.sourceID == FATIGUE_NODE_ID && fatigueStatusPending) {
                    fatigueStatusReadyToSend = true;
                }
                sendDataAckForPacket(packet);
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
                lastRxRoutePathLen = packet.header.routePathLen;
                memcpy(lastRxRoutePath, packet.header.routePath, MAX_ROUTE_PATH);
                enqueueSafetyConditionData(data, packet.header.sourceID, packet.header.sequenceNum, rssi, snr, packet.header.hopCount);
                {
                  uint32_t lat = 0;
                  bool latValid = computeLatencyFromTx(data.ts, lat);
                  logPacketCSV(packet.header.sourceID, "SAFETY", packet.header.sequenceNum,
                              rssi, snr, packet.header.hopCount, lat, latValid);
                  updateNodeStats(packet.header.sourceID, getNodeName(packet.header.sourceID), lat, latValid);
                }
                sendDataAckForPacket(packet);
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
                lastRxRoutePathLen = packet.header.routePathLen;
                memcpy(lastRxRoutePath, packet.header.routePath, MAX_ROUTE_PATH);
                enqueueVehicleTelemetryData(data, packet.header.sourceID, packet.header.sequenceNum, rssi, snr, packet.header.hopCount);
                {
                  uint32_t lat = 0;
                  bool latValid = computeLatencyFromTx(data.txTimestamp, lat);
                  logPacketCSV(packet.header.sourceID, "VEHICLE", packet.header.sequenceNum,
                              rssi, snr, packet.header.hopCount, lat, latValid);
                  updateNodeStats(packet.header.sourceID, getNodeName(packet.header.sourceID), lat, latValid);
                }
                sendDataAckForPacket(packet);
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

        case PKT_TYPE_DIAGNOSTIC:
            if (packet.header.destinationID == GATEWAY_ID) {
                if (packet.header.payloadLength == sizeof(DiscoveryDiagPayload)) {
                    DiscoveryDiagPayload diag;
                    memcpy(&diag, packet.payload, sizeof(diag));

                    Serial.println("========================================");
                    Serial.printf("[DIAG RX] Node %s -> target=%u\n",
                                  getNodeName(diag.originNodeId),
                                  diag.targetNodeId);
                    Serial.printf("  RREQ at : %lu\n", (unsigned long)diag.rreqTimestamp);
                    Serial.printf("  RREP at : %lu\n", (unsigned long)diag.rrepTimestamp);
                    Serial.printf("  Duration: %lu ms\n", (unsigned long)diag.discoveryMs);
                    Serial.printf("  Hops    : %u\n", diag.hopCount);
                    Serial.printf("  Retries : %u\n", diag.retryCount);
                    Serial.printf("  Result  : %s\n", diag.success ? "SUCCESS" : "FAILED");
                    Serial.println("========================================");

                    // Publish to MQTT
                    if (mqttConnected && mqtt_client) {
                        StaticJsonDocument<384> doc;
                        doc["node_asal"] = diag.originNodeId;
                        doc["node_name"] = getNodeName(diag.originNodeId);
                        doc["target_rute"] = diag.targetNodeId;
                        doc["rreq_at"] = diag.rreqTimestamp;
                        doc["rrep_at"] = diag.rrepTimestamp;
                        doc["discovery_ms"] = diag.discoveryMs;
                        doc["hops"] = diag.hopCount;
                        doc["retries"] = diag.retryCount;
                        doc["success"] = diag.success;
                        doc["rssi"] = rssi;

                        char buf[384];
                        serializeJson(doc, buf, sizeof(buf));
                        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DIAGNOSTIC, buf, 0, 1, 0);
                        Serial.printf("[MQTT] Published diagnostic -> %s\n", MQTT_TOPIC_DIAGNOSTIC);
                    }
                }
            }
            break;

        default:
            break;
    }
}

void sendPacketCallback(const LoRaPacket& packet) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    int len = packetHandler.serializePacket(packet, buf, sizeof(buf));

    if (packet.header.packetType == PKT_TYPE_ACK) {
        delay(random(5, 20));
    } else {
        delay(random(30, 150));
    }
    if (len > 0) {
        rf95.send(buf, len);
        rf95.waitPacketSent();
        delay(10);
        rf95.setModeRx();
    }
}

void enqueueSensorData(const SensorDataPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops) {
    if (!mqttQueue) {
        return;
    }

    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_SENSOR;
    item.sourceNodeID = src;
    item.sequenceNum = seqNum;
    item.rssi = rssi;
    item.snr = snr;
    item.hopCount = hops;
    item.routePathLen = lastRxRoutePathLen;
    memcpy(item.routePath, lastRxRoutePath, MAX_ROUTE_PATH);
    item.sensorData = data;

    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("[MQTT] Queue full, sensor packet dropped");
    }
}

void enqueueFatigueImuData(const ImuFatiguePayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops) {
    if (!mqttQueue) {
        return;
    }

    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_FATIGUE_IMU;
    item.sourceNodeID = src;
    item.sequenceNum = seqNum;
    item.rssi = rssi;
    item.snr = snr;
    item.hopCount = hops;
    item.routePathLen = lastRxRoutePathLen;
    memcpy(item.routePath, lastRxRoutePath, MAX_ROUTE_PATH);
    item.fatigueImu = data;

    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("[MQTT] Queue full, fatigue packet dropped");
    }
}

void enqueueSafetyConditionData(const SafetyConditionPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops) {
    if (!mqttQueue) {
        return;
    }

    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_SAFETY_CONDITION;
    item.sourceNodeID = src;
    item.sequenceNum = seqNum;
    item.rssi = rssi;
    item.snr = snr;
    item.hopCount = hops;
    item.routePathLen = lastRxRoutePathLen;
    memcpy(item.routePath, lastRxRoutePath, MAX_ROUTE_PATH);
    item.safetyCondition = data;

    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("[MQTT] Queue full, safety packet dropped");
    }
}

void enqueueVehicleTelemetryData(const VehicleTelemetryPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops) {
    if (!mqttQueue) {
        return;
    }

    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_VEHICLE_TELEMETRY;
    item.sourceNodeID = src;
    item.sequenceNum = seqNum;
    item.rssi = rssi;
    item.snr = snr;
    item.hopCount = hops;
    item.routePathLen = lastRxRoutePathLen;
    memcpy(item.routePath, lastRxRoutePath, MAX_ROUTE_PATH);
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
    bool latencyValid = computeLatencyFromTx(txTimestamp, latency);

    if (latencyValid) {
        Serial.printf("[LATENCY] %s -> %lu ms (hops=%u)\n",
                      name,
                      (unsigned long)latency,
                      item.hopCount);
    }

    if (item.kind == QUEUE_KIND_SENSOR) {
        publishSensorQueueItem(item, name, latency, latencyValid);
    } else if (item.kind == QUEUE_KIND_FATIGUE_IMU) {
        publishFatigueQueueItem(item, latency, latencyValid);
    } else if (item.kind == QUEUE_KIND_SAFETY_CONDITION) {
        publishSafetyConditionQueueItem(item, latency, latencyValid);
    } else if (item.kind == QUEUE_KIND_VEHICLE_TELEMETRY) {
        publishVehicleTelemetryQueueItem(item, latency, latencyValid);
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

    struct tm timeinfo;
    time_t txSec = eventTimestamp;
    localtime_r(&txSec, &timeinfo);
    char timeStr[32];
    uint32_t txMsec = txEpochMs64 > 0 ? (txEpochMs64 % 1000) : 0;
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d:%03d",
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, txMsec);

    StaticJsonDocument<512> doc;
    doc["nodeId"] = name;
    doc["ts"] = timeStr;
    doc["epoch"] = eventTimestamp;
    doc["seq"] = item.sequenceNum;
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
    doc["rssi"] = item.rssi;
    doc["snr"] = item.snr;
    doc["route_disc_ms"] = item.sensorData.routeDiscMs;
    doc["route_hops"] = item.sensorData.routeHops;

    // Route path: array nama node yang dilalui paket
    JsonArray rp = doc.createNestedArray("route_path");
    uint8_t rpLen = item.routePathLen;
    if (rpLen > MAX_ROUTE_PATH) rpLen = MAX_ROUTE_PATH;
    for (uint8_t i = 0; i < rpLen; i++) {
        rp.add(getNodeName(item.routePath[i]));
    }
    rp.add("GATEWAY");  // Gateway selalu jadi tujuan akhir

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

void publishFatigueQueueItem(const MQTTQueueItem& item, uint32_t latency, bool latencyValid) {
    StaticJsonDocument<448> doc;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t currentMs = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000);
    uint64_t txMs = currentMs;
    if (latencyValid) {
        txMs = currentMs - latency;
    }
    time_t txSec = txMs / 1000;
    uint32_t txMsec = txMs % 1000;
    struct tm timeinfo;
    localtime_r(&txSec, &timeinfo);
    char timeStr[32];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d:%03d",
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, txMsec);

    doc["nodeId"] = item.fatigueImu.nodeId;
    doc["ts"] = timeStr;
    doc["epoch"] = item.fatigueImu.ts; // untuk kalkulasi internal
    doc["seq"] = item.sequenceNum;
    doc["pitch"] = item.fatigueImu.pitch100 / 100.0f;
    doc["roll"] = item.fatigueImu.roll100 / 100.0f;
    doc["f_gx"] = item.fatigueImu.gx1000 / 1000.0f;
    doc["f_gy"] = item.fatigueImu.gy1000 / 1000.0f;
    doc["f_gz"] = item.fatigueImu.gz1000 / 1000.0f;
    doc["hopCount"] = item.hopCount;
    doc["rssi"] = item.rssi;
    doc["snr"] = item.snr;
    doc["route_disc_ms"] = item.fatigueImu.routeDiscMs;
    doc["route_hops"] = item.fatigueImu.routeHops;
    doc["status_buzzer"] = item.fatigueImu.buzzerActive ? "ON" : "OFF";

    // Route path: array nama node yang dilalui paket
    JsonArray rp = doc.createNestedArray("route_path");
    uint8_t rpLen = item.routePathLen;
    if (rpLen > MAX_ROUTE_PATH) rpLen = MAX_ROUTE_PATH;
    for (uint8_t i = 0; i < rpLen; i++) {
        rp.add(getNodeName(item.routePath[i]));
    }
    rp.add("GATEWAY");  // Gateway selalu jadi tujuan akhir
    if (latencyValid) {
        doc["latency_ms"] = latency;
    }

    char json[448];
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

void publishSafetyConditionQueueItem(const MQTTQueueItem& item, uint32_t latency, bool latencyValid) {
    StaticJsonDocument<448> doc;
    const uint8_t flags = item.safetyCondition.flags;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t currentMs = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000);
    uint64_t txMs = currentMs;
    if (latencyValid) {
        txMs = currentMs - latency;
    }
    time_t txSec = txMs / 1000;
    uint32_t txMsec = txMs % 1000;
    struct tm timeinfo;
    localtime_r(&txSec, &timeinfo);
    char timeStr[32];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d:%03d",
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, txMsec);

    doc["nodeId"] = item.safetyCondition.nodeId;
    doc["ts"] = timeStr;
    doc["epoch"] = item.safetyCondition.ts;
    doc["seq"] = item.sequenceNum;
    doc["gpsValid"] = (flags & FLAG_GPS_VALID) != 0;
    doc["drActive"] = (flags & FLAG_DR_ACTIVE) != 0;
    doc["rolloverRisk"] = (flags & FLAG_ROLLOVER_RISK) != 0;
    doc["rollover"] = (flags & FLAG_ROLLOVER) != 0;
    doc["harshBraking"] = (flags & FLAG_HARSH_BRAKE) != 0;
    doc["overspeed"] = (flags & FLAG_OVERSPEED) != 0;
    doc["flags"] = flags;
    doc["hopCount"] = item.hopCount;
    doc["rssi"] = item.rssi;
    doc["snr"] = item.snr;
    doc["route_disc_ms"] = item.safetyCondition.routeDiscMs;
    doc["route_hops"] = item.safetyCondition.routeHops;

    JsonArray rp = doc.createNestedArray("route_path");
    uint8_t rpLen = item.routePathLen;
    if (rpLen > MAX_ROUTE_PATH) rpLen = MAX_ROUTE_PATH;
    for (uint8_t i = 0; i < rpLen; i++) {
        rp.add(getNodeName(item.routePath[i]));
    }
    rp.add("GATEWAY");

    if (latencyValid) {
        doc["latency_ms"] = latency;
    }

    char json[448];
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

void publishVehicleTelemetryQueueItem(const MQTTQueueItem& item, uint32_t latency, bool latencyValid) {
    StaticJsonDocument<1280> doc;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t currentMs = (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000);
    uint64_t txMs = currentMs;
    if (latencyValid) {
        txMs = currentMs - latency;
    }
    time_t txSec = txMs / 1000;
    uint32_t txMsec = txMs % 1000;
    struct tm timeinfo;
    localtime_r(&txSec, &timeinfo);
    char timeStr[32];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d:%03d",
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, txMsec);

    doc["nodeId"] = item.sourceNodeID;
    doc["ts"] = timeStr;
    doc["epoch"] = item.vehicleTelemetry.txTimestamp
        ? item.vehicleTelemetry.txTimestamp
        : (uint32_t)(currentMs & 0xFFFFFFFF);
    doc["seq"] = item.sequenceNum;

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
    doc["hopCount"] = item.hopCount;
    doc["rssi"] = item.rssi;
    doc["snr"] = item.snr;
    doc["route_disc_ms"] = item.vehicleTelemetry.routeDiscMs;
    doc["route_hops"] = item.vehicleTelemetry.routeHops;
    if (latencyValid) {
        doc["latency_ms"] = latency;
    }

    JsonArray rp = doc.createNestedArray("route_path");
    uint8_t rpLen = item.routePathLen;
    if (rpLen > MAX_ROUTE_PATH) rpLen = MAX_ROUTE_PATH;
    for (uint8_t i = 0; i < rpLen; i++) {
        rp.add(getNodeName(item.routePath[i]));
    }
    rp.add("GATEWAY");

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
    } else if (cmd == "START") {
        observationRound++;
        resetQoSStats();
        Serial.println("[CLI] New observation period started");
    } else if (cmd == "RESET") {
        // Reset NVRAM config ke default config.h
        Preferences prefs;
        prefs.begin("lora-cfg", false);
        prefs.clear();
        prefs.end();
        Serial.println("[CLI] NVRAM config cleared. Rebooting in 2s...");
        delay(2000);
        ESP.restart();
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
        Serial.println("STATS        - Print QoS table");
        Serial.println("CSV          - Print CSV stats");
        Serial.println("START        - Reset QoS counters (new observation)");
        Serial.println("RESET        - Reset NVRAM config & reboot (factory default)");
        Serial.println("STATUS       - Print gateway status");
        Serial.println("SYNC         - Send time sync");
        Serial.println("ROUTE        - Print route discovery stats");
        Serial.println("LELAH        - Send LELAH status to lora_saenab");
        Serial.println("TERTIDUR     - Send TERTIDUR status to lora_saenab");
        Serial.println("NORMAL       - Send NORMAL status to lora_saenab");
        Serial.println("TEST SF7 BW125 - Broadcast START_TEST (semua node reboot SF/BW baru)");
    } else if (cmd.startsWith("TEST ")) {
        // Parse: TEST SF7 BW125
        uint8_t newSF = 7;
        uint32_t newBW = 125;
        int sfIdx = cmd.indexOf("SF");
        int bwIdx = cmd.indexOf("BW");
        if (sfIdx >= 0) newSF = cmd.substring(sfIdx + 2).toInt();
        if (bwIdx >= 0) newBW = cmd.substring(bwIdx + 2).toInt();
        newSF = constrain(newSF, 7, 12);
        newBW = (newBW == 250) ? 250 : 125;

        Serial.printf("[CLI] Broadcasting START_TEST: SF=%d BW=%lukHz\n", newSF, (unsigned long)newBW);

        // 1. Broadcast START_TEST packet ke semua node
        StartTestPayload st;
        st.sf = newSF;
        st.bwKHz = newBW;
        LoRaPacket pkt = LoRaPacketHandler::createStartTestPacket(GATEWAY_ID, st);
        sendPacketCallback(pkt);
        delay(500);
        sendPacketCallback(pkt);  // Kirim 2x untuk reliability

        // 2. Simpan config baru di Gateway juga, lalu reboot
        Serial.println("[CLI] Gateway akan reboot dengan parameter baru dalam 3 detik...");
        GWWebConfig::saveTestConfig(newSF, newBW);
        delay(3000);
        ESP.restart();
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
