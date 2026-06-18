#include <SPI.h>
#include <WiFi.h>
#include <RH_RF95.h>
#include <time.h>
#include <sys/time.h>
#include "config.h"
#include "LoRa_Packet.h"
#include "AODV_Routing.h"
#include "nfc_integration.h"
#include "WebConfig.h"   // Web Config UI port 80
#include <ArduinoJson.h>
#include "wifi_handler.h"
#include "fms_mqtt.h"    

extern "C" {
  #include "esp_event.h"
  #include "mqtt_client.h"
  #include "esp_crt_bundle.h"
}

#define NODE_ID GATEWAY_ID

enum MQTTQueueKind : uint8_t {
    QUEUE_KIND_SENSOR = 0,
    QUEUE_KIND_FATIGUE_IMU,
    QUEUE_KIND_SAFETY_CONDITION,
    QUEUE_KIND_VEHICLE_TELEMETRY
};

struct MQTTQueueItem {
    MQTTQueueKind kind;
    uint8_t sourceNodeID;
    uint32_t sequenceNum;
    int8_t  rssi;
    int8_t  snr;
    uint8_t hopCount;
    uint8_t routePathLen;
    uint8_t routePath[MAX_ROUTE_PATH];
    SensorDataPayload data;
    ImuFatiguePayload fatigueImu;
    SafetyConditionPayload safetyCondition;
    VehicleTelemetryPayload vehicleTelemetry;
};

// ================================================================
// STATISTIK QoS PER-NODE
// ================================================================
#define MAX_TRACKED_NODES 6

struct NodeStats {
    uint32_t packetsReceived;
    uint32_t latencySum;
    uint32_t latencyMin;
    uint32_t latencyMax;
    uint32_t latencyCount;
    char nodeName[16];
    NodeStats() {
        packetsReceived = 0; latencySum = 0;
        latencyMin = UINT32_MAX; latencyMax = 0; latencyCount = 0;
        memset(nodeName, 0, sizeof(nodeName));
    }
};

#define OBSERVATION_PERIOD_MS  300000UL  // 5 menit
#define NODE_SEND_INTERVAL_MS  3000UL    // Harus = Node config DATA_SEND_INTERVAL
#define TIMESYNC_INTERVAL_MS   30000UL   // Broadcast time sync tiap 30 detik
#define GATEWAY_MQTT_START_DELAY_MS 5000UL
#define GATEWAY_MQTT_RETRY_MS       10000UL
#define NTP_RETRY_INTERVAL_MS       10000UL
#define NTP_RESYNC_INTERVAL_MS      3600000UL

// ================================================================
// GLOBAL OBJECTS
// ================================================================
RH_RF95 rf95(LORA_CS_PIN, LORA_DIO0_PIN);
AODVRouting aodv(NODE_ID);
LoRaPacketHandler packetHandler;
WiFiHelper wifiHelper;
// fmsMqtt is extern in fms_mqtt.h
QueueHandle_t mqttQueue;
TaskHandle_t LoRaTaskHandle;

// ================================================================
// STATUS VARIABLES
// ================================================================
bool wifiConnected = false;
volatile bool mqttConnected = false;
unsigned long lastWiFiCheck = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastNtpAttemptMs = 0;
unsigned long lastNtpSyncMs = 0;
volatile unsigned long packetsReceivedTotal = 0;
volatile unsigned long packetsSentToServer  = 0;

// NTP Time Sync (P1)
bool ntpSynced = false;
uint64_t ntpBaseEpochMs  = 0;   // Gateway epoch ms penuh saat NTP sync
unsigned long ntpBaseMillis = 0;
volatile bool timeSyncRequested = false;

// QoS Stats
NodeStats nodeStats[MAX_TRACKED_NODES];
unsigned long observationStartTime = 0;
unsigned long lastQoSPrint = 0;
uint8_t observationRound = 0;

// ================================================================
// FUNCTION PROTOTYPES
// ================================================================
void initLoRa(); void initNTP();
void checkWiFi();
void maintainMqttConnection(unsigned long now);
void receivePackets(); void handleReceivedPacket(const LoRaPacket& packet);
void sendPacketCallback(const LoRaPacket& packet);
void enqueueSensorData(const SensorDataPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops, uint8_t routePathLen, const uint8_t* routePath);
void enqueueFatigueImuData(const ImuFatiguePayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops, uint8_t routePathLen, const uint8_t* routePath);
void enqueueSafetyConditionData(const SafetyConditionPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops, uint8_t routePathLen, const uint8_t* routePath);
void enqueueVehicleTelemetryData(const VehicleTelemetryPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops, uint8_t routePathLen, const uint8_t* routePath);
void publishQueueItem(const MQTTQueueItem& item);
void sendTimeSyncPacket();
void handleSerialCLI();
void handleRemoteCommand(const String &cmd);
void printQoSStats(); void printCSVStats(); void resetQoSStats(); void printStatus();
const char* getNodeName(uint8_t nodeId);
uint64_t getEpochMs64();
uint32_t getEpochMsLow32();
void TaskLoRa(void*);
void initWiFiProfiles();
bool shouldAckDataPacketType(uint8_t packetType);
void sendDataAckForPacket(const LoRaPacket& packet);
bool computeLatencyFromTx(uint32_t txTimestamp, uint32_t& latencyMs);
void maintainNtpSync(unsigned long now);

// ================================================================
// SETUP
// ================================================================
LoRaRuntimeCfg gwRuntimeCfg;
WiFiCredential gatewayWiFiProfiles[WIFI_PROFILE_COUNT];
uint8_t gatewayWiFiProfileCount = 0;

void initWiFiProfiles() {
    gatewayWiFiProfileCount = 0;
    for (uint8_t i = 0; i < WIFI_PROFILE_COUNT; ++i) {
        const char* ssid = WIFI_PROFILES[i].ssid;
        if (ssid == nullptr || ssid[0] == '\0') continue;

        gatewayWiFiProfiles[gatewayWiFiProfileCount].ssid = ssid;
        gatewayWiFiProfiles[gatewayWiFiProfileCount].pass =
            (WIFI_PROFILES[i].password != nullptr) ? WIFI_PROFILES[i].password : "";
        gatewayWiFiProfileCount++;
    }

    if (gatewayWiFiProfileCount == 0) {
        Serial.println("[WiFi] ERROR: WIFI_PROFILES kosong, tidak bisa konek STA.");
        return;
    }

    Serial.printf("[WiFi] Auto-select dari %u profile (WiFiMulti)\n", gatewayWiFiProfileCount);
    wifiHelper.begin(gatewayWiFiProfiles, gatewayWiFiProfileCount);
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);
    Serial.println("\n\n========================================");
    Serial.println("FIRMWARE VERSION: 2.3 (WebConfig + QoS + NFC)");
    Serial.println("========================================\n");

    nfcSubsystemSetup();

    gwRuntimeCfg = GWWebConfig::getConfig(
        LORA_SPREADING_FACTOR, LORA_BANDWIDTH, LORA_CODING_RATE,
        LORA_TX_POWER, LORA_USE_RFO, 0);
    
    Serial.printf("Runtime: SF=%d BW=%dkHz CR=4/%d Pwr=%ddBm\n",
                  gwRuntimeCfg.sf, gwRuntimeCfg.bwKHz, gwRuntimeCfg.cr, gwRuntimeCfg.txPower);
    Serial.println("WiFi target: Auto-scan profile aktif (WiFiMulti).");
    
    mqttQueue = xQueueCreate(20, sizeof(MQTTQueueItem));
    if (!mqttQueue) { Serial.println("ERROR: Queue fail!"); while(1); }

    initLoRa();
    aodv.begin();
    aodv.onSendPacket = sendPacketCallback;

    xTaskCreatePinnedToCore(TaskLoRa, "LoRaTask", 10000, NULL, 1, &LoRaTaskHandle, 1);
    
    // WiFi & MQTT Init (auto-pilih dari semua profile di secrets.h)
    initWiFiProfiles();

    fmsMqtt.begin("master_nfc", "GATEWAY-01");
    fmsMqtt.setCommandCallback(handleRemoteCommand);
    
    initNTP();
    GWWebConfig::begin(WiFi.localIP().toString(), WiFi.softAPIP().toString(), gwRuntimeCfg);
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

// ================================================================
// MAIN LOOP
// ================================================================
void loop() {
    unsigned long now = millis();

    wifiHelper.update(now);
    wifiConnected = wifiHelper.connected();
    maintainNtpSync(now);

    maintainMqttConnection(now);
    fmsMqtt.loop();
    mqttConnected = fmsMqtt.isConnected();

    nfcSubsystemLoop(wifiConnected);

    // Proses queue MQTT
    if (wifiConnected && mqttConnected) {
        MQTTQueueItem item; int count = 0;
        while (xQueueReceive(mqttQueue, &item, 0) == pdTRUE && count++ < 5)
            publishQueueItem(item);
    }

    // Broadcast Time Sync
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
    if (now - lastStatus > 60000) { printStatus(); lastStatus = now; }

    handleSerialCLI();
    delay(10);
}

void TaskLoRa(void *pvParameters) {
    for (;;) {
        aodv.update();
        receivePackets();
        if (timeSyncRequested) { timeSyncRequested = false; sendTimeSyncPacket(); }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void initLoRa() {
    pinMode(LORA_RST_PIN, OUTPUT); digitalWrite(LORA_RST_PIN, HIGH);
    pinMode(LORA_DIO0_PIN, INPUT);
    digitalWrite(LORA_RST_PIN, LOW); delay(10);
    digitalWrite(LORA_RST_PIN, HIGH); delay(10);
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
    if (!rf95.init()) {
        Serial.println("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println("FATAL ERROR: Modul LoRa RFM95 TIDAK TERDETEKSI!");
        Serial.println("Periksa kabel SPI (MISO, MOSI, SCK, CS) di Gateway!");
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
    } else {
        Serial.println("LoRa Radio Hardware OK!");
    }
    rf95.setFrequency(LORA_FREQUENCY);
    float bwHz = gwRuntimeCfg.bwKHz * 1000.0f;
    int8_t pwr = gwRuntimeCfg.useRFO ? constrain(gwRuntimeCfg.txPower, 0, 15) : constrain(gwRuntimeCfg.txPower, 2, 20);
    rf95.setTxPower(pwr, gwRuntimeCfg.useRFO);
    rf95.setSpreadingFactor(gwRuntimeCfg.sf);
    rf95.setSignalBandwidth(bwHz);
    rf95.setCodingRate4(gwRuntimeCfg.cr);
    rf95.setPreambleLength(LORA_PREAMBLE_LENGTH);
    rf95.setCADTimeout(LORA_CAD_TIMEOUT_MS);

    Serial.printf("[LORA] Init: SF=%d, BW=%.1f kHz, CR=4/%d, Preamble=%d\n",
                  gwRuntimeCfg.sf, gwRuntimeCfg.bwKHz, gwRuntimeCfg.cr, LORA_PREAMBLE_LENGTH);
}

void initNTP() {
    if (!wifiConnected) return;
    
    // Gunakan server NTP lokal Indonesia dan Google, atur zona waktu WITA (UTC+8) untuk Makassar
    configTzTime("WITA-8", "id.pool.ntp.org", "time.google.com", "pool.ntp.org");
    
    Serial.println("[NTP] Memulai sinkronisasi waktu (timeout 20s)...");
    time_t now = 0;
    unsigned long t = millis();
    
    while (now < 1000000000L && millis()-t < 20000) { 
        delay(500); 
        time(&now); 
        Serial.print(".");
    }
    Serial.println();
    
    if (now > 1000000000L) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        ntpSynced = true;
        ntpBaseMillis = millis();
        ntpBaseEpochMs = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
        lastNtpSyncMs = ntpBaseMillis;
        Serial.println("[NTP] Berhasil tersinkronisasi!");
    } else {
        Serial.println("[NTP] GAGAL sinkronisasi (timeout)! Pastikan hotspot laptop Anda memiliki koneksi internet dan tidak memblokir port NTP (UDP 123).");
    }
}

void maintainNtpSync(unsigned long now) {
    if (!wifiConnected) return;

    if (!ntpSynced) {
        if (lastNtpAttemptMs == 0 || (now - lastNtpAttemptMs) >= NTP_RETRY_INTERVAL_MS) {
            lastNtpAttemptMs = now;
            Serial.println("[NTP] Attempt sync...");
            initNTP();
        }
        return;
    }

    if ((now - lastNtpSyncMs) >= NTP_RESYNC_INTERVAL_MS &&
        (now - lastNtpAttemptMs) >= NTP_RETRY_INTERVAL_MS) {
        lastNtpAttemptMs = now;
        Serial.println("[NTP] Periodic resync...");
        initNTP();
    }
}

void maintainMqttConnection(unsigned long now) {
    static bool lastWiFiConnectedState = false;
    static unsigned long wifiConnectedSince = 0;

    if (wifiConnected && !lastWiFiConnectedState) {
        wifiConnectedSince = now;
        lastMqttReconnectAttempt = 0;
        Serial.println("[MQTT] WiFi connected, MQTT start scheduled.");
    } else if (!wifiConnected && lastWiFiConnectedState) {
        wifiConnectedSince = 0;
        Serial.println("[MQTT] WiFi disconnected, MQTT waiting for WiFi.");
    }
    lastWiFiConnectedState = wifiConnected;

    if (!wifiConnected || fmsMqtt.isConnected()) {
        return;
    }

    if (!fmsMqtt.isRunning()) {
        if (wifiConnectedSince == 0) {
            wifiConnectedSince = now;
        }
        if ((uint32_t)(now - wifiConnectedSince) < GATEWAY_MQTT_START_DELAY_MS) {
            return;
        }
        if (lastMqttReconnectAttempt != 0 &&
            (uint32_t)(now - lastMqttReconnectAttempt) < GATEWAY_MQTT_RETRY_MS) {
            return;
        }

        lastMqttReconnectAttempt = now;
        Serial.println("[MQTT] Starting MQTT client...");
        if (!fmsMqtt.startClient()) {
            Serial.println("[MQTT] MQTT client start failed, retry scheduled.");
        }
        return;
    }

    if ((uint32_t)(now - lastMqttReconnectAttempt) >= GATEWAY_MQTT_RETRY_MS) {
        lastMqttReconnectAttempt = now;
        Serial.println("[MQTT] MQTT client running but disconnected, requesting reconnect...");
        fmsMqtt.connect();
    }
}

void handleRemoteCommand(const String &cmd) {
    if (cmd.length() > 0) {
        Serial.printf("[MQTT] Executing remote command: %s\n", cmd.c_str());
        if (cmd.equalsIgnoreCase("STATS"))            { printQoSStats(); }
        else if (cmd.equalsIgnoreCase("CSV"))              { printCSVStats(); }
        else if (cmd.equalsIgnoreCase("RESET") || cmd.equalsIgnoreCase("START")) {
            observationRound++;
            resetQoSStats();
        }
        else if (cmd.equalsIgnoreCase("STATUS"))           { printStatus(); }
        else if (cmd.equalsIgnoreCase("SYNC"))             { timeSyncRequested = true; }
        else if (!nfcHandleSerialCommand(cmd)) {
             Serial.printf("[MQTT] Unknown command: %s\n", cmd.c_str());
        }
    }
}

void publishQueueItem(const MQTTQueueItem& item) {
    if (!mqttConnected) return;
    const char* nodeName = getNodeName(item.sourceNodeID);

    if (item.sourceNodeID < MAX_TRACKED_NODES) {
        nodeStats[item.sourceNodeID].packetsReceived++;
        strncpy(nodeStats[item.sourceNodeID].nodeName, nodeName, 15);
    }

    uint32_t txTimestamp = 0;
    switch (item.kind) {
        case QUEUE_KIND_SENSOR: txTimestamp = item.data.txTimestamp; break;
        case QUEUE_KIND_FATIGUE_IMU: txTimestamp = item.fatigueImu.ts; break;
        case QUEUE_KIND_SAFETY_CONDITION: txTimestamp = item.safetyCondition.ts; break;
        case QUEUE_KIND_VEHICLE_TELEMETRY: txTimestamp = item.vehicleTelemetry.txTimestamp; break;
        default: break;
    }

    uint32_t latencyMs = 0;
    bool latencyValid = computeLatencyFromTx(txTimestamp, latencyMs);
    if (latencyValid && item.sourceNodeID < MAX_TRACKED_NODES) {
        nodeStats[item.sourceNodeID].latencySum += latencyMs;
        nodeStats[item.sourceNodeID].latencyCount++;
        if (latencyMs < nodeStats[item.sourceNodeID].latencyMin) nodeStats[item.sourceNodeID].latencyMin = latencyMs;
        if (latencyMs > nodeStats[item.sourceNodeID].latencyMax) nodeStats[item.sourceNodeID].latencyMax = latencyMs;
    }

    StaticJsonDocument<1536> doc;
    doc["seq"] = item.sequenceNum;
    doc["hopCount"] = item.hopCount;
    doc["rssi"] = item.rssi;
    doc["snr"] = item.snr;
    if (latencyValid) doc["latency_ms"] = latencyMs;

    JsonArray routePath = doc.createNestedArray("route_path");
    uint8_t rpLen = item.routePathLen > MAX_ROUTE_PATH ? MAX_ROUTE_PATH : item.routePathLen;
    for (uint8_t i = 0; i < rpLen; i++) {
        routePath.add(getNodeName(item.routePath[i]));
    }
    if (rpLen == 0 || item.routePath[rpLen - 1] != GATEWAY_ID) {
        routePath.add("GATEWAY");
    }

    char topic[96] = {0};
    if (item.kind == QUEUE_KIND_SENSOR) {
        doc["nodeId"] = item.sourceNodeID;
        doc["nodeName"] = nodeName;
        doc["epoch"] = item.data.txTimestamp;
        JsonObject gps = doc.createNestedObject("gps");
        gps["lat"] = item.data.gps.latitude;
        gps["lon"] = item.data.gps.longitude;
        gps["altitude"] = item.data.gps.altitude;
        doc["battery"] = item.data.batteryVoltage;
        doc["route_disc_ms"] = item.data.routeDiscMs;
        doc["route_hops"] = item.data.routeHops;
        snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_PREFIX, nodeName);
    } else if (item.kind == QUEUE_KIND_FATIGUE_IMU) {
        doc["nodeId"] = item.fatigueImu.nodeId;
        doc["epoch"] = item.fatigueImu.ts;
        doc["pitch"] = item.fatigueImu.pitch100 / 100.0f;
        doc["roll"] = item.fatigueImu.roll100 / 100.0f;
        doc["f_gx"] = item.fatigueImu.gx1000 / 1000.0f;
        doc["f_gy"] = item.fatigueImu.gy1000 / 1000.0f;
        doc["f_gz"] = item.fatigueImu.gz1000 / 1000.0f;
        doc["route_disc_ms"] = item.fatigueImu.routeDiscMs;
        doc["route_hops"] = item.fatigueImu.routeHops;
        doc["status_buzzer"] = item.fatigueImu.buzzerActive ? "ON" : "OFF";
        snprintf(topic, sizeof(topic), "%s", MQTT_TOPIC_FATIGUE_IMU);
    } else if (item.kind == QUEUE_KIND_SAFETY_CONDITION) {
        const uint8_t flags = item.safetyCondition.flags;
        doc["nodeId"] = item.safetyCondition.nodeId;
        doc["epoch"] = item.safetyCondition.ts;
        doc["gpsValid"] = (flags & FLAG_GPS_VALID) != 0;
        doc["drActive"] = (flags & FLAG_DR_ACTIVE) != 0;
        doc["rolloverRisk"] = (flags & FLAG_ROLLOVER_RISK) != 0;
        doc["rollover"] = (flags & FLAG_ROLLOVER) != 0;
        doc["harshBraking"] = (flags & FLAG_HARSH_BRAKE) != 0;
        doc["overspeed"] = (flags & FLAG_OVERSPEED) != 0;
        doc["flags"] = flags;
        doc["route_disc_ms"] = item.safetyCondition.routeDiscMs;
        doc["route_hops"] = item.safetyCondition.routeHops;
        snprintf(topic, sizeof(topic), "%s", MQTT_TOPIC_SAFETY_CONDITION);
    } else if (item.kind == QUEUE_KIND_VEHICLE_TELEMETRY) {
        doc["nodeId"] = item.sourceNodeID;
        doc["epoch"] = item.vehicleTelemetry.txTimestamp;
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
        doc["rollover"] = item.vehicleTelemetry.rollover;
        doc["rolloverRisk"] = item.vehicleTelemetry.rolloverRisk;
        doc["harshBraking"] = item.vehicleTelemetry.harshBraking;
        doc["overspeed"] = item.vehicleTelemetry.overspeed;
        doc["route_disc_ms"] = item.vehicleTelemetry.routeDiscMs;
        doc["route_hops"] = item.vehicleTelemetry.routeHops;
        JsonObject calibration = doc.createNestedObject("calibration");
        calibration["sys"] = item.vehicleTelemetry.calSys;
        calibration["gyro"] = item.vehicleTelemetry.calGyro;
        calibration["accel"] = item.vehicleTelemetry.calAccel;
        calibration["mag"] = item.vehicleTelemetry.calMag;
        snprintf(topic, sizeof(topic), "%s", MQTT_TOPIC_BNO_DATA);
    } else {
        return;
    }

    String json;
    serializeJson(doc, json);
    if (fmsMqtt.publishToTopic(topic, json)) {
        packetsSentToServer++;
    } else {
        Serial.printf("[MQTT] Publish failed topic=%s\n", topic);
    }
}

void printQoSStats() {
    unsigned long dur = millis() - observationStartTime;
    uint32_t exp = max(1UL, dur / NODE_SEND_INTERVAL_MS);
    Serial.println("\n--- QoS Stats ---");
    for (int i = 1; i < MAX_TRACKED_NODES; i++) {
        if (!nodeStats[i].packetsReceived && !nodeStats[i].nodeName[0]) continue;
        uint32_t rx = nodeStats[i].packetsReceived;
        float pdr = min(100.0f, rx * 100.0f / exp);
        Serial.printf("Node %s: Rx=%lu, PDR=%.1f%%\n", nodeStats[i].nodeName, rx, pdr);
    }
    if (mqttConnected) {
        StaticJsonDocument<192> s;
        s["round"] = observationRound;
        s["routeOK"] = aodv.routeDiscoverySuccess;
        s["routeFail"] = aodv.routeDiscoveryFail;
        String js; serializeJson(s, js);
        fmsMqtt.publishToTopic(MQTT_TOPIC_STATUS, js);
    }
}

void printCSVStats() {
    Serial.println("CSV: NODE,RX,EXP,PDR");
}

void resetQoSStats() {
    for (int i = 0; i < MAX_TRACKED_NODES; i++) nodeStats[i] = NodeStats();
    observationStartTime = millis();
}

void handleSerialCLI() {
    if (!Serial.available()) return;
    String raw = Serial.readStringUntil('\n');
    raw.trim();
    if (raw.length() == 0) return;
    if (raw.equalsIgnoreCase("STATS")) {
        printQoSStats();
    } else if (raw.equalsIgnoreCase("CSV")) {
        printCSVStats();
    } else if (raw.equalsIgnoreCase("START")) {
        observationRound++;
        resetQoSStats();
        Serial.println("[CLI] New observation period started");
    } else if (raw.startsWith("TEST ")) {
        // Parse: TEST SF7 BW125
        String cmd = raw;
        cmd.toUpperCase();
        uint8_t newSF = 7;
        uint32_t newBW = 125;
        int sfIdx = cmd.indexOf("SF");
        int bwIdx = cmd.indexOf("BW");
        if (sfIdx >= 0) newSF = cmd.substring(sfIdx + 2).toInt();
        if (bwIdx >= 0) newBW = cmd.substring(bwIdx + 2).toInt();
        newSF = constrain(newSF, 7, 12);
        newBW = (newBW == 250) ? 250 : 125;

        Serial.printf("[CLI] Broadcasting START_TEST: SF=%d BW=%lukHz\n", newSF, (unsigned long)newBW);

        StartTestPayload st;
        st.sf = newSF;
        st.bwKHz = newBW;
        LoRaPacket pkt = LoRaPacketHandler::createStartTestPacket(GATEWAY_ID, st);
        sendPacketCallback(pkt);
        delay(500);
        sendPacketCallback(pkt);

        Serial.println("[CLI] Gateway akan reboot dengan parameter baru dalam 3 detik...");
        GWWebConfig::saveTestConfig(newSF, newBW);
        delay(3000);
        ESP.restart();
    } else if (!nfcHandleSerialCommand(raw)) {
        Serial.println("Unknown command");
    }
}

const char* getNodeName(uint8_t id) {
    switch (id) {
        case GATEWAY_ID: return "GATEWAY";
        case 1: return "TRK-001";
        case 2: return "TRK-002";
        case 3: return "TRK-003";
        case 4: return "lora_saenab";
        case 5: return "lora_nailah";
        case BROADCAST_ADDR: return "BROADCAST";
        default: return "UNKNOWN";
    }
}

void printStatus() {
    Serial.printf("WiFi: %s | MQTT: %s | NTP: %s\n",
                  wifiConnected?"OK":"FAIL", mqttConnected?"OK":"FAIL", ntpSynced?"SYNCED":"NO SYNC");
}

uint64_t getEpochMs64() {
    if (!ntpSynced) return (uint64_t)millis();
    return ntpBaseEpochMs + (uint64_t)(millis() - ntpBaseMillis);
}

uint32_t getEpochMsLow32() { return (uint32_t)getEpochMs64(); }

bool computeLatencyFromTx(uint32_t txTimestamp, uint32_t& latencyMs) {
    latencyMs = 0;
    if (!ntpSynced || txTimestamp == 0) return false;
    uint32_t nowLow32 = getEpochMsLow32();
    latencyMs = nowLow32 - txTimestamp;
    return latencyMs < 60000UL;
}

void sendTimeSyncPacket() {
    if (!ntpSynced) return;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    TimeSyncPayload ts;
    ts.epochSeconds = (uint32_t)tv.tv_sec;
    ts.millisPart = (uint16_t)(tv.tv_usec / 1000);
    LoRaPacket pkt = LoRaPacketHandler::createTimeSyncPacket(NODE_ID, ts);
    uint8_t buf[255]; int len = packetHandler.serializePacket(pkt, buf, sizeof(buf));
    if (len > 0) { rf95.send(buf, len); rf95.waitPacketSent(); rf95.setModeRx(); }
}

void receivePackets() {
    if (!rf95.available()) return;
    uint8_t buf[255]; uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
        packetsReceivedTotal++;
        LoRaPacket pkt;
        if (packetHandler.deserializePacket(buf, len, pkt)) handleReceivedPacket(pkt);
    }
}

bool shouldAckDataPacketType(uint8_t packetType) {
    return packetType == PKT_TYPE_DATA ||
           packetType == PKT_TYPE_FATIGUE_IMU ||
           packetType == PKT_TYPE_SAFETY_CONDITION ||
           packetType == PKT_TYPE_VEHICLE_TELEMETRY;
}

void sendDataAckForPacket(const LoRaPacket& packet) {
    if (!DATA_ACK_ENABLE || !shouldAckDataPacketType(packet.header.packetType)) return;
    if (packet.header.destinationID != GATEWAY_ID || packet.header.nextHop != NODE_ID) return;

    AckPayload ackPayload = {};
    ackPayload.ackedPacketType = packet.header.packetType;
    ackPayload.ackedSequence = packet.header.sequenceNum;

    aodv.incrementSequenceNumber();
    LoRaPacket ackPacket = LoRaPacketHandler::createAckPacket(
        NODE_ID, packet.header.sourceID, ackPayload, aodv.getSequenceNumber());

    uint8_t nextHop = aodv.getNextHop(packet.header.sourceID);
    if (nextHop == 0 && packet.header.sourceID != 0) {
        nextHop = packet.header.sourceID;
    }
    ackPacket.header.nextHop = nextHop;
    ackPacket.header.checksum = LoRaPacketHandler::calculateChecksum(ackPacket);

    sendPacketCallback(ackPacket);
    Serial.printf("[ACK] Sent to node %d for seq %lu type=0x%02X\n",
                  packet.header.sourceID,
                  (unsigned long)packet.header.sequenceNum,
                  packet.header.packetType);
}

// Cache payload yang sudah diproses agar retry ACK tidak masuk MQTT/log dua kali.
// Kunci memakai payloadHash, bukan header checksum, supaya retry lewat route berbeda tetap dianggap duplikat.
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
            Serial.printf("[RX] DUPLICATE payload src=%u seq=%lu type=0x%02X (ACK ulang, tidak publish)\n",
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
    int8_t rssi = rf95.lastRssi();
    int8_t snr  = rf95.lastSNR();

    bool isDuplicate = isDuplicateUplinkPayload(packet);

    switch (packet.header.packetType) {
        case PKT_TYPE_DATA: {
            if (packet.header.destinationID != GATEWAY_ID || packet.header.nextHop != NODE_ID) {
                break;
            }
            if (packet.header.payloadLength != sizeof(SensorDataPayload)) {
                Serial.printf("[RX] DATA size mismatch: got=%u expected=%u\n",
                              packet.header.payloadLength, (unsigned)sizeof(SensorDataPayload));
                break;
            }
            SensorDataPayload sd; 
            memcpy(&sd, packet.payload, sizeof(SensorDataPayload));
            if (!isDuplicate) {
                enqueueSensorData(sd,
                                  packet.header.sourceID,
                                  packet.header.sequenceNum,
                                  rssi,
                                  snr,
                                  packet.header.hopCount,
                                  packet.header.routePathLen,
                                  packet.header.routePath);
            }
            sendDataAckForPacket(packet);
            break;
        }
        case PKT_TYPE_FATIGUE_IMU: {
            if (packet.header.destinationID != GATEWAY_ID || packet.header.nextHop != NODE_ID) {
                break;
            }
            if (packet.header.payloadLength != sizeof(ImuFatiguePayload)) {
                Serial.printf("[RX] FATIGUE_IMU size mismatch: got=%u expected=%u\n",
                              packet.header.payloadLength, (unsigned)sizeof(ImuFatiguePayload));
                break;
            }
            ImuFatiguePayload imu = {};
            memcpy(&imu, packet.payload, sizeof(imu));
            if (imu.packetType != PKT_TYPE_FATIGUE_IMU) {
                Serial.printf("[RX] FATIGUE_IMU marker mismatch: got=0x%02X expected=0x%02X\n",
                              imu.packetType, PKT_TYPE_FATIGUE_IMU);
                break;
            }
            if (!isDuplicate) {
                enqueueFatigueImuData(imu,
                                      packet.header.sourceID,
                                      packet.header.sequenceNum,
                                      rssi,
                                      snr,
                                      packet.header.hopCount,
                                      packet.header.routePathLen,
                                      packet.header.routePath);
            }
            sendDataAckForPacket(packet);
            break;
        }
        case PKT_TYPE_SAFETY_CONDITION: {
            if (packet.header.destinationID != GATEWAY_ID || packet.header.nextHop != NODE_ID) {
                break;
            }
            if (packet.header.payloadLength != sizeof(SafetyConditionPayload)) {
                Serial.printf("[RX] SAFETY size mismatch: got=%u expected=%u\n",
                              packet.header.payloadLength, (unsigned)sizeof(SafetyConditionPayload));
                break;
            }
            SafetyConditionPayload sc = {};
            memcpy(&sc, packet.payload, sizeof(sc));
            if (sc.packetType != PKT_TYPE_SAFETY_CONDITION) {
                Serial.printf("[RX] SAFETY marker mismatch: got=0x%02X expected=0x%02X\n",
                              sc.packetType, PKT_TYPE_SAFETY_CONDITION);
                break;
            }
            if (!isDuplicate) {
                enqueueSafetyConditionData(sc,
                                           packet.header.sourceID,
                                           packet.header.sequenceNum,
                                           rssi,
                                           snr,
                                           packet.header.hopCount,
                                           packet.header.routePathLen,
                                           packet.header.routePath);
            }
            sendDataAckForPacket(packet);
            break;
        }
        case PKT_TYPE_VEHICLE_TELEMETRY: {
            if (packet.header.destinationID != GATEWAY_ID || packet.header.nextHop != NODE_ID) {
                break;
            }
            if (packet.header.payloadLength != sizeof(VehicleTelemetryPayload)) {
                Serial.printf("[RX] VEHICLE size mismatch: got=%u expected=%u\n",
                              packet.header.payloadLength, (unsigned)sizeof(VehicleTelemetryPayload));
                break;
            }
            VehicleTelemetryPayload vt = {};
            memcpy(&vt, packet.payload, sizeof(vt));
            if (!isDuplicate) {
                enqueueVehicleTelemetryData(vt,
                                            packet.header.sourceID,
                                            packet.header.sequenceNum,
                                            rssi,
                                            snr,
                                            packet.header.hopCount,
                                            packet.header.routePathLen,
                                            packet.header.routePath);
            }
            sendDataAckForPacket(packet);
            break;
        }
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
        case PKT_TYPE_ACK:
            // Gateway doesn't process ACK from nodes generally unless it sends DATA
            break;
        case PKT_TYPE_DIAGNOSTIC:
            if (packet.header.destinationID == GATEWAY_ID &&
                packet.header.payloadLength == sizeof(DiscoveryDiagPayload)) {
                DiscoveryDiagPayload diag;
                memcpy(&diag, packet.payload, sizeof(diag));

                Serial.printf("[DIAG] node=%u target=%u success=%u hops=%u retries=%u discovery=%lums\n",
                              diag.originNodeId, diag.targetNodeId, diag.success,
                              diag.hopCount, diag.retryCount, (unsigned long)diag.discoveryMs);

                if (mqttConnected) {
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
                    doc["snr"] = snr;

                    String js;
                    serializeJson(doc, js);
                    fmsMqtt.publishToTopic(MQTT_TOPIC_DIAGNOSTIC, js);
                }
            }
            break;
        case PKT_TYPE_START_TEST:
            // Umumnya gateway hanya mengirim START_TEST, bukan menerima.
            Serial.println("[RX] START_TEST received on gateway (ignored)");
            break;
        default:
            Serial.printf("[RX] Unknown packet type 0x%02X\n", packet.header.packetType);
            break;
    }
}

void sendPacketCallback(const LoRaPacket& packet) {
    uint8_t buf[255]; int len = packetHandler.serializePacket(packet, buf, sizeof(buf));
    if (len > 0) { rf95.send(buf, len); rf95.waitPacketSent(); rf95.setModeRx(); }
}

void enqueueSensorData(const SensorDataPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops, uint8_t routePathLen, const uint8_t* routePath) {
    if (!mqttQueue) return;
    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_SENSOR;
    item.data = data;
    item.sourceNodeID = src;
    item.sequenceNum = seqNum;
    item.rssi = rssi;
    item.snr = snr;
    item.hopCount = hops;
    item.routePathLen = routePathLen > MAX_ROUTE_PATH ? MAX_ROUTE_PATH : routePathLen;
    if (routePath != nullptr && item.routePathLen > 0) {
        memcpy(item.routePath, routePath, item.routePathLen);
    }
    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("WARNING: Queue full, drop sensor pkt");
    }
}

void enqueueFatigueImuData(const ImuFatiguePayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops, uint8_t routePathLen, const uint8_t* routePath) {
    if (!mqttQueue) return;
    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_FATIGUE_IMU;
    item.fatigueImu = data;
    item.sourceNodeID = src;
    item.sequenceNum = seqNum;
    item.rssi = rssi;
    item.snr = snr;
    item.hopCount = hops;
    item.routePathLen = routePathLen > MAX_ROUTE_PATH ? MAX_ROUTE_PATH : routePathLen;
    if (routePath != nullptr && item.routePathLen > 0) {
        memcpy(item.routePath, routePath, item.routePathLen);
    }
    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("WARNING: Queue full, drop fatigue pkt");
    }
}

void enqueueSafetyConditionData(const SafetyConditionPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops, uint8_t routePathLen, const uint8_t* routePath) {
    if (!mqttQueue) return;
    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_SAFETY_CONDITION;
    item.safetyCondition = data;
    item.sourceNodeID = src;
    item.sequenceNum = seqNum;
    item.rssi = rssi;
    item.snr = snr;
    item.hopCount = hops;
    item.routePathLen = routePathLen > MAX_ROUTE_PATH ? MAX_ROUTE_PATH : routePathLen;
    if (routePath != nullptr && item.routePathLen > 0) {
        memcpy(item.routePath, routePath, item.routePathLen);
    }
    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("WARNING: Queue full, drop safety pkt");
    }
}

void enqueueVehicleTelemetryData(const VehicleTelemetryPayload& data, uint8_t src, uint32_t seqNum, int8_t rssi, int8_t snr, uint8_t hops, uint8_t routePathLen, const uint8_t* routePath) {
    if (!mqttQueue) return;
    MQTTQueueItem item = {};
    item.kind = QUEUE_KIND_VEHICLE_TELEMETRY;
    item.vehicleTelemetry = data;
    item.sourceNodeID = src;
    item.sequenceNum = seqNum;
    item.rssi = rssi;
    item.snr = snr;
    item.hopCount = hops;
    item.routePathLen = routePathLen > MAX_ROUTE_PATH ? MAX_ROUTE_PATH : routePathLen;
    if (routePath != nullptr && item.routePathLen > 0) {
        memcpy(item.routePath, routePath, item.routePathLen);
    }
    if (xQueueSend(mqttQueue, &item, 10) != pdPASS) {
        Serial.println("WARNING: Queue full, drop vehicle pkt");
    }
}
