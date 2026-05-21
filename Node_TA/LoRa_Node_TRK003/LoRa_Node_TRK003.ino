/*
 * LoRa Mesh Node Firmware v2.3
 * ESP32-S3 + RFM95 | AODV Routing
 * Project: PERANCANGAN SISTEM KOMUNIKASI DATA IOT BERBASIS LORA MESH AD HOC
 *
 * v2.3: WiFi AP Web Config - ubah SF/BW tanpa re-flash
 *       Boot -> connect ke SSID config node tanpa password
 *       Buka http://192.168.4.1/ -> simpan -> node reboot dengan config baru
 */

#include <SPI.h>
#include <RH_RF95.h>
#include "config.h"
#include "LoRa_Packet.h"
#include "AODV_Routing.h"
#include "WebConfig.h"   // WiFi AP Config UI

// ================================================================
// KONFIGURASI NODE - Ganti untuk setiap unit hardware
// ================================================================
#define NODE_ID   3
#define NODE_NAME "TRK-003"

// ================================================================
// RUNTIME CONFIG - diisi dari NVRAM atau default config.h
// ================================================================
LoRaRuntimeCfg runtimeCfg;

// ================================================================
// GLOBAL OBJECTS
// ================================================================
RH_RF95 rf95(LORA_CS_PIN, LORA_DIO0_PIN);
AODVRouting aodv(NODE_ID);
LoRaPacketHandler packetHandler;

SensorDataPayload sensorData;
unsigned long lastGPSUpdate = 0;
unsigned long lastIMUUpdate = 0;
float gpsLatOffset = 0, gpsLonOffset = 0, gpsAltOffset = 0;

// Time Sync (dari Gateway)
uint32_t epochOffsetMsLow32 = 0;
bool timeSynced             = false;

// Data packet sequence counter (independen dari AODV sequence)
uint32_t dataSequence = 0;

// DUPLICATE PACKET DETECTION CACHE
#define DUP_CACHE_SIZE 32
#define DUP_CACHE_TTL_MS 10000
struct DupCacheEntry { uint8_t sourceID; uint32_t seqNum; uint8_t pktType; unsigned long timestamp; bool valid; };
DupCacheEntry dupCache[DUP_CACHE_SIZE]; uint8_t dupCacheIdx = 0;
bool isDuplicate(uint8_t src, uint32_t seq, uint8_t type) {
    unsigned long now = millis();
    for (int i = 0; i < DUP_CACHE_SIZE; i++) {
        if (dupCache[i].valid && dupCache[i].sourceID == src && dupCache[i].seqNum == seq
            && dupCache[i].pktType == type && (now - dupCache[i].timestamp) < DUP_CACHE_TTL_MS) return true;
    } return false;
}
void addToDupCache(uint8_t src, uint32_t seq, uint8_t type) {
    dupCache[dupCacheIdx] = {src, seq, type, millis(), true};
    dupCacheIdx = (dupCacheIdx + 1) % DUP_CACHE_SIZE;
}

// ================================================================
// FUNCTION PROTOTYPES
// ================================================================
void initLoRa();
void initSensors();
void updateGPSData();
void updateIMUData();
bool sendSensorData();
void receivePackets();
void handleReceivedPacket(const LoRaPacket& packet);
void sendPacketCallback(const LoRaPacket& packet);

// ================================================================
// SETUP
// ================================================================
void sendDiagnosticCallback(const DiscoveryDiagPayload& diag) {
    if (!aodv.hasRouteTo(GATEWAY_ID)) return;
    static uint32_t diagSeq = 0;
    LoRaPacket pkt = LoRaPacketHandler::createDiagnosticPacket(
        NODE_ID, GATEWAY_ID, diag, ++diagSeq);
    pkt.header.nextHop = aodv.getNextHop(GATEWAY_ID);
    pkt.header.checksum = LoRaPacketHandler::calculateChecksum(pkt);
    sendPacketCallback(pkt);
    Serial.printf("[DIAG TX] target=%u discovery=%lums hops=%u success=%u\n",
                  diag.targetNodeId, (unsigned long)diag.discoveryMs,
                  diag.hopCount, diag.success);
}
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);

    Serial.println("\n========================================");
    Serial.printf("  LoRa Node v2.3 ??? %s (ID=%d)\n", NODE_NAME, NODE_ID);
    Serial.println("========================================");

    // 1. Baca config dari NVRAM (atau default dari config.h)
    runtimeCfg = WebConfig::getConfig(
        LORA_SPREADING_FACTOR,
        LORA_BANDWIDTH,
        LORA_CODING_RATE,
        LORA_TX_POWER,
        LORA_USE_RFO
    );
    Serial.printf("Runtime: SF=%d BW=%dkHz CR=4/%d Pwr=%ddBm\n",
                  runtimeCfg.sf, runtimeCfg.bwKHz, runtimeCfg.cr, runtimeCfg.txPower);

    // 2. Mulai WiFi AP Config Mode (selalu aktif selama node menyala)
    //    LoRa dan WiFi AP bisa berjalan bersamaan di ESP32-S3
    WebConfig::begin(NODE_NAME, runtimeCfg, NODE_ID);

    // 3. Init LoRa dengan runtime config
    initLoRa();

    // 4. Init AODV Routing
    aodv.begin();
    aodv.onSendPacket = sendPacketCallback;
    aodv.onDiagnosticReady = sendDiagnosticCallback;
    aodv.epochOffsetPtr = &epochOffsetMsLow32;

    // 5. Init dummy sensors
    initSensors();

    Serial.printf("\nSetup complete! %s siap.\n", NODE_NAME);
    Serial.println("Ketik 'RESET' di Serial untuk reset config NVRAM.\n");
}

// ================================================================
// MAIN LOOP — Interval Tetap 3 Detik + Offset Per Node
// ================================================================
unsigned long lastSendTime = 0;
bool initialOffsetDone = false;

void loop() {
    unsigned long now = millis();

    WebConfig::handle();
    aodv.update();

    if (now - lastGPSUpdate > GPS_UPDATE_INTERVAL) { updateGPSData(); lastGPSUpdate = now; }
    if (now - lastIMUUpdate > IMU_UPDATE_INTERVAL)  { updateIMUData(); lastIMUUpdate = now; }

    // FIXED INTERVAL SCHEDULING (3 detik + offset NODE_ID * 500ms)
    if (!initialOffsetDone) {
        lastSendTime = now - DATA_SEND_INTERVAL + (NODE_ID * 500UL);
        initialOffsetDone = true;
    }

    if (now - lastSendTime >= DATA_SEND_INTERVAL) {
        bool ok = sendSensorData();
        if (ok) {
            lastSendTime = now;
            Serial.printf("[TX] Sent OK | SF=%d BW=%dkHz | interval=%dms\n",
                          runtimeCfg.sf, runtimeCfg.bwKHz, DATA_SEND_INTERVAL);
        }
    }

    receivePackets();

    static unsigned long lastPrint = 0;
    if (now - lastPrint > 60000) {
        Serial.printf("[STATUS] Route OK=%d GAGAL=%d | Sync=%s EpochLow32=%lu\n",
                      aodv.routeDiscoverySuccess, aodv.routeDiscoveryFail,
                      timeSynced?"YA":"BELUM", (unsigned long)epochOffsetMsLow32);
        Serial.printf("[CONFIG] SF=%d BW=%dkHz Interval=%dms Offset=%dms\n",
                      runtimeCfg.sf, runtimeCfg.bwKHz,
                      DATA_SEND_INTERVAL, NODE_ID * 500);
        lastPrint = now;
    }

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim(); cmd.toUpperCase();
        if (cmd == "RESET") {
            WebConfig::resetConfig();
            Serial.println("NVRAM direset. Reboot untuk gunakan default.");
        }
    }

    delay(10);
}


// ================================================================
// LORA INITIALIZATION ??? Gunakan runtimeCfg dari NVRAM/default
// ================================================================
void initLoRa() {
    Serial.println("Initializing LoRa...");
    pinMode(LORA_RST_PIN, OUTPUT); digitalWrite(LORA_RST_PIN, HIGH);
    pinMode(LORA_DIO0_PIN, INPUT);
    digitalWrite(LORA_RST_PIN, LOW); delay(10);
    digitalWrite(LORA_RST_PIN, HIGH); delay(10);
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);

    if (!rf95.init()) { Serial.println("ERROR: LoRa init fail!"); while(1); }
    if (!rf95.setFrequency(LORA_FREQUENCY)) { Serial.println("ERROR: Freq fail!"); while(1); }

    // Gunakan RUNTIME CONFIG - diisi dari NVRAM atau default config.h
    float bwHz = runtimeCfg.bwKHz * 1000.0f;
    int8_t pwr = runtimeCfg.useRFO
                 ? constrain(runtimeCfg.txPower, 0, 15)
                 : constrain(runtimeCfg.txPower, 2, 20);

    rf95.setSpreadingFactor(runtimeCfg.sf);
    rf95.setSignalBandwidth(bwHz);
    rf95.setCodingRate4(runtimeCfg.cr);
    rf95.setTxPower(pwr, runtimeCfg.useRFO);
    rf95.setPreambleLength(LORA_PREAMBLE_LENGTH);

    Serial.printf("LoRa OK ??? SF=%d BW=%.0fkHz CR=4/%d Pwr=%ddBm (%s)\n",
                  runtimeCfg.sf, bwHz/1000.0, runtimeCfg.cr, pwr,
                  runtimeCfg.useRFO ? "RFO" : "PA_BOOST");
}

// ================================================================
// SENSOR INIT (DUMMY)
// ================================================================
void initSensors() {
    randomSeed(analogRead(0) + NODE_ID * 1000);
    gpsLatOffset = (NODE_ID - 1) * 0.002f;
    gpsLonOffset = (NODE_ID - 1) * 0.002f;
    gpsAltOffset = NODE_ID * 5.0f;
    sensorData.batteryVoltage = 3.7f + (random(0, 100) / 100.0f);
    sensorData.txTimestamp    = 0;
    updateGPSData(); updateIMUData();
    Serial.println("Sensors initialized (dummy mode).");
}

void updateGPSData() {
    sensorData.gps.latitude  = GPS_LAT_BASE + gpsLatOffset + (random(-10,10)/10000.0f);
    sensorData.gps.longitude = GPS_LON_BASE + gpsLonOffset + (random(-10,10)/10000.0f);
    sensorData.gps.altitude  = GPS_ALT_BASE + gpsAltOffset + (random(-5,5)/10.0f);
    sensorData.gps.timestamp = millis();
}

void updateIMUData() {
    sensorData.imu.accelX = (random(-100,100)/100.0f)*0.2f;
    sensorData.imu.accelY = (random(-100,100)/100.0f)*0.2f;
    sensorData.imu.accelZ = 1.0f + (random(-50,50)/1000.0f);
    sensorData.imu.gyroX  = random(-100,100)/10.0f;
    sensorData.imu.gyroY  = random(-100,100)/10.0f;
    sensorData.imu.gyroZ  = random(-100,100)/10.0f;
    sensorData.imu.timestamp = millis();
}

// ================================================================
// SEND SENSOR DATA
// ================================================================
bool sendSensorData() {
    sensorData.batteryVoltage = 3.7f + (random(0,100)/100.0f);
    sensorData.signalStrength = rf95.lastRssi();
    sensorData.txTimestamp    = timeSynced
        ? ((uint32_t)millis() + epochOffsetMsLow32)
        : (uint32_t)millis();
    sensorData.routeDiscMs = 0;
    sensorData.routeHops = 0;

    if (!aodv.hasRouteTo(GATEWAY_ID)) {
        Serial.println("No route, initiating discovery...");
        aodv.initiateRouteDiscovery(GATEWAY_ID);
        return false;
    }

    uint32_t lastDiscoveryMs = 0;
    uint8_t lastDiscoveryHops = 0;
    if (aodv.getLastSuccessfulDiscovery(GATEWAY_ID, lastDiscoveryMs, lastDiscoveryHops)) {
        sensorData.routeDiscMs = lastDiscoveryMs;
    }

    sensorData.routeHops = aodv.getRouteHopCount(GATEWAY_ID);
    if (sensorData.routeHops == 0) {
        sensorData.routeHops = lastDiscoveryHops;
    }

    LoRaPacket pkt = packetHandler.createDataPacket(
        NODE_ID, GATEWAY_ID, sensorData, ++dataSequence);
    pkt.header.nextHop  = aodv.getNextHop(GATEWAY_ID);
    pkt.header.checksum = LoRaPacketHandler::calculateChecksum(pkt);

    Serial.printf("[TX] Data-> SF=%d BW=%dkHz Sync=%s\n",
                  runtimeCfg.sf, runtimeCfg.bwKHz, timeSynced?"YA":"NO");
    sendPacketCallback(pkt);
    return true;
}

// ================================================================
// RECEIVE PACKETS
// ================================================================
void receivePackets() {
    if (!rf95.available()) return;
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
        LoRaPacket pkt;
        if (packetHandler.deserializePacket(buf, len, pkt)) handleReceivedPacket(pkt);
    }
}

void handleReceivedPacket(const LoRaPacket& packet) {
    if (packet.header.sourceID == NODE_ID) return;
    uint8_t ptype = packet.header.packetType;
    if (ptype == PKT_TYPE_DATA || ptype == PKT_TYPE_FATIGUE_IMU ||
        ptype == PKT_TYPE_SAFETY_CONDITION || ptype == PKT_TYPE_VEHICLE_TELEMETRY ||
        ptype == PKT_TYPE_DIAGNOSTIC) {
        if (isDuplicate(packet.header.sourceID, packet.header.sequenceNum, ptype)) return;
        addToDupCache(packet.header.sourceID, packet.header.sequenceNum, ptype);
    }
    switch (ptype) {
        case PKT_TYPE_DATA: case PKT_TYPE_FATIGUE_IMU: case PKT_TYPE_FATIGUE_STATUS:
        case PKT_TYPE_SAFETY_CONDITION: case PKT_TYPE_VEHICLE_TELEMETRY: case PKT_TYPE_DIAGNOSTIC:
            if (packet.header.nextHop == NODE_ID && aodv.hasRouteTo(packet.header.destinationID)
                && packet.header.hopCount < MAX_HOP_COUNT) {
                LoRaPacket fwd = packet; fwd.header.hopCount++;
                if (fwd.header.routePathLen < MAX_ROUTE_PATH) {
                    fwd.header.routePath[fwd.header.routePathLen] = NODE_ID;
                    fwd.header.routePathLen++;
                }
                fwd.header.nextHop = aodv.getNextHop(packet.header.destinationID);
                fwd.header.checksum = LoRaPacketHandler::calculateChecksum(fwd);
                sendPacketCallback(fwd);
            }
            break;
        case PKT_TYPE_RREQ: aodv.handleRREQ(packet); break;
        case PKT_TYPE_RREP: aodv.handleRREP(packet); break;
        case PKT_TYPE_RERR: aodv.handleRERR(packet); break;
        case PKT_TYPE_HELLO: aodv.handleHello(packet); break;
        case PKT_TYPE_TIMESYNC:
            if (packet.header.payloadLength == sizeof(TimeSyncPayload)) {
                TimeSyncPayload ts; memcpy(&ts, packet.payload, sizeof(TimeSyncPayload));
                uint32_t epochMs = (uint32_t)((uint64_t)ts.epochSeconds * 1000ULL + ts.millisPart);
                epochOffsetMsLow32 = epochMs - (uint32_t)millis(); timeSynced = true;
                Serial.printf("[TIMESYNC] OK EpochLow32=%lu\n", (unsigned long)epochOffsetMsLow32);
            }
            break;
        case PKT_TYPE_START_TEST:
            if (packet.header.payloadLength == sizeof(StartTestPayload)) {
                StartTestPayload st; memcpy(&st, packet.payload, sizeof(StartTestPayload));
                Serial.printf("[START_TEST] SF=%u BW=%lukHz\n", st.sf, (unsigned long)st.bwKHz);
                WebConfig::saveTestConfig(st.sf, st.bwKHz); delay(2000); ESP.restart();
            }
            break;
    }
}

void sendPacketCallback(const LoRaPacket& packet) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    int len = packetHandler.serializePacket(packet, buf, sizeof(buf));
    delay(random(30, 150));
    if (len > 0) { rf95.send(buf, len); rf95.waitPacketSent(); rf95.setModeRx(); }
}



