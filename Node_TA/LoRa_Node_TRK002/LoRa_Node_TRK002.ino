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
#define NODE_ID   2
#define NODE_NAME "TRK-002"

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

static bool pendingDataAck = false;
static LoRaPacket pendingDataPacket;
static uint32_t pendingDataSeq = 0;
static uint8_t pendingDataRetries = 0;
static uint8_t consecutiveAckFailures = 0;
static unsigned long pendingDataLastTxMs = 0;

// ================================================================
// DUPLICATE PACKET DETECTION CACHE
// ================================================================
#define DUP_CACHE_SIZE 32
#define DUP_CACHE_TTL_MS 10000  // Paket dianggap duplikat jika diterima dalam 10 detik
struct DupCacheEntry {
    uint8_t sourceID;
    uint32_t seqNum;
    uint8_t pktType;
    unsigned long timestamp;
    bool valid;
};
DupCacheEntry dupCache[DUP_CACHE_SIZE];
uint8_t dupCacheIdx = 0;

bool isDuplicate(uint8_t src, uint32_t seq, uint8_t type) {
    unsigned long now = millis();
    for (int i = 0; i < DUP_CACHE_SIZE; i++) {
        if (dupCache[i].valid &&
            dupCache[i].sourceID == src &&
            dupCache[i].seqNum == seq &&
            dupCache[i].pktType == type &&
            (now - dupCache[i].timestamp) < DUP_CACHE_TTL_MS) {
            return true;
        }
    }
    return false;
}

void addToDupCache(uint8_t src, uint32_t seq, uint8_t type) {
    dupCache[dupCacheIdx].sourceID = src;
    dupCache[dupCacheIdx].seqNum = seq;
    dupCache[dupCacheIdx].pktType = type;
    dupCache[dupCacheIdx].timestamp = millis();
    dupCache[dupCacheIdx].valid = true;
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
uint32_t getDataAckTimeoutMs();
uint8_t getDataAckMaxRetries();
void processDataAckTimeout(unsigned long nowMs);
void receivePackets();
void handleReceivedPacket(const LoRaPacket& packet);
void sendPacketCallback(const LoRaPacket& packet);

// ================================================================
// SETUP
// ================================================================
// Callback untuk mengirim diagnostic route discovery ke Gateway via LoRa
void sendDiagnosticCallback(const DiscoveryDiagPayload& diag) {
    if (!aodv.hasRouteTo(GATEWAY_ID)) return;

    static uint32_t diagSeq = 0;
    LoRaPacket pkt = LoRaPacketHandler::createDiagnosticPacket(
        NODE_ID, GATEWAY_ID, diag, ++diagSeq);
    pkt.header.nextHop = aodv.getNextHop(GATEWAY_ID);
    pkt.header.checksum = LoRaPacketHandler::calculateChecksum(pkt);
    sendPacketCallback(pkt);
    Serial.printf("[DIAG TX] target=%u discovery=%lums hops=%u retries=%u success=%u\n",
                  diag.targetNodeId, (unsigned long)diag.discoveryMs,
                  diag.hopCount, diag.retryCount, diag.success);
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
// MAIN LOOP ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â Interval + Payload Jitter Acak + Offset Per Node
// ================================================================
// Setiap node mengirim data setiap DATA_SEND_INTERVAL + jitter acak.
// Offset awal = NODE_ID * 500ms untuk menghindari tabrakan saat boot.

unsigned long lastLoRaSendTime = 0;
unsigned long currentSendIntervalMs = DATA_SEND_INTERVAL;
bool initialOffsetDone = false;
static const uint16_t PAYLOAD_JITTER_MIN_MS = 100;
static const uint16_t PAYLOAD_JITTER_MAX_MS = 500;

static unsigned long nextPayloadIntervalMs() {
    return DATA_SEND_INTERVAL + (unsigned long)random(PAYLOAD_JITTER_MIN_MS, PAYLOAD_JITTER_MAX_MS + 1);
}

void loop() {
    unsigned long now = millis();

    // Handle WebConfig (SSID config tetap aktif selama node menyala)
    WebConfig::handle();

    aodv.update();

    if (now - lastGPSUpdate > GPS_UPDATE_INTERVAL) { updateGPSData(); lastGPSUpdate = now; }
    if (now - lastIMUUpdate > IMU_UPDATE_INTERVAL)  { updateIMUData(); lastIMUUpdate = now; }
    receivePackets();
    processDataAckTimeout(now);

    // ============================================================
    // FIXED BASE INTERVAL + RANDOM JITTER (+ offset NODE_ID * 500ms)
    // ============================================================
    if (!initialOffsetDone) {
        // Offset awal agar node tidak kirim bersamaan saat boot
        currentSendIntervalMs = nextPayloadIntervalMs();
        lastLoRaSendTime = now - currentSendIntervalMs + (NODE_ID * 500UL);
        initialOffsetDone = true;
        Serial.printf("[TX] Payload jitter aktif: +%u..+%u ms\n",
                      PAYLOAD_JITTER_MIN_MS, PAYLOAD_JITTER_MAX_MS);
    }

    if (now - lastLoRaSendTime >= currentSendIntervalMs) {
        bool ok = sendSensorData();
        if (ok) {
            lastLoRaSendTime = now;
            currentSendIntervalMs = nextPayloadIntervalMs();
            Serial.printf("[TX] Sent OK | SF=%d BW=%dkHz | interval=%lums\n",
                          runtimeCfg.sf, runtimeCfg.bwKHz, currentSendIntervalMs);
        } else {
            // Retry di iterasi berikutnya (tidak reset lastLoRaSendTime)
        }
    }
    // Print route + sync stats tiap 60 detik
    static unsigned long lastPrint = 0;
    if (now - lastPrint > 60000) {
        Serial.printf("[STATUS] Route OK=%d GAGAL=%d | Sync=%s EpochLow32=%lu\n",
                      aodv.routeDiscoverySuccess, aodv.routeDiscoveryFail,
                      timeSynced?"YA":"BELUM", (unsigned long)epochOffsetMsLow32);
        Serial.printf("[CONFIG] SF=%d BW=%dkHz IntervalBase=%dms Offset=%dms Jitter=%u-%ums\n",
                      runtimeCfg.sf, runtimeCfg.bwKHz,
                      DATA_SEND_INTERVAL, NODE_ID * 500,
                      PAYLOAD_JITTER_MIN_MS, PAYLOAD_JITTER_MAX_MS);
        lastPrint = now;
    }

    // Serial command: RESET untuk hapus config NVRAM
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
    rf95.setCADTimeout(LORA_CAD_TIMEOUT_MS);

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
    sensorData.padding[0] = 'E';
    sensorData.padding[1] = 'N';
    sensorData.padding[2] = 'D';
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
    if (DATA_ACK_ENABLE && pendingDataAck) {
        return false;
    }

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

    uint32_t nextSeq = dataSequence + 1;
    LoRaPacket pkt = packetHandler.createDataPacket(
        NODE_ID, GATEWAY_ID, sensorData, nextSeq);
    pkt.header.nextHop  = aodv.getNextHop(GATEWAY_ID);
    pkt.header.checksum = LoRaPacketHandler::calculateChecksum(pkt);

    Serial.printf("[TX] Data-> SF=%d BW=%dkHz Sync=%s\n",
                  runtimeCfg.sf, runtimeCfg.bwKHz, timeSynced?"YA":"NO");
    sendPacketCallback(pkt);
    dataSequence = nextSeq;
    if (DATA_ACK_ENABLE) {
        pendingDataAck = true;
        pendingDataPacket = pkt;
        pendingDataSeq = nextSeq;
        pendingDataRetries = 0;
        pendingDataLastTxMs = millis();
    }
    return true;
}

uint32_t getDataAckTimeoutMs() {
    uint32_t baseMs;
    switch (runtimeCfg.sf) {
        case 7:  baseMs = DATA_ACK_TIMEOUT_SF7_MS; break;
        case 9:  baseMs = DATA_ACK_TIMEOUT_SF9_MS; break;
        case 12: baseMs = DATA_ACK_TIMEOUT_SF12_MS; break;
        default: baseMs = DATA_ACK_TIMEOUT_MS; break;
    }
    uint8_t hops = aodv.getRouteHopCount(GATEWAY_ID);
    if (hops == 0) hops = 1;
    return (baseMs * hops) + ((hops - 1) * 1000);
}

uint8_t getDataAckMaxRetries() {
    switch (runtimeCfg.sf) {
        case 7:  return DATA_ACK_MAX_RETRIES_SF7;
        case 9:  return DATA_ACK_MAX_RETRIES_SF9;
        case 12: return DATA_ACK_MAX_RETRIES_SF12;
        default: return DATA_ACK_MAX_RETRIES;
    }
}

void processDataAckTimeout(unsigned long nowMs) {
    if (!DATA_ACK_ENABLE || !pendingDataAck) {
        return;
    }
    const uint32_t ackTimeoutMs = getDataAckTimeoutMs();
    const uint8_t ackMaxRetries = getDataAckMaxRetries();

    if ((nowMs - pendingDataLastTxMs) < ackTimeoutMs) {
        return;
    }
    if (pendingDataRetries >= ackMaxRetries) {
        Serial.printf("[ACK] timeout seq=%lu retries=%u, drop\n",
                      (unsigned long)pendingDataSeq,
                      (unsigned)pendingDataRetries);
        pendingDataAck = false;
        
        consecutiveAckFailures++;
        if (consecutiveAckFailures >= 5) {
            Serial.println("[AODV] Link terputus (5x ACK Timeout berturut-turut)! Menghapus rute lama...");
            aodv.invalidateRoute(GATEWAY_ID);
            consecutiveAckFailures = 0;
        } else {
            Serial.printf("[AODV] Paket gagal, tapi rute dipertahankan (%d/5 kegagalan)\n", consecutiveAckFailures);
        }
        
        return;
    }

    pendingDataRetries++;
    pendingDataLastTxMs = nowMs;
    sendPacketCallback(pendingDataPacket);
    Serial.printf("[ACK] retry %u/%u seq=%lu\n",
                  (unsigned)pendingDataRetries,
                  (unsigned)ackMaxRetries,
                  (unsigned long)pendingDataSeq);
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
    const bool isForwardPayload =
        (ptype == PKT_TYPE_DATA ||
         ptype == PKT_TYPE_FATIGUE_IMU ||
         ptype == PKT_TYPE_FATIGUE_STATUS ||
         ptype == PKT_TYPE_SAFETY_CONDITION ||
         ptype == PKT_TYPE_VEHICLE_TELEMETRY ||
         ptype == PKT_TYPE_DIAGNOSTIC ||
         ptype == PKT_TYPE_ACK);

    // Duplicate detection hanya untuk paket yang memang ditujukan ke node ini.
    // Paket transit boleh lewat agar retry seq yang sama tidak ter-drop di relay.
    if (packet.header.destinationID == NODE_ID &&
        (ptype == PKT_TYPE_DATA || ptype == PKT_TYPE_FATIGUE_IMU ||
         ptype == PKT_TYPE_SAFETY_CONDITION || ptype == PKT_TYPE_VEHICLE_TELEMETRY ||
         ptype == PKT_TYPE_DIAGNOSTIC || ptype == PKT_TYPE_ACK)) {
        if (isDuplicate(packet.header.sourceID, packet.header.sequenceNum, ptype)) {
            Serial.printf("[DUP] Dropped src=%u seq=%u type=0x%02X\n",
                          packet.header.sourceID, packet.header.sequenceNum, ptype);
            return;
        }
        addToDupCache(packet.header.sourceID, packet.header.sequenceNum, ptype);
    }

    if (ptype == PKT_TYPE_ACK &&
        packet.header.destinationID == NODE_ID &&
        packet.header.payloadLength == sizeof(AckPayload)) {
        AckPayload ack = {};
        memcpy(&ack, packet.payload, sizeof(ack));
        if (DATA_ACK_ENABLE &&
            pendingDataAck &&
            ack.ackedPacketType == PKT_TYPE_DATA &&
            ack.ackedSequence == pendingDataSeq) {
            pendingDataAck = false;
            consecutiveAckFailures = 0;
            Serial.printf("[ACK] received seq=%lu retries=%u\n",
                          (unsigned long)pendingDataSeq,
                          (unsigned)pendingDataRetries);
        }
        return;
    }

    switch (ptype) {
        case PKT_TYPE_DATA:
        case PKT_TYPE_FATIGUE_IMU:
        case PKT_TYPE_FATIGUE_STATUS:
        case PKT_TYPE_SAFETY_CONDITION:
        case PKT_TYPE_VEHICLE_TELEMETRY:
        case PKT_TYPE_DIAGNOSTIC:
        case PKT_TYPE_ACK:
            if (isForwardPayload &&
                packet.header.nextHop == NODE_ID &&
                packet.header.destinationID != NODE_ID &&
                aodv.hasRouteTo(packet.header.destinationID) &&
                packet.header.hopCount < MAX_HOP_COUNT) {
                LoRaPacket fwd = packet;
                fwd.header.hopCount++;
                // Append NODE_ID ke route path agar Gateway bisa melihat jalur yang dilalui
                if (fwd.header.routePathLen < MAX_ROUTE_PATH) {
                    fwd.header.routePath[fwd.header.routePathLen] = NODE_ID;
                    fwd.header.routePathLen++;
                }
                fwd.header.nextHop  = aodv.getNextHop(packet.header.destinationID);
                fwd.header.checksum = LoRaPacketHandler::calculateChecksum(fwd);
                sendPacketCallback(fwd);
                Serial.printf("[RELAY] src=%u dst=%u type=0x%02X hop=%u via=%s\n",
                              packet.header.sourceID, packet.header.destinationID,
                              ptype, fwd.header.hopCount, NODE_NAME);
            }
            break;
        case PKT_TYPE_RREQ: aodv.handleRREQ(packet); break;
        case PKT_TYPE_RREP: aodv.handleRREP(packet); break;
        case PKT_TYPE_RERR: aodv.handleRERR(packet); break;
        case PKT_TYPE_HELLO: aodv.handleHello(packet); break;
        case PKT_TYPE_TIMESYNC:
            if (packet.header.payloadLength == sizeof(TimeSyncPayload)) {
                TimeSyncPayload ts;
                memcpy(&ts, packet.payload, sizeof(ts));
                uint32_t epochMs = (uint32_t)((uint64_t)ts.epochSeconds * 1000ULL + ts.millisPart);
                
                static uint32_t lastSyncEpoch = 0;
                // Flood rebroadcast mechanism: hanya forward jika sync baru
                if (epochMs > lastSyncEpoch + 1000) {
                    lastSyncEpoch = epochMs;
                    epochOffsetMsLow32 = epochMs - (uint32_t)millis();
                    timeSynced = true;
                    Serial.printf("[TIMESYNC] Synced! EpochLow32=%lu\n", (unsigned long)epochOffsetMsLow32);
                    
                    // Rebroadcast agar node multi-hop juga mendapat sinkronisasi jam
                    delay(random(20, 100)); // Jeda anti-kolisi
                    sendPacketCallback(packet);
                }
            }
            break;
        case PKT_TYPE_START_TEST:
            if (packet.header.payloadLength == sizeof(StartTestPayload)) {
                StartTestPayload st;
                memcpy(&st, packet.payload, sizeof(StartTestPayload));
                Serial.printf("[START_TEST] SF=%u BW=%lukHz\n", st.sf, (unsigned long)st.bwKHz);
                WebConfig::saveTestConfig(st.sf, st.bwKHz);
                delay(2000);
                ESP.restart();
            }
            break;
    }
}

void sendPacketCallback(const LoRaPacket& packet) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    int len = packetHandler.serializePacket(packet, buf, sizeof(buf));
    if (packet.header.packetType == PKT_TYPE_ACK) {
        delay(random(5, 20));
    } else {
        delay(random(20, 80));
    }
    if (len > 0) { rf95.send(buf, len); rf95.waitPacketSent(); rf95.setModeRx(); }
}


