#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#include "config.h"
#include "LoRa_Packet.h"
#include "AODV_Routing.h"
#include "WebConfig.h"

LoRaRuntimeCfg runtimeCfg;
RH_RF95 rf95(LORA_CS_PIN, LORA_DIO0_PIN);
AODVRouting aodv(NODE_ID);
LoRaPacketHandler packetHandler;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

double lat = 0.0;
double lon = 0.0;
bool hasInitialGpsFix = false;

float speedMps = 0.0f;
float headingDeg = 180.0f;
bool headingOffsetSet = false;
float headingOffsetDeg = 0.0f;
unsigned long lastMillis = 0;

unsigned long overspeedStartTime = 0;
unsigned long brakingStartTime = 0;

float pitchDeg = 0.0f;
float rollDeg = 0.0f;
float rawYawDeg = 0.0f;
bool drActive = true;

float lastAccelForward = 0.0f;
float lastAx = 0.0f;
float lastAy = 0.0f;
float lastAz = 0.0f;
float lastMx = 0.0f;
float lastMy = 0.0f;
float lastMz = 0.0f;
float lastDt = 0.0f;
float lastGpsSpeed = 0.0f;
float lastGpsHeading = 0.0f;
float lastHdop = 99.9f;
uint16_t lastSatellites = 0;
bool lastGpsValid = false;
uint8_t calSys = 0;
uint8_t calGyro = 0;
uint8_t calAccel = 0;
uint8_t calMag = 0;
uint8_t isRollover = 0;
uint8_t isRolloverRisk = 0;
uint8_t isHarshBraking = 0;
uint8_t isOverspeed = 0;

uint32_t epochOffsetMsLow32 = 0;
bool timeSynced = false;
unsigned long lastPublishMs = 0;
unsigned long lastStatusPrintMs = 0;
uint32_t vehicleSequence = 0;

static bool pendingDataAck = false;
static LoRaPacket pendingDataPacket;
static uint32_t pendingDataSeq = 0;
static uint8_t pendingDataRetries = 0;
static uint8_t consecutiveAckFailures = 0;
static unsigned long pendingDataLastTxMs = 0;

void initLoRa();
bool initBNO055();
void initGPS();
void updateTelemetry(unsigned long nowMs);
bool sendVehicleTelemetry(unsigned long nowMs);
uint32_t getDataAckTimeoutMs();
uint8_t getDataAckMaxRetries();
void processDataAckTimeout(unsigned long nowMs);
void receivePackets();
void handleReceivedPacket(const LoRaPacket& packet);
void sendPacketCallback(const LoRaPacket& packet);
bool forwardPacket(const LoRaPacket& packet);

static inline float clampf(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static inline float wrap360(float degrees) {
    while (degrees < 0.0f) degrees += 360.0f;
    while (degrees >= 360.0f) degrees -= 360.0f;
    return degrees;
}

void headingToUnitENU(float heading, float &xEast, float &yNorth) {
    float rad = heading * DEG_TO_RAD;
    xEast = sinf(rad);
    yNorth = cosf(rad);
}

void moveLatLon(double &latDeg, double &lonDeg, float dEastM, float dNorthM) {
    const double earthRadius = 6378137.0;
    double latRad = latDeg * DEG_TO_RAD;

    double dLat = dNorthM / earthRadius;
    double dLon = dEastM / (earthRadius * cos(latRad));

    latDeg += dLat * RAD_TO_DEG;
    lonDeg += dLon * RAD_TO_DEG;
}

float readHeadingDegFromMag() {
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

    rawYawDeg = event.orientation.x;
    pitchDeg = event.orientation.z;
    rollDeg = event.orientation.y;

    float currentHeading = rawYawDeg;
    if (!headingOffsetSet) {
        headingOffsetDeg = 180.0f - currentHeading;
        headingOffsetSet = true;
    }

    return wrap360(currentHeading + headingOffsetDeg);
}

void readLinearAccel(float &ax, float &ay, float &az) {
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
    ax = event.acceleration.x;
    ay = event.acceleration.y;
    az = event.acceleration.z;
}

float projectAccelToHeading(float heading, float ax, float ay) {
    float hx;
    float hy;
    headingToUnitENU(heading, hx, hy);
    float accelForward = ax * hx + ay * hy;
    if (fabs(accelForward) < ACCEL_DEADBAND) {
        accelForward = 0.0f;
    }
    return clampf(accelForward, -MAX_ACCEL_USED, MAX_ACCEL_USED);
}

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
    randomSeed((uint32_t)micros() ^ ((uint32_t)NODE_ID << 16));

    Serial.println();
    Serial.println("========================================");
    Serial.printf("  LoRa Vehicle Node %s (ID=%d)\n", NODE_NAME, NODE_ID);
    Serial.println("========================================");

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    runtimeCfg = WebConfig::getConfig(
        LORA_SPREADING_FACTOR,
        LORA_BANDWIDTH,
        LORA_CODING_RATE,
        LORA_TX_POWER,
        LORA_USE_RFO
    );

    Serial.printf("Runtime: SF=%d BW=%lukHz CR=4/%d Pwr=%ddBm\n",
                  runtimeCfg.sf,
                  (unsigned long)runtimeCfg.bwKHz,
                  runtimeCfg.cr,
                  runtimeCfg.txPower);
    Serial.printf("[PAYLOAD] VehicleTelemetryPayload=%u bytes (LoRa total=%u bytes)\n",
                  (unsigned)sizeof(VehicleTelemetryPayload),
                  (unsigned)(sizeof(PacketHeader) + sizeof(VehicleTelemetryPayload)));

    WebConfig::begin(NODE_NAME, runtimeCfg, NODE_ID);
    initLoRa();
    initGPS();

    if (!initBNO055()) {
        Serial.println("[IMU] BNO055 not detected. Node halted.");
        while (true) {
            digitalWrite(BUZZER_PIN, LOW);
            delay(2000);
        }
    }

    aodv.begin();
    aodv.onSendPacket = sendPacketCallback;
    aodv.onDiagnosticReady = sendDiagnosticCallback;
    aodv.epochOffsetPtr = &epochOffsetMsLow32;

    lastMillis = millis();
    Serial.println("[NODE] Setup complete");
    Serial.println("Ketik RESET di Serial untuk reset config NVRAM.");
}

// ================================================================
// MAIN LOOP Ã¢â‚¬â€ Interval + Payload Jitter Acak + Offset Per Node
// ================================================================
unsigned long lastSendTime = 0;
unsigned long currentSendIntervalMs = DATA_SEND_INTERVAL;
bool initialOffsetDone = false;
static const uint16_t PAYLOAD_JITTER_MIN_MS = 100;
static const uint16_t PAYLOAD_JITTER_MAX_MS = 500;

static unsigned long nextPayloadIntervalMs() {
    return DATA_SEND_INTERVAL + (unsigned long)random(PAYLOAD_JITTER_MIN_MS, PAYLOAD_JITTER_MAX_MS + 1);
}

void loop() {
    unsigned long nowMs = millis();

    WebConfig::handle();
    aodv.update();
    receivePackets();
    processDataAckTimeout(nowMs);
    updateTelemetry(nowMs);

    // FIXED BASE INTERVAL + RANDOM JITTER (+ offset NODE_ID * 500ms)
    if (!initialOffsetDone) {
        currentSendIntervalMs = nextPayloadIntervalMs();
        lastSendTime = nowMs - currentSendIntervalMs + (NODE_ID * 500UL);
        initialOffsetDone = true;
        Serial.printf("[TX] Payload jitter aktif: +%u..+%u ms\n",
                      PAYLOAD_JITTER_MIN_MS, PAYLOAD_JITTER_MAX_MS);
    }

    if (nowMs - lastSendTime >= currentSendIntervalMs) {
        bool ok = sendVehicleTelemetry(nowMs);
        if (ok) {
            lastSendTime = nowMs;
            currentSendIntervalMs = nextPayloadIntervalMs();
            Serial.printf("[TX] Sent OK | SF=%d BW=%dkHz | interval=%lums\n",
                          runtimeCfg.sf, runtimeCfg.bwKHz, currentSendIntervalMs);
        }
    }

    if (nowMs - lastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS) {
        Serial.printf("[STATUS] Route OK=%u FAIL=%u | Sync=%s EpochLow32=%lu | DR=%s | GPS=%s | Speed=%.2f\n",
                      aodv.routeDiscoverySuccess,
                      aodv.routeDiscoveryFail,
                      timeSynced ? "YES" : "NO",
                      (unsigned long)epochOffsetMsLow32,
                      drActive ? "ON" : "OFF",
                      lastGpsValid ? "VALID" : "INVALID",
                      speedMps);
        Serial.printf("[CONFIG] SF=%d BW=%dkHz IntervalBase=%dms Offset=%dms Jitter=%u-%ums\n",
                      runtimeCfg.sf, runtimeCfg.bwKHz,
                      DATA_SEND_INTERVAL, NODE_ID * 500,
                      PAYLOAD_JITTER_MIN_MS, PAYLOAD_JITTER_MAX_MS);
        lastStatusPrintMs = nowMs;
    }

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();

        if (cmd == "RESET") {
            WebConfig::resetConfig();
            Serial.println("[CLI] NVRAM reset. Reboot to use defaults.");
        }
    }

    delay(10);
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

    Serial.printf("[LoRa] OK SF=%d BW=%.0fkHz CR=4/%d Pwr=%ddBm (%s)\n",
                  runtimeCfg.sf,
                  bwHz / 1000.0f,
                  runtimeCfg.cr,
                  pwr,
                  runtimeCfg.useRFO ? "RFO" : "PA_BOOST");
}

bool initBNO055() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (!bno.begin()) {
        return false;
    }

    bno.setExtCrystalUse(true);
    return true;
}

void initGPS() {
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.printf("[GPS] UART started RX=%d TX=%d baud=%lu\n",
                  GPS_RX_PIN,
                  GPS_TX_PIN,
                  (unsigned long)GPS_BAUD);
}

void updateTelemetry(unsigned long nowMs) {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    float dt = (nowMs - lastMillis) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) {
        lastMillis = nowMs;
        return;
    }
    lastMillis = nowMs;
    lastDt = dt;

    headingDeg = readHeadingDegFromMag();

    float linAx;
    float linAy;
    float linAz;
    readLinearAccel(linAx, linAy, linAz);
    lastAx = linAx;
    lastAy = linAy;
    lastAz = linAz;

    float accelForward = projectAccelToHeading(headingDeg, linAx, linAy);
    lastAccelForward = accelForward;

    bool isGpsValid = gps.location.isValid();
    bool isGpsUpdated = gps.location.isUpdated();
    float hdopValue = gps.hdop.isValid() ? gps.hdop.value() / 100.0f : 99.9f;
    uint16_t satsValue = gps.satellites.isValid() ? gps.satellites.value() : 0;
    float gpsSpeedOut = gps.speed.isValid() ? gps.speed.mps() : 0.0f;
    float gpsHeadingOut = gps.course.isValid() ? gps.course.deg() : 0.0f;

    bool isGpsGood = isGpsValid && (hdopValue <= HDOP_THRESHOLD);

    if (isGpsGood) {
        drActive = false;
        if (gps.location.isValid()) {
            lat = gps.location.lat();
            lon = gps.location.lng();
            hasInitialGpsFix = true;
        }
        if (isGpsUpdated) {
            speedMps = gpsSpeedOut;
        }
    } else if (hasInitialGpsFix) {
        drActive = true;

        if (accelForward == 0.0f) {
            if (speedMps > 0.0f) {
                speedMps -= FRICTION_DECAY * dt;
                if (speedMps < 0.0f) speedMps = 0.0f;
            } else if (speedMps < 0.0f) {
                speedMps += FRICTION_DECAY * dt;
                if (speedMps > 0.0f) speedMps = 0.0f;
            }
            if (fabs(speedMps) < 0.1f) {
                speedMps = 0.0f;
            }
        } else {
            speedMps += accelForward * dt;
        }

        if (!ALLOW_REVERSE && speedMps < 0.0f) {
            speedMps = 0.0f;
        }
        if (speedMps > MAX_SPEED_MPS) {
            speedMps = MAX_SPEED_MPS;
        }
        if (speedMps < -MAX_SPEED_MPS) {
            speedMps = -MAX_SPEED_MPS;
        }

        float distanceM = speedMps * dt;
        float hx;
        float hy;
        headingToUnitENU(headingDeg, hx, hy);
        moveLatLon(lat, lon, distanceM * hx, distanceM * hy);
    }

    sensors_event_t magEvent;
    bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    lastMx = magEvent.magnetic.x;
    lastMy = magEvent.magnetic.y;
    lastMz = magEvent.magnetic.z;
    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

    float absRoll = fabs(rollDeg);
    float absPitch = fabs(pitchDeg);
    isRollover = (absRoll >= 46.0f || absPitch >= 46.0f) ? 1 : 0;
    isRolloverRisk = (!isRollover && (absRoll >= 35.0f || absPitch >= 35.0f)) ? 1 : 0;

    digitalWrite(BUZZER_PIN, (isRolloverRisk || isRollover) ? HIGH : LOW);

    float hxEval;
    float hyEval;
    headingToUnitENU(headingDeg, hxEval, hyEval);
    float rawAccelForward = linAx * hxEval + linAy * hyEval;

    isHarshBraking = 0;
    if (rawAccelForward <= -3.0f) {
        if (brakingStartTime == 0) {
            brakingStartTime = nowMs;
        } else if ((nowMs - brakingStartTime) >= 3000) {
            isHarshBraking = 1;
        }
    } else {
        brakingStartTime = 0;
    }

    isOverspeed = 0;
    if (speedMps > SPEED_LIMIT_MPS) {
        if (overspeedStartTime == 0) {
            overspeedStartTime = nowMs;
        } else if ((nowMs - overspeedStartTime) >= 3000) {
            isOverspeed = 1;
        }
    } else {
        overspeedStartTime = 0;
    }

    lastGpsValid = isGpsValid;
    lastGpsSpeed = gpsSpeedOut;
    lastGpsHeading = gpsHeadingOut;
    lastHdop = hdopValue;
    lastSatellites = satsValue;
}

bool sendVehicleTelemetry(unsigned long nowMs) {
    if (DATA_ACK_ENABLE && pendingDataAck) {
        return false;
    }

    if (!aodv.hasRouteTo(GATEWAY_ID)) {
        Serial.println("[LoRa] No route to gateway. Starting discovery.");
        aodv.initiateRouteDiscovery(GATEWAY_ID);
        return false;
    }

    VehicleTelemetryPayload payload = {};
    payload.latitude = lat;
    payload.longitude = lon;
    payload.headingDeg = headingDeg;
    payload.speedMps = speedMps;
    payload.drActive = drActive ? 1 : 0;
    payload.gpsValid = lastGpsValid ? 1 : 0;
    payload.gpsSpeed = lastGpsSpeed;
    payload.gpsHeading = lastGpsHeading;
    payload.hdop = lastHdop;
    payload.satellites = lastSatellites;
    payload.accelForward = lastAccelForward;
    payload.ax = lastAx;
    payload.ay = lastAy;
    payload.az = lastAz;
    payload.mx = lastMx;
    payload.my = lastMy;
    payload.mz = lastMz;
    payload.pitch = pitchDeg;
    payload.yaw = rawYawDeg;
    payload.roll = rollDeg;
    payload.dt = lastDt;
    payload.calSys = calSys;
    payload.calGyro = calGyro;
    payload.calAccel = calAccel;
    payload.calMag = calMag;
    payload.rollover = isRollover;
    payload.rolloverRisk = isRolloverRisk;
    payload.harshBraking = isHarshBraking;
    payload.overspeed = isOverspeed;
    payload.txTimestamp = timeSynced
        ? ((uint32_t)millis() + epochOffsetMsLow32)
        : (uint32_t)nowMs;
    payload.routeDiscMs = 0;
    payload.routeHops = 0;

    uint32_t lastDiscoveryMs = 0;
    uint8_t lastDiscoveryHops = 0;
    if (aodv.getLastSuccessfulDiscovery(GATEWAY_ID, lastDiscoveryMs, lastDiscoveryHops)) {
        payload.routeDiscMs = lastDiscoveryMs;
    }
    payload.routeHops = aodv.getRouteHopCount(GATEWAY_ID);
    if (payload.routeHops == 0) {
        payload.routeHops = lastDiscoveryHops;
    }

    uint32_t nextSeq = vehicleSequence + 1;
    LoRaPacket packet = LoRaPacketHandler::createVehicleTelemetryPacket(
        NODE_ID,
        GATEWAY_ID,
        payload,
        nextSeq
    );
    packet.header.nextHop = aodv.getNextHop(GATEWAY_ID);
    packet.header.checksum = LoRaPacketHandler::calculateChecksum(packet);

    Serial.printf(
        "[lora_nailah TX VEHICLE120] ts=%lu lat=%.6f lon=%.6f spd=%.2f heading=%.2f gpsValid=%u dr=%u roll=%u risk=%u brake=%u over=%u route_ms=%lu route_hops=%u\n",
        (unsigned long)payload.txTimestamp,
        payload.latitude,
        payload.longitude,
        payload.speedMps,
        payload.headingDeg,
        payload.gpsValid,
        payload.drActive,
        payload.rollover,
        payload.rolloverRisk,
        payload.harshBraking,
        payload.overspeed,
        (unsigned long)payload.routeDiscMs,
        payload.routeHops
    );

    sendPacketCallback(packet);
    vehicleSequence = nextSeq;
    if (DATA_ACK_ENABLE) {
        pendingDataAck = true;
        pendingDataPacket = packet;
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
    return (baseMs * hops) + ((hops - 1) * 500);
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
            Serial.println("[AODV] Link terputus (3x ACK Timeout berturut-turut)! Menghapus rute lama...");
            aodv.invalidateRoute(GATEWAY_ID);
            consecutiveAckFailures = 0;
        } else {
            Serial.printf("[AODV] Paket gagal, tapi rute dipertahankan (%d/3 kegagalan)\n", consecutiveAckFailures);
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

void receivePackets() {
    if (!rf95.available()) {
        return;
    }

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
        LoRaPacket packet;
        if (packetHandler.deserializePacket(buf, len, packet)) {
            handleReceivedPacket(packet);
        }
    }
}

void handleReceivedPacket(const LoRaPacket& packet) {
    if (packet.header.sourceID == NODE_ID) {
        return;
    }

    uint8_t ptype = packet.header.packetType;
    const bool isForwardPayload =
        (ptype == PKT_TYPE_DATA ||
         ptype == PKT_TYPE_FATIGUE_IMU ||
         ptype == PKT_TYPE_FATIGUE_STATUS ||
         ptype == PKT_TYPE_SAFETY_CONDITION ||
         ptype == PKT_TYPE_VEHICLE_TELEMETRY ||
         ptype == PKT_TYPE_DIAGNOSTIC ||
         ptype == PKT_TYPE_ACK);

    // Duplicate detection hanya untuk paket tujuan node ini.
    // Paket transit dibiarkan lewat agar retry seq yang sama tidak ter-drop.
    static struct { uint8_t src; uint32_t seq; uint8_t type; unsigned long ts; bool valid; } _dupCache[32];
    static uint8_t _dupIdx = 0;
    if (packet.header.destinationID == NODE_ID &&
        (ptype == PKT_TYPE_DATA || ptype == PKT_TYPE_FATIGUE_IMU ||
         ptype == PKT_TYPE_SAFETY_CONDITION || ptype == PKT_TYPE_VEHICLE_TELEMETRY ||
         ptype == PKT_TYPE_DIAGNOSTIC || ptype == PKT_TYPE_ACK)) {
        unsigned long now = millis();
        for (int i = 0; i < 32; i++) {
            if (_dupCache[i].valid && _dupCache[i].src == packet.header.sourceID &&
                _dupCache[i].seq == packet.header.sequenceNum &&
                _dupCache[i].type == ptype && (now - _dupCache[i].ts) < 10000) return;
        }
        _dupCache[_dupIdx] = {packet.header.sourceID, packet.header.sequenceNum, ptype, now, true};
        _dupIdx = (_dupIdx + 1) % 32;
    }

    if (ptype == PKT_TYPE_ACK &&
        packet.header.destinationID == NODE_ID &&
        packet.header.payloadLength == sizeof(AckPayload)) {
        AckPayload ack = {};
        memcpy(&ack, packet.payload, sizeof(ack));
        if (DATA_ACK_ENABLE &&
            pendingDataAck &&
            ack.ackedPacketType == PKT_TYPE_VEHICLE_TELEMETRY &&
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
                packet.header.destinationID != NODE_ID) {
                forwardPacket(packet);
            }
            break;

        case PKT_TYPE_RREQ:  aodv.handleRREQ(packet); break;
        case PKT_TYPE_RREP:  aodv.handleRREP(packet); break;
        case PKT_TYPE_RERR:  aodv.handleRERR(packet); break;
        case PKT_TYPE_HELLO: aodv.handleHello(packet); break;

        case PKT_TYPE_TIMESYNC:
            if (packet.header.payloadLength == sizeof(TimeSyncPayload)) {
                TimeSyncPayload ts;
                memcpy(&ts, packet.payload, sizeof(ts));
                uint32_t epochMs = (uint32_t)((uint64_t)ts.epochSeconds * 1000ULL + ts.millisPart);
                epochOffsetMsLow32 = epochMs - (uint32_t)millis();
                timeSynced = true;
                Serial.printf("[TIMESYNC] EpochLow32=%lu\n",
                              (unsigned long)epochOffsetMsLow32);
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

        default:
            break;
    }
}

bool forwardPacket(const LoRaPacket& packet) {
    if (!aodv.hasRouteTo(packet.header.destinationID) ||
        packet.header.hopCount >= MAX_HOP_COUNT) {
        return false;
    }

    LoRaPacket forwarded = packet;
    forwarded.header.hopCount++;
    if (forwarded.header.routePathLen < MAX_ROUTE_PATH) {
        forwarded.header.routePath[forwarded.header.routePathLen] = NODE_ID;
        forwarded.header.routePathLen++;
    }
    forwarded.header.nextHop = aodv.getNextHop(packet.header.destinationID);
    forwarded.header.checksum = LoRaPacketHandler::calculateChecksum(forwarded);
    sendPacketCallback(forwarded);
    return true;
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
        rf95.setModeRx();
    }
}

