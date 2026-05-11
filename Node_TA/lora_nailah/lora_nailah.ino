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
uint32_t safetySequence = 0;

void initLoRa();
bool initBNO055();
void initGPS();
void updateTelemetry(unsigned long nowMs);
bool sendSafetyCondition(unsigned long nowMs);
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

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);

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

    WebConfig::begin(NODE_NAME, runtimeCfg);
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

    lastMillis = millis();
    Serial.println("[NODE] Setup complete");
    Serial.println("Ketik RESET di Serial untuk reset config NVRAM.");
}

void loop() {
    unsigned long nowMs = millis();

    WebConfig::handle();
    aodv.update();
    receivePackets();
    updateTelemetry(nowMs);

    if (nowMs - lastPublishMs >= (unsigned long)(DATA_SEND_INTERVAL + random(-1000, 1000))) {
        lastPublishMs = nowMs;
        sendSafetyCondition(nowMs);
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

bool sendSafetyCondition(unsigned long nowMs) {
    if (!aodv.hasRouteTo(GATEWAY_ID)) {
        Serial.println("[LoRa] No route to gateway. Starting discovery.");
        aodv.initiateRouteDiscovery(GATEWAY_ID);
        return false;
    }

    SafetyConditionPayload payload = {};
    payload.packetType = PKT_TYPE_SAFETY_CONDITION;
    payload.nodeId = NODE_ID;
    payload.ts = timeSynced
        ? ((uint32_t)millis() + epochOffsetMsLow32)
        : (uint32_t)nowMs;
    payload.flags = 0;

    if (lastGpsValid) {
        payload.flags |= FLAG_GPS_VALID;
    }
    if (drActive) {
        payload.flags |= FLAG_DR_ACTIVE;
    }
    if (isRolloverRisk) {
        payload.flags |= FLAG_ROLLOVER_RISK;
    }
    if (isRollover) {
        payload.flags |= FLAG_ROLLOVER;
    }
    if (isHarshBraking) {
        payload.flags |= FLAG_HARSH_BRAKE;
    }
    if (isOverspeed) {
        payload.flags |= FLAG_OVERSPEED;
    }

    LoRaPacket packet = LoRaPacketHandler::createSafetyConditionPacket(
        NODE_ID,
        GATEWAY_ID,
        payload,
        ++safetySequence
    );
    packet.header.nextHop = aodv.getNextHop(GATEWAY_ID);
    packet.header.checksum = LoRaPacketHandler::calculateChecksum(packet);

    Serial.printf(
        "[lora_nailah TX SAFETY flags] ts=%lu flags=0x%02X gpsValid=%u drActive=%u rolloverRisk=%u rollover=%u harshBraking=%u overspeed=%u\n",
        (unsigned long)payload.ts,
        payload.flags,
        lastGpsValid ? 1 : 0,
        drActive ? 1 : 0,
        isRolloverRisk ? 1 : 0,
        isRollover ? 1 : 0,
        isHarshBraking ? 1 : 0,
        isOverspeed ? 1 : 0
    );

    sendPacketCallback(packet);
    return true;
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

    switch (packet.header.packetType) {
        case PKT_TYPE_DATA:
        case PKT_TYPE_FATIGUE_IMU:
        case PKT_TYPE_FATIGUE_STATUS:
        case PKT_TYPE_SAFETY_CONDITION:
        case PKT_TYPE_VEHICLE_TELEMETRY:
            if (packet.header.nextHop == NODE_ID &&
                packet.header.destinationID != NODE_ID) {
                forwardPacket(packet);
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
    forwarded.header.nextHop = aodv.getNextHop(packet.header.destinationID);
    forwarded.header.checksum = LoRaPacketHandler::calculateChecksum(forwarded);
    sendPacketCallback(forwarded);
    return true;
}

void sendPacketCallback(const LoRaPacket& packet) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    int len = packetHandler.serializePacket(packet, buf, sizeof(buf));

    delay(random(30, 150));
    if (len > 0) {
        rf95.send(buf, len);
        rf95.waitPacketSent();
        rf95.setModeRx();
    }
}
