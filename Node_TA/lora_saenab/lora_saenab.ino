#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "config.h"
#include "LoRa_Packet.h"
#include "AODV_Routing.h"
#include "WebConfig.h"

enum AlarmState : uint8_t {
    ALARM_NORMAL = 0,
    ALARM_LELAH = 1,
    ALARM_TERTIDUR = 2
};

LoRaRuntimeCfg runtimeCfg;
RH_RF95 rf95(LORA_CS_PIN, LORA_DIO0_PIN);
AODVRouting aodv(NODE_ID);
LoRaPacketHandler packetHandler;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

AlarmState alarmState = ALARM_NORMAL;
unsigned long patternLastMs = 0;
uint8_t patternStep = 0;
unsigned long lastPublishMs = 0;
unsigned long lastStatusPrintMs = 0;
uint32_t imuSequenceNumber = 0;

uint32_t epochOffsetMsLow32 = 0;
bool timeSynced = false;

bool gyroInit = false;
float pitch = 0.0f;
float roll = 0.0f;
float f_gx = 0.0f;
float f_gy = 0.0f;
float f_gz = 0.0f;
uint8_t cal_sys = 0;
uint8_t cal_gyro = 0;
uint8_t cal_acc = 0;
uint8_t cal_mag = 0;

void initLoRa();
bool initBNO055();
void readBNO055AndFilter();
bool sendImuData(unsigned long nowMs);
void receivePackets();
void handleReceivedPacket(const LoRaPacket& packet);
void sendPacketCallback(const LoRaPacket& packet);
bool forwardPacket(const LoRaPacket& packet);
void applyAlarmStatusPayload(const FatigueStatusPayload& payload);
const char* alarmText(AlarmState state);

static inline int16_t toScaledInt16(float value, float scale) {
    long scaled = lroundf(value * scale);
    if (scaled > 32767L) {
        scaled = 32767L;
    } else if (scaled < -32768L) {
        scaled = -32768L;
    }
    return (int16_t)scaled;
}

static inline void buzzerWrite(bool on) {
#if BUZZER_ACTIVE_HIGH
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
#else
    digitalWrite(BUZZER_PIN, on ? LOW : HIGH);
#endif
}

// Tambahkan sinyal buzzer untuk debugging tanpa serial monitor
void beepSignal(int count) {
    for (int i = 0; i < count; i++) {
        buzzerWrite(true);
        delay(150);
        buzzerWrite(false);
        delay(100);
    }
}

void resetPattern() {
    patternStep = 0;
    patternLastMs = millis();
    buzzerWrite(false);
}

void setAlarmState(AlarmState nextState) {
    if (nextState == alarmState) {
        return;
    }

    alarmState = nextState;
    resetPattern();
    Serial.printf("[ALARM] State -> %s\n", alarmText(alarmState));
}

void patternLelah(unsigned long nowMs) {
    const uint16_t onMs = 120;
    const uint16_t offMs = 120;
    const uint16_t gapMs = 600;

    switch (patternStep) {
        case 0:
            buzzerWrite(true);
            if (nowMs - patternLastMs >= onMs) {
                patternLastMs = nowMs;
                patternStep = 1;
            }
            break;
        case 1:
            buzzerWrite(false);
            if (nowMs - patternLastMs >= offMs) {
                patternLastMs = nowMs;
                patternStep = 2;
            }
            break;
        case 2:
            buzzerWrite(true);
            if (nowMs - patternLastMs >= onMs) {
                patternLastMs = nowMs;
                patternStep = 3;
            }
            break;
        case 3:
            buzzerWrite(false);
            if (nowMs - patternLastMs >= offMs) {
                patternLastMs = nowMs;
                patternStep = 4;
            }
            break;
        case 4:
            buzzerWrite(true);
            if (nowMs - patternLastMs >= onMs) {
                patternLastMs = nowMs;
                patternStep = 5;
            }
            break;
        default:
            buzzerWrite(false);
            if (nowMs - patternLastMs >= gapMs) {
                patternLastMs = nowMs;
                patternStep = 0;
            }
            break;
    }
}

void patternTertidur(unsigned long nowMs) {
    const uint16_t onMs = 2000;
    const uint16_t offMs = 1000;

    switch (patternStep) {
        case 0:
            buzzerWrite(true);
            if (nowMs - patternLastMs >= onMs) {
                patternLastMs = nowMs;
                patternStep = 1;
            }
            break;
        default:
            buzzerWrite(false);
            if (nowMs - patternLastMs >= offMs) {
                patternLastMs = nowMs;
                patternStep = 0;
            }
            break;
    }
}

const char* alarmText(AlarmState state) {
    switch (state) {
        case ALARM_LELAH:
            return "LELAH";
        case ALARM_TERTIDUR:
            return "TERTIDUR";
        case ALARM_NORMAL:
        default:
            return "NORMAL";
    }
}

void setup() {
    // 1. Inisialisasi Buzzer segera agar bisa memberi sinyal
    pinMode(BUZZER_PIN, OUTPUT);
    resetPattern();
    beepSignal(1); // Bunyi 1x: Board menyala

    // 2. Inisialisasi Serial (Tunggu sebentar untuk USB-CDC pada ESP32-C3)
    Serial.begin(SERIAL_BAUD);
    unsigned long startSerial = millis();
    while (!Serial && millis() - startSerial < 3000); // Tunggu max 3 detik

    delay(3000);
    Serial.println("\n[SYSTEM] Booting Fatigue Node...");
    Serial.println("========================================");
    Serial.printf("  Node: %s (ID=%d)\n", NODE_NAME, NODE_ID);
    Serial.println("========================================");

    // 3. Muat Konfigurasi & Jalankan WiFi Config AP (Dahulukan agar SSID muncul)
    runtimeCfg = WebConfig::getConfig(
        LORA_SPREADING_FACTOR,
        LORA_BANDWIDTH,
        LORA_CODING_RATE,
        LORA_TX_POWER,
        LORA_USE_RFO
    );
    
    WebConfig::begin(NODE_NAME, runtimeCfg);
    beepSignal(2); // Bunyi 2x: WiFi SSID Siap

    // 4. Inisialisasi LoRa
    initLoRa();

    // 5. Inisialisasi AODV
    aodv.begin();
    aodv.onSendPacket = sendPacketCallback;

    // 6. Inisialisasi BNO055
    if (!initBNO055()) {
        Serial.println("[IMU] ERROR: BNO055 init failed. Node halted.");
        while (true) {
            beepSignal(1); // Beep pendek terus-menerus = ERROR HARDWARE
            delay(1000);
        }
    }

    Serial.println("[IMU] BNO055 ready");
    Serial.println("[NODE] Setup complete. Ready to work.");
    Serial.println("Ketik RESET di Serial untuk reset config NVRAM.");
    beepSignal(3); // Bunyi 3x: Semua sistem OK
}

void loop() {
    unsigned long nowMs = millis();

    WebConfig::handle();
    aodv.update();
    receivePackets();

    if (alarmState == ALARM_LELAH) {
        patternLelah(nowMs);
    } else if (alarmState == ALARM_TERTIDUR) {
        patternTertidur(nowMs);
    } else {
        buzzerWrite(false);
    }

    if (nowMs - lastPublishMs >= (unsigned long)(IMU_PUBLISH_INTERVAL_MS + random(-1000, 1000))) {
        lastPublishMs = nowMs;
        sendImuData(nowMs);
    }

    if (nowMs - lastStatusPrintMs >= STATUS_PRINT_INTERVAL_MS) {
        Serial.printf("[STATUS] Route OK=%u FAIL=%u | Sync=%s EpochLow32=%lu | Alarm=%s\n",
                      aodv.routeDiscoverySuccess,
                      aodv.routeDiscoveryFail,
                      timeSynced ? "YES" : "NO",
                      (unsigned long)epochOffsetMsLow32,
                      alarmText(alarmState));
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

    delay(5);
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

    const float bwHz = runtimeCfg.bwKHz * 1000.0f;
    const int8_t pwr = runtimeCfg.useRFO
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

void readBNO055AndFilter() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    pitch = euler.y();
    roll = euler.z();

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float gx = gyro.x();
    float gy = gyro.y();
    float gz = gyro.z();

    if (!gyroInit) {
        f_gx = gx;
        f_gy = gy;
        f_gz = gz;
        gyroInit = true;
    } else {
        f_gx = GYRO_EMA_ALPHA * gx + (1.0f - GYRO_EMA_ALPHA) * f_gx;
        f_gy = GYRO_EMA_ALPHA * gy + (1.0f - GYRO_EMA_ALPHA) * f_gy;
        f_gz = GYRO_EMA_ALPHA * gz + (1.0f - GYRO_EMA_ALPHA) * f_gz;
    }

    bno.getCalibration(&cal_sys, &cal_gyro, &cal_acc, &cal_mag);
}

bool sendImuData(unsigned long nowMs) {
    readBNO055AndFilter();

    if (!aodv.hasRouteTo(GATEWAY_ID)) {
        Serial.println("[LoRa] No route to gateway. Starting discovery.");
        aodv.initiateRouteDiscovery(GATEWAY_ID);
        return false;
    }

    ImuFatiguePayload payload = {};
    payload.packetType = PKT_TYPE_FATIGUE_IMU;
    payload.nodeId = NODE_ID;
    payload.ts = timeSynced
        ? ((uint32_t)millis() + epochOffsetMsLow32)
        : (uint32_t)nowMs;
    payload.pitch100 = toScaledInt16(pitch, 100.0f);
    payload.roll100 = toScaledInt16(roll, 100.0f);
    payload.gx1000 = toScaledInt16(f_gx, 1000.0f);
    payload.gy1000 = toScaledInt16(f_gy, 1000.0f);
    payload.gz1000 = toScaledInt16(f_gz, 1000.0f);

    LoRaPacket packet = LoRaPacketHandler::createImuFatiguePacket(
        NODE_ID,
        GATEWAY_ID,
        payload,
        ++imuSequenceNumber
    );
    packet.header.nextHop = aodv.getNextHop(GATEWAY_ID);
    packet.header.checksum = LoRaPacketHandler::calculateChecksum(packet);

    Serial.printf("[lora_saenab TX IMU] ts=%lu pitch=%.2f roll=%.2f f_gx=%.3f f_gy=%.3f f_gz=%.3f\n",
                  (unsigned long)payload.ts,
                  payload.pitch100 / 100.0f,
                  payload.roll100 / 100.0f,
                  payload.gx1000 / 1000.0f,
                  payload.gy1000 / 1000.0f,
                  payload.gz1000 / 1000.0f);

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
        case PKT_TYPE_SAFETY_CONDITION:
        case PKT_TYPE_VEHICLE_TELEMETRY:
            if (packet.header.nextHop == NODE_ID &&
                packet.header.destinationID != NODE_ID) {
                forwardPacket(packet);
            }
            break;

        case PKT_TYPE_FATIGUE_STATUS:
            if (packet.header.destinationID == NODE_ID &&
                packet.header.payloadLength == sizeof(FatigueStatusPayload)) {
                FatigueStatusPayload payload;
                memcpy(&payload, packet.payload, sizeof(payload));
                if (payload.packetType == PKT_TYPE_FATIGUE_STATUS &&
                    payload.targetNodeId == NODE_ID) {
                    applyAlarmStatusPayload(payload);
                } else {
                    Serial.printf("[lora_saenab RX STATUS] ignored target=%u status=%u\n",
                                  payload.targetNodeId,
                                  payload.status);
                }
            } else if (packet.header.nextHop == NODE_ID &&
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

void applyAlarmStatusPayload(const FatigueStatusPayload& payload) {
    AlarmState nextState = ALARM_NORMAL;

    if (payload.status == ALARM_LELAH) {
        nextState = ALARM_LELAH;
    } else if (payload.status == ALARM_TERTIDUR) {
        nextState = ALARM_TERTIDUR;
    }

    setAlarmState(nextState);
    Serial.printf("[lora_saenab RX STATUS] target=%u status=%u (%s)\n",
                  payload.targetNodeId,
                  payload.status,
                  alarmText(alarmState));
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
