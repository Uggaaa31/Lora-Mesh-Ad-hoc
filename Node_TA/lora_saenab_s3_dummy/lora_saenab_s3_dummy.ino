#include <WiFi.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <time.h>

// Library Lokal Mesh & TDMA
#include "LoRa_Packet.h"
#include "AODV_Routing.h"
#include "WebConfig.h"
#include "config.h"

extern "C" {
  #include "esp_event.h"
  #include "mqtt_client.h"
  #include "esp_crt_bundle.h"
}

// ================================================================
// WIFI & MQTT SETTINGS (PREFIX lora/ ADDED)
// ================================================================
const char* WIFI_SSID = "hmm";
const char* WIFI_PASS = "12345678";

static const char* MQTT_URI    = "wss://mqtt.aistrack.site:443/";
static const char* TOPIC_IMU    = "lora/fms/fatigue_detection/imu";
static const char* TOPIC_STATUS = "lora/fms/fatigue_detection/vision";
static const char* TOPIC_BUZZER = "lora/fms/fatigue_detection/buzzer";

// ================================================================
// HARDWARE PINS (Diselaraskan dengan config.h untuk mencegah bentrok SPI)
// ================================================================
#define RFM95_CS    LORA_CS_PIN
#define RFM95_RST   LORA_RST_PIN
#define RFM95_INT   LORA_DIO0_PIN
#define BUZZER_PIN  3
#define BUZZER_ACTIVE_HIGH 1

#define I2C_SDA 8
#define I2C_SCL 9

// ================================================================
// GLOBALS
// ================================================================
RH_RF95 rf95(RFM95_CS, RFM95_INT);
AODVRouting aodv(NODE_ID);
LoRaPacketHandler packetHandler;
LoRaRuntimeCfg runtimeCfg;
static uint32_t dataSequence = 0;

static bool pendingDataAck = false;
static LoRaPacket pendingDataPacket;
static uint32_t pendingDataSeq = 0;
static uint8_t pendingDataRetries = 0;
static uint8_t consecutiveAckFailures = 0;
static unsigned long pendingDataLastTxMs = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

static esp_mqtt_client_handle_t mqtt = nullptr;
static volatile bool mqttConnected = false;

// Timers
static unsigned long lastWiFiReconnectAttempt = 0;
const unsigned long wifiReconnectIntervalMs = 5000;
static unsigned long lastMqttReconnectAttempt = 0;
const unsigned long mqttReconnectIntervalMs = 8000;
unsigned long lastPub = 0;
const uint32_t pubIntervalMs = 200;

// IMU Filter Logic
float alpha = 0.2f;
bool gyroInit = false;
float f_gx = 0, f_gy = 0, f_gz = 0;
float pitch = 0, roll = 0;
uint8_t cal_sys = 0, cal_gyro = 0, cal_acc = 0, cal_mag = 0;

// Alarm State
enum AlarmState { ALARM_NORMAL, ALARM_LELAH, ALARM_TERTIDUR };
volatile AlarmState alarmState = ALARM_NORMAL;
void setAlarmState(AlarmState s);

unsigned long patternLastMs = 0;
int patternStep = 0;

volatile bool currentBuzzerState = false;
bool lastBuzzerPubState = false;

// Time Sync
bool timeSynced = false;
uint32_t epochOffsetMsLow32 = 0;

// Fixed Interval Scheduling
unsigned long lastLoRaSendTime = 0;
bool initialOffsetDone = false;

// ================================================================
// TIME FORMATTER (FROM USER)
// ================================================================
String formatTs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  time_t now = tv.tv_sec;
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  
  char buffer[32];
  int ms = tv.tv_usec / 1000;
  snprintf(
    buffer, sizeof(buffer),
    "%02d:%02d:%02d:%03d",
    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ms
  );
  return String(buffer);
}

// ================================================================
// BUZZER & PATTERN
// ================================================================
static inline void buzzerWrite(bool on) {
    currentBuzzerState = on;
#if BUZZER_ACTIVE_HIGH
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
#else
    digitalWrite(BUZZER_PIN, on ? LOW : HIGH);
#endif
}

void resetPattern() {
    patternStep = 0;
    patternLastMs = millis();
    buzzerWrite(false);
}

void setAlarmState(AlarmState s) {
    if (s != alarmState) {
        alarmState = s;
        resetPattern();
    }
}

void patternLelah(unsigned long nowMs) {
    const uint16_t onMs  = 120;
    const uint16_t offMs = 120;
    const uint16_t gapMs = 600;
    switch (patternStep) {
        case 0: buzzerWrite(true); if (nowMs - patternLastMs >= onMs) { patternLastMs = nowMs; patternStep = 1; } break;
        case 1: buzzerWrite(false); if (nowMs - patternLastMs >= offMs) { patternLastMs = nowMs; patternStep = 2; } break;
        case 2: buzzerWrite(true); if (nowMs - patternLastMs >= onMs) { patternLastMs = nowMs; patternStep = 3; } break;
        case 3: buzzerWrite(false); if (nowMs - patternLastMs >= offMs) { patternLastMs = nowMs; patternStep = 4; } break;
        case 4: buzzerWrite(true); if (nowMs - patternLastMs >= onMs) { patternLastMs = nowMs; patternStep = 5; } break;
        default: buzzerWrite(false); if (nowMs - patternLastMs >= gapMs) { patternLastMs = nowMs; patternStep = 0; } break;
    }
}

void patternTertidur(unsigned long nowMs) {
    const uint16_t onMs  = 2000;
    const uint16_t offMs = 1000;
    switch (patternStep) {
        case 0: buzzerWrite(true); if (nowMs - patternLastMs >= onMs) { patternLastMs = nowMs; patternStep = 1; } break;
        default: buzzerWrite(false); if (nowMs - patternLastMs >= offMs) { patternLastMs = nowMs; patternStep = 0; } break;
    }
}

// ================================================================
// WIFI & MQTT CORE
// ================================================================
static void connect_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    Serial.print("Connecting WiFi");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi connected");
        configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // UTC+8 Sync
        Serial.print("Syncing NTP Time");
        int retries = 0;
        while (time(nullptr) < 100000 && retries < 20) {
            delay(100);
            Serial.print(".");
            retries++;
        }
        if (time(nullptr) > 100000) {
            Serial.println(" Time Synced via NTP");
            timeSynced = true;
        } else {
            Serial.println(" NTP Sync Failed");
        }
    } else {
        Serial.println("\n❌ WiFi timeout, starting in Hybrid/LoRa mode");
    }
}

static void handleStatusMessage(const char* topic, const char* payload) {
    if (strcmp(topic, TOPIC_STATUS) != 0) return;
    String msg(payload); msg.trim();
    String status;
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, msg) == DeserializationError::Ok) {
        if (doc["status"].is<const char*>()) status = (const char*)doc["status"];
    } else { status = msg; }
    status.toUpperCase();
    if (status == "LELAH") setAlarmState(ALARM_LELAH);
    else if (status == "TERTIDUR") setAlarmState(ALARM_TERTIDUR);
    else if (status == "NORMAL") setAlarmState(ALARM_NORMAL);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqttConnected = true;
            esp_mqtt_client_subscribe(mqtt, TOPIC_STATUS, 1);
            break;
        case MQTT_EVENT_DISCONNECTED: mqttConnected = false; break;
        case MQTT_EVENT_DATA: {
            String t = String(event->topic).substring(0, event->topic_len);
            String p = String(event->data).substring(0, event->data_len);
            handleStatusMessage(t.c_str(), p.c_str());
            break;
        }
        default: break;
    }
}

static void start_mqtt() {
    static char clientId[32];
    snprintf(clientId, sizeof(clientId), "esp32-saenab-%llX", ESP.getEfuseMac());
    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = MQTT_URI;
    cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.credentials.client_id = clientId;
    mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(mqtt, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt);
}

// ================================================================
// IMU SENSING
// ================================================================
static void read_bno055_and_filter(unsigned long nowMs) {
    pitch = 5.0f;
    roll  = -2.0f;
    
    float dummy_gx = 0.1f;
    float dummy_gy = 0.2f;
    float dummy_gz = 0.3f;
    
    if (!gyroInit) {
        f_gx = dummy_gx; f_gy = dummy_gy; f_gz = dummy_gz;
        gyroInit = true;
    } else {
        f_gx = alpha * dummy_gx + (1.0f - alpha) * f_gx;
        f_gy = alpha * dummy_gy + (1.0f - alpha) * f_gy;
        f_gz = alpha * dummy_gz + (1.0f - alpha) * f_gz;
    }
    cal_sys = 3; cal_gyro = 3; cal_acc = 3; cal_mag = 3;
}

// ================================================================
// LORA MESH
// ================================================================
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

void sendDiagnosticCallback(const DiscoveryDiagPayload& diag) {
    if (!aodv.hasRouteTo(GATEWAY_ID)) return;
    static uint32_t diagSeq = 0;
    LoRaPacket pkt = LoRaPacketHandler::createDiagnosticPacket(
        NODE_ID, GATEWAY_ID, diag, ++diagSeq);
    pkt.header.nextHop = aodv.getNextHop(GATEWAY_ID);
    pkt.header.checksum = LoRaPacketHandler::calculateChecksum(pkt);
    delay(500); // Jeda agar tidak collision dengan RREP/data
    sendPacketCallback(pkt);
    
    // Tunda pengiriman data sensor utama agar tidak bertabrakan dengan paket DIAG
    extern unsigned long lastLoRaSendTime;
    lastLoRaSendTime = millis();
    
    Serial.printf("[DIAG TX] target=%u discovery=%lums hops=%u retries=%u success=%u\n",
                  diag.targetNodeId, (unsigned long)diag.discoveryMs,
                  diag.hopCount, diag.retryCount, diag.success);
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
    if (!DATA_ACK_ENABLE || !pendingDataAck) return;
    const uint32_t ackTimeoutMs = getDataAckTimeoutMs();
    const uint8_t ackMaxRetries = getDataAckMaxRetries();

    if ((nowMs - pendingDataLastTxMs) < ackTimeoutMs) return;

    if (pendingDataRetries >= ackMaxRetries) {
        Serial.printf("[ACK] timeout seq=%lu retries=%u, drop\n", (unsigned long)pendingDataSeq, (unsigned)pendingDataRetries);
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
    Serial.printf("[ACK] RETRY seq=%lu, retry=%u/%u\n", (unsigned long)pendingDataSeq, pendingDataRetries, ackMaxRetries);
}

void sendLoRaImuData() {
    if (DATA_ACK_ENABLE && pendingDataAck) {
        return;
    }
    if (!aodv.hasRouteTo(GATEWAY_ID)) {
        aodv.initiateRouteDiscovery(GATEWAY_ID);
        Serial.println("[LoRa] No route to GW, discovering...");
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t nowEpoch = timeSynced
        ? (uint32_t)((uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000))
        : (uint32_t)millis();

    ImuFatiguePayload imu = {};
    imu.packetType = PKT_TYPE_FATIGUE_IMU;
    imu.nodeId = NODE_ID;
    imu.ts = nowEpoch;
    imu.pitch100 = (int16_t)(pitch * 100);
    imu.roll100 = (int16_t)(roll * 100);
    imu.gx1000 = (int16_t)(f_gx * 1000);
    imu.gy1000 = (int16_t)(f_gy * 1000);
    imu.gz1000 = (int16_t)(f_gz * 1000);
    imu.routeDiscMs = 0;
    imu.routeHops = aodv.getRouteHopCount(GATEWAY_ID);
    imu.buzzerActive = currentBuzzerState ? 1 : 0;

    LoRaPacket pkt = packetHandler.createImuFatiguePacket(
        NODE_ID, GATEWAY_ID, imu, ++dataSequence);
    pkt.header.nextHop = aodv.getNextHop(GATEWAY_ID);
    pkt.header.checksum = LoRaPacketHandler::calculateChecksum(pkt);
    
    sendPacketCallback(pkt);
    Serial.printf("[LoRa TX] IMU SF%d BW%dkHz\n", runtimeCfg.sf, runtimeCfg.bwKHz);

    if (DATA_ACK_ENABLE) {
        pendingDataAck = true;
        pendingDataPacket = pkt;
        pendingDataSeq = dataSequence;
        pendingDataRetries = 0;
        pendingDataLastTxMs = millis();
    }
}

// ================================================================
// MAIN LOOP
// ================================================================
void setup() {
    Serial.begin(115200);
    pinMode(BUZZER_PIN, OUTPUT);
    buzzerWrite(false);
    resetPattern();

    // BNO055 Init
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!bno.begin()) Serial.println("❌ BNO055 Fail");
    else bno.setExtCrystalUse(true);

    // LoRa Hardware Init
    pinMode(RFM95_RST, OUTPUT); digitalWrite(RFM95_RST, HIGH);
    delay(10); digitalWrite(RFM95_RST, LOW); delay(10); digitalWrite(RFM95_RST, HIGH); delay(10);
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
    if (!rf95.init()) Serial.println("ERROR: LoRa Fail");
    
    // WebConfig & LoRa Params
    runtimeCfg = WebConfig::getConfig(
        LORA_SPREADING_FACTOR, LORA_BANDWIDTH,
        LORA_CODING_RATE, LORA_TX_POWER, LORA_USE_RFO);
    rf95.setFrequency(LORA_FREQUENCY);
    rf95.setSpreadingFactor(runtimeCfg.sf);
    rf95.setSignalBandwidth(runtimeCfg.bwKHz * 1000.0f);
    rf95.setCodingRate4(runtimeCfg.cr);
    rf95.setTxPower(runtimeCfg.txPower, runtimeCfg.useRFO);
    rf95.setPreambleLength(LORA_PREAMBLE_LENGTH);
    rf95.setCADTimeout(LORA_CAD_TIMEOUT_MS);
    Serial.printf("LoRa OK: SF=%d BW=%dkHz CR=4/%d Freq=%.1fMHz\n",
                  runtimeCfg.sf, runtimeCfg.bwKHz, runtimeCfg.cr, LORA_FREQUENCY);

    // WiFi AP Config Mode (View Only)
    WebConfig::begin(NODE_NAME, runtimeCfg, NODE_ID);

    // AODV Init
    aodv.begin();
    aodv.onSendPacket = sendPacketCallback;
    aodv.onDiagnosticReady = sendDiagnosticCallback;
    aodv.epochOffsetPtr = &epochOffsetMsLow32;

    // WiFi & MQTT Init (Dimatikan untuk Skenario 5)
    // connect_wifi();
    // start_mqtt();
}

void loop() {
    unsigned long nowMs = millis();
    processDataAckTimeout(nowMs);
    WebConfig::handle();
    aodv.update();
    
    // 1. Read & Filter IMU
    read_bno055_and_filter(nowMs);
    
    // 2. Connectivity Checks
    bool internetOk = false; // (WiFi.status() == WL_CONNECTED && mqttConnected);
    
    // 3. MQTT Publish (If Online)
    if (internetOk && (nowMs - lastPub >= pubIntervalMs)) {
        lastPub = nowMs;
        float gyro_magnitude = sqrt(f_gx * f_gx + f_gy * f_gy + f_gz * f_gz);
        
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint32_t nowEpoch = timeSynced
            ? (uint32_t)((uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000))
            : (uint32_t)millis();

        StaticJsonDocument<512> doc;
        doc["nodeId"] = NODE_ID;
        doc["ts"] = formatTs();
        doc["epoch"] = nowEpoch;  // Diperlukan server untuk hitung latency Wi-Fi!
        doc["cal_gyro"] = cal_gyro;
        doc["pitch"] = pitch;
        doc["roll"] = roll;
        doc["f_gx"] = f_gx;
        doc["f_gy"] = f_gy;
        doc["f_gz"] = f_gz;
        doc["kecepatan"] = gyro_magnitude; 
        
        doc["hopCount"] = 0;
        JsonArray rp = doc.createNestedArray("route_path");
        rp.add(NODE_NAME);
        rp.add("WiFi Direct");
        rp.add("GATEWAY");

        char out[512];
        size_t len = serializeJson(doc, out, sizeof(out));
        esp_mqtt_client_publish(mqtt, TOPIC_IMU, out, (int)len, 1, 0);
    }

    // 4. LoRa Failover
    if (!internetOk) {
        if (!initialOffsetDone) {
            lastLoRaSendTime = nowMs - DATA_SEND_INTERVAL + (NODE_ID * 500UL);
            initialOffsetDone = true;
        }
        if (nowMs - lastLoRaSendTime >= DATA_SEND_INTERVAL) {
            sendLoRaImuData();
            lastLoRaSendTime = nowMs;
        }
    }

    // 5. Buzzer Status Change Publish
    if (internetOk && (currentBuzzerState != lastBuzzerPubState)) {
        lastBuzzerPubState = currentBuzzerState;
        StaticJsonDocument<128> doc;
        doc["ts"] = formatTs();
        doc["status_buzzer"] = currentBuzzerState ? "ON" : "OFF";
        char out[128];
        size_t len = serializeJson(doc, out, sizeof(out));
        esp_mqtt_client_publish(mqtt, TOPIC_BUZZER, out, (int)len, 1, 0);
    }

    // 6. Alarm Logic
    if (alarmState == ALARM_LELAH) patternLelah(nowMs);
    else if (alarmState == ALARM_TERTIDUR) patternTertidur(nowMs);
    else if (alarmState == ALARM_NORMAL) buzzerWrite(false);

    // 7. LoRa RX & Sync
    if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; uint8_t len = sizeof(buf);
        if (rf95.recv(buf, &len)) {
            LoRaPacket pkt;
            if (packetHandler.deserializePacket(buf, len, pkt)) {
                // --- ARTIFICIAL TOPOLOGY FILTER ---
                // Jika node ini adalah TRK-003 (Lantai 1) dan paket datang langsung dari Gateway (Lantai 3), kita abaikan
                // agar Node 3 (Lantai 1) tidak menganggap Gateway berjarak 1-Hop!
                if (NODE_ID == 3 && pkt.header.sourceID == GATEWAY_ID && pkt.header.hopCount == 0) {
                    // Ignore
                }
                else if (pkt.header.sourceID == NODE_ID) { /* skip own */ }
                else if (pkt.header.packetType == PKT_TYPE_TIMESYNC) {
                    TimeSyncPayload ts; memcpy(&ts, pkt.payload, sizeof(TimeSyncPayload));
                    struct timeval tv;
                    tv.tv_sec = ts.epochSeconds;
                    tv.tv_usec = ts.millisPart * 1000;
                    settimeofday(&tv, NULL);
                    timeSynced = true;
                } else if (pkt.header.packetType == PKT_TYPE_FATIGUE_STATUS && pkt.header.destinationID == NODE_ID) {
                    FatigueStatusPayload stat; memcpy(&stat, pkt.payload, sizeof(FatigueStatusPayload));
                    if (stat.status == 1) setAlarmState(ALARM_LELAH);
                    else if (stat.status == 2) setAlarmState(ALARM_TERTIDUR);
                    else setAlarmState(ALARM_NORMAL);
                } else if (pkt.header.packetType == PKT_TYPE_ACK && pkt.header.destinationID == NODE_ID) {
                      AckPayload ack = {};
                      memcpy(&ack, pkt.payload, sizeof(ack));
                      if (DATA_ACK_ENABLE && pendingDataAck && 
                          ack.ackedPacketType == PKT_TYPE_FATIGUE_IMU &&
                          ack.ackedSequence == pendingDataSeq) {
                          pendingDataAck = false;
                          consecutiveAckFailures = 0;
                          Serial.printf("[ACK] received seq=%lu retries=%u\n", (unsigned long)pendingDataSeq, pendingDataRetries);
                      }
                  } else if (pkt.header.packetType == PKT_TYPE_START_TEST) {
                    if (pkt.header.payloadLength == sizeof(StartTestPayload)) {
                        StartTestPayload st; memcpy(&st, pkt.payload, sizeof(StartTestPayload));
                        WebConfig::saveTestConfig(st.sf, st.bwKHz); delay(2000); ESP.restart();
                    }
                } else if (pkt.header.packetType == PKT_TYPE_RREQ) { aodv.handleRREQ(pkt); }
                  else if (pkt.header.packetType == PKT_TYPE_RREP) { aodv.handleRREP(pkt); }
                  else if (pkt.header.packetType == PKT_TYPE_RERR) { aodv.handleRERR(pkt); }
                  else if (pkt.header.packetType == PKT_TYPE_HELLO) { aodv.handleHello(pkt); }

                  // --- MESH RELAY / FORWARDING LOGIC ---
                  uint8_t ptype = pkt.header.packetType;
                  bool isForwardPayload = (ptype == PKT_TYPE_DATA || ptype == PKT_TYPE_FATIGUE_IMU ||
                                           ptype == PKT_TYPE_FATIGUE_STATUS || ptype == PKT_TYPE_SAFETY_CONDITION ||
                                           ptype == PKT_TYPE_VEHICLE_TELEMETRY || ptype == PKT_TYPE_DIAGNOSTIC ||
                                           ptype == PKT_TYPE_ACK);

                  if (isForwardPayload && pkt.header.nextHop == NODE_ID && pkt.header.destinationID != NODE_ID) {
                      if (aodv.hasRouteTo(pkt.header.destinationID) && pkt.header.hopCount < MAX_HOP_COUNT) {
                          LoRaPacket fwd = pkt;
                          fwd.header.hopCount++;
                          if (fwd.header.routePathLen < MAX_ROUTE_PATH) {
                              fwd.header.routePath[fwd.header.routePathLen] = NODE_ID;
                              fwd.header.routePathLen++;
                          }
                          fwd.header.nextHop = aodv.getNextHop(pkt.header.destinationID);
                          fwd.header.checksum = LoRaPacketHandler::calculateChecksum(fwd);
                          sendPacketCallback(fwd);
                          Serial.printf("[RELAY] src=%u dst=%u type=0x%02X hop=%u via=%s\n",
                                        fwd.header.sourceID, fwd.header.destinationID, ptype, fwd.header.hopCount, NODE_NAME);
                      }
                  }
            }
        }
    }
    
    delay(5);
}
