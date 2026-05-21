#include <WiFi.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <SPI.h>
#include <RH_RF95.h>

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
static const char* TOPIC_IMU    = "fms/fatigue_detection/imu";
static const char* TOPIC_STATUS = "fms/fatigue_detection/vision";

// ================================================================
// HARDWARE PINS (Diselaraskan dengan config.h untuk mencegah bentrok SPI)
// ================================================================
#define RFM95_CS    LORA_CS_PIN
#define RFM95_RST   LORA_RST_PIN
#define RFM95_INT   LORA_DIO0_PIN

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

// IMU Filter Logic (FROM YOUR SCRIPT)
float alpha = 0.2f;
bool gyroInit = false;
float f_gx = 0, f_gy = 0, f_gz = 0;
float pitch = 0, roll = 0;
uint8_t cal_sys = 0, cal_gyro = 0, cal_acc = 0, cal_mag = 0;

// Time Sync
uint32_t epochOffsetMsLow32 = 0;
bool timeSynced = false;

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
    // Status diterima, tanpa aksi tambahan.
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
// IMU SENSING (EXACTLY AS YOUR SCRIPT)
// ================================================================
static void read_bno055_and_filter(unsigned long nowMs) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    pitch = euler.y();
    roll  = euler.z();
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    if (!gyroInit) {
        f_gx = gyro.x(); f_gy = gyro.y(); f_gz = gyro.z();
        gyroInit = true;
    } else {
        f_gx = alpha * gyro.x() + (1.0f - alpha) * f_gx;
        f_gy = alpha * gyro.y() + (1.0f - alpha) * f_gy;
        f_gz = alpha * gyro.z() + (1.0f - alpha) * f_gz;
    }
    bno.getCalibration(&cal_sys, &cal_gyro, &cal_acc, &cal_mag);
}

// ================================================================
// LORA MESH — Fixed 3s Interval (bukan TDMA adaptive)
// ================================================================
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

void sendLoRaImuData() {
    if (!aodv.hasRouteTo(GATEWAY_ID)) {
        aodv.initiateRouteDiscovery(GATEWAY_ID);
        Serial.println("[LoRa] No route to GW, discovering...");
        return;
    }

    uint32_t nowEpoch = timeSynced
        ? ((uint32_t)millis() + epochOffsetMsLow32)
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

    LoRaPacket pkt = packetHandler.createImuFatiguePacket(
        NODE_ID, GATEWAY_ID, imu, ++dataSequence);
    pkt.header.nextHop = aodv.getNextHop(GATEWAY_ID);
    pkt.header.checksum = LoRaPacketHandler::calculateChecksum(pkt);
    
    sendPacketCallback(pkt);
    Serial.printf("[LoRa TX] IMU SF%d BW%dkHz\n", runtimeCfg.sf, runtimeCfg.bwKHz);
}

// ================================================================
// MAIN LOOP
// ================================================================
void setup() {
    Serial.begin(115200);
    // BNO055 Init (FROM YOUR SCRIPT)
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
    Serial.printf("LoRa OK: SF=%d BW=%dkHz CR=4/%d Freq=%.1fMHz\n",
                  runtimeCfg.sf, runtimeCfg.bwKHz, runtimeCfg.cr, LORA_FREQUENCY);

    // WiFi AP Config Mode (View Only)
    WebConfig::begin(NODE_NAME, runtimeCfg, NODE_ID);

    // AODV Init
    aodv.begin();
    aodv.onSendPacket = sendPacketCallback;

    // WiFi & MQTT Init
    connect_wifi();
    start_mqtt();
}

void loop() {
    unsigned long nowMs = millis();
    WebConfig::handle();
    aodv.update();
    
    // 1. Read & Filter IMU
    read_bno055_and_filter(nowMs);
    
    // 2. Connectivity Checks
    bool internetOk = (WiFi.status() == WL_CONNECTED && mqttConnected);
    
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
        doc["epoch"] = nowEpoch;
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

    // 4. LoRa Failover — hanya kirim saat WiFi off/no internet
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

    // 5. LoRa RX & Sync
    if (rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; uint8_t len = sizeof(buf);
        if (rf95.recv(buf, &len)) {
            LoRaPacket pkt;
            if (packetHandler.deserializePacket(buf, len, pkt)) {
                if (pkt.header.sourceID == NODE_ID) { /* skip own */ }
                else if (pkt.header.packetType == PKT_TYPE_TIMESYNC) {
                    TimeSyncPayload ts; memcpy(&ts, pkt.payload, sizeof(TimeSyncPayload));
                    
                    // Set RTC Internal ESP32 menggunakan data dari Gateway!
                    struct timeval tv;
                    tv.tv_sec = ts.epochSeconds;
                    tv.tv_usec = ts.millisPart * 1000;
                    settimeofday(&tv, NULL);
                    
                    timeSynced = true;
                } else if (pkt.header.packetType == PKT_TYPE_FATIGUE_STATUS && pkt.header.destinationID == NODE_ID) {
                    FatigueStatusPayload stat; memcpy(&stat, pkt.payload, sizeof(FatigueStatusPayload));
                    // Status diterima, tanpa aksi tambahan.
                } else if (pkt.header.packetType == PKT_TYPE_START_TEST) {
                    if (pkt.header.payloadLength == sizeof(StartTestPayload)) {
                        StartTestPayload st; memcpy(&st, pkt.payload, sizeof(StartTestPayload));
                        Serial.printf("[START_TEST] SF=%u BW=%lukHz\n", st.sf, (unsigned long)st.bwKHz);
                        WebConfig::saveTestConfig(st.sf, st.bwKHz); delay(2000); ESP.restart();
                    }
                } else if (pkt.header.packetType == PKT_TYPE_RREQ) { aodv.handleRREQ(pkt); }
                  else if (pkt.header.packetType == PKT_TYPE_RREP) { aodv.handleRREP(pkt); }
                  else if (pkt.header.packetType == PKT_TYPE_RERR) { aodv.handleRERR(pkt); }
                  else if (pkt.header.packetType == PKT_TYPE_HELLO) { aodv.handleHello(pkt); }
            }
        }
    }
    
    delay(5);
}
