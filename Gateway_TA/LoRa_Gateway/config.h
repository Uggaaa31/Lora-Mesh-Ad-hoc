/*
 * LoRa Gateway Configuration
 * Project: PERANCANGAN SISTEM KOMUNIKASI DATA IOT BERBASIS LORA MESH AD HOC
 * Platform: ESP32-S3 + RFM95
 * Protocol: AODV Routing
 */

#ifndef CONFIG_H
#define CONFIG_H

struct WiFiProfile {
    const char* ssid;
    const char* password;
};

#if defined(__has_include)
  #if __has_include("secrets.h")
    #include "secrets.h"
  #endif
#endif

// ================================================================
// LORA CONFIGURATION
// Ganti SF dan BW untuk pengujian skenario 4 (Pengaruh Parameter LoRa)
// ================================================================
#define LORA_FREQUENCY         921.0   // MHz - Sesuai regulasi Indonesia (Kominfo)
#define LORA_BANDWIDTH         125E3   // 125 kHz (ganti ke 250E3 untuk BW=250 kHz)
#define LORA_SPREADING_FACTOR  7       // SF7 (ganti 7-12 untuk skenario 4)
#define LORA_CODING_RATE       5       // 4/5 (tetap untuk semua skenario)
#define LORA_TX_POWER          20      // dBm (RFO path, max 15)
#define LORA_USE_RFO           false    // true = RFO, false = PA_BOOST
#define LORA_PREAMBLE_LENGTH   8       // default
#define LORA_DEBUG_DUMP_REGS   false   // true untuk dump register LoRa saat boot

// ================================================================
// ESP32 PIN CONFIGURATION (SPI-B/HSPI untuk RFM95)
// ================================================================
#define LORA_SCK_PIN        42       // SPI-B SCK
#define LORA_MOSI_PIN       41       // SPI-B MOSI
#define LORA_MISO_PIN       40       // SPI-B MISO
#define LORA_CS_PIN         5        // LoRa CS
#define LORA_RST_PIN        7        // LoRa reset (output)
#define LORA_DIO0_PIN       6        // LoRa DIO0/IRQ (input)

// ================================================================
// NETWORK CONFIGURATION
// ================================================================
#define GATEWAY_ID          0
#define NODE_ID             GATEWAY_ID
#define BROADCAST_ADDR      255
#define MAX_NODES           10
#define MAX_PACKET_SIZE     250      // Maximum LoRa packet size
#define MAX_TRACKED_NODES   6

// Node role mapping for this gateway deployment
#define FATIGUE_NODE_ID     4
#define VEHICLE_NODE_ID     5

// ================================================================
// AODV PROTOCOL PARAMETERS
// ================================================================
#define ROUTE_TIMEOUT          60000  // 60 detik - route expiration time
#define RREQ_TIMEOUT           5000   // 5 detik - timeout untuk RREQ retry
#define RREQ_RETRIES           3      // Jumlah retry untuk RREQ
#define HELLO_INTERVAL         30000  // 30 detik - periodic hello message (dikurangi untuk anti-kolisi)
#define MAX_HOP_COUNT          10     // Maximum hop count
#define ROUTE_CLEANUP_INTERVAL 60000  // 60 detik - garbage collection interval

// ================================================================
// PACKET TYPES
// ================================================================
#define PKT_TYPE_DATA       0x01     // Data packet (sensor data)
#define PKT_TYPE_RREQ       0x02     // Route Request
#define PKT_TYPE_RREP       0x03     // Route Reply
#define PKT_TYPE_RERR       0x04     // Route Error
#define PKT_TYPE_HELLO      0x05     // Hello message
#define PKT_TYPE_TIMESYNC   0x06     // Time Sync (Gateway -> Nodes, untuk akurasi latensi)
#define PKT_TYPE_FATIGUE_IMU    0x31 // IMU fatigue payload (Node -> Gateway)
#define PKT_TYPE_FATIGUE_STATUS 0x32 // Alarm/status command (Gateway -> Node)
#define PKT_TYPE_SAFETY_CONDITION 0x41 // Safety condition flags (Node -> Gateway)
#define PKT_TYPE_VEHICLE_TELEMETRY 0x09 // Legacy payload compatibility
#define PKT_TYPE_DIAGNOSTIC         0x0A // Route discovery diagnostic (Node -> Gateway)
#define PKT_TYPE_START_TEST         0x0B // Start test command (Gateway -> Nodes, broadcast)

// ================================================================
// DATA COLLECTION PARAMETERS
// CATATAN: Gateway hanya MENERIMA data, bukan mengirim sensor.
// Nilai berikut adalah REFERENSI — harus sama dengan Node_TA/config.h
// ================================================================
#define DATA_SEND_INTERVAL  3000     // 3 detik - interval kirim sensor (harus sama dengan Node)
#define GPS_UPDATE_INTERVAL 100      // 100 ms - Referensi (harus sama dengan Node)
#define IMU_UPDATE_INTERVAL 100      // 100 ms - Referensi (harus sama dengan Node)
#define OBSERVATION_PERIOD_MS            300000UL
#define DEFAULT_NODE_SEND_INTERVAL_MS    3000UL
#define FATIGUE_NODE_SEND_INTERVAL_MS    3000UL
#define VEHICLE_NODE_SEND_INTERVAL_MS    3000UL
#define TIMESYNC_INTERVAL_MS             30000UL
#define FATIGUE_STATUS_ROUTE_RETRY_MS    5000UL

// Safety condition bit flags
#define FLAG_GPS_VALID      0x01
#define FLAG_DR_ACTIVE      0x02
#define FLAG_ROLLOVER_RISK  0x04
#define FLAG_ROLLOVER       0x08
#define FLAG_HARSH_BRAKE    0x10
#define FLAG_OVERSPEED      0x20

// ================================================================
// GATEWAY WIFI CONFIGURATION
// Override in local secrets.h (not committed to git)
// Secara default hanya ada 1 profil WiFi. Jika ingin beberapa pilihan,
// definisikan WIFI_PROFILES di secrets.h.
// ================================================================
#define GATEWAY_CONFIG_SSID "LoRa-Gateway-CFG"

#ifndef HAS_WIFI_PROFILE_LIST
  #ifndef WIFI_SSID
    #define WIFI_SSID         "YOUR_WIFI_SSID"
  #endif
  #ifndef WIFI_PASSWORD
    #define WIFI_PASSWORD     "YOUR_WIFI_PASSWORD"
  #endif

  static const WiFiProfile WIFI_PROFILES[] = {
      {WIFI_SSID, WIFI_PASSWORD}
  };
#endif

#define WIFI_PROFILE_COUNT  (sizeof(WIFI_PROFILES) / sizeof(WIFI_PROFILES[0]))
#define WIFI_TIMEOUT        10000    // 10 detik - timeout koneksi WiFi

// ================================================================
// SERVER CONFIGURATION - MQTT
// ================================================================
#define USE_MQTT            true
#define MQTT_URI            "wss://mqtt.aistrack.site:443/"
#define MQTT_TOPIC_PREFIX   "lora/fms"
#define MQTT_TOPIC_DATA     "lora/fms"           // Sensor data per node
#define MQTT_TOPIC_STATUS   "lora/fms/status"    // Statistik QoS gateway
#define MQTT_TOPIC_DEBUG    "lora/fms/debug"     // Debug messages
#define MQTT_TOPIC_FATIGUE_IMU    "lora/fms/fatigue_detection/imu"
#define MQTT_TOPIC_FATIGUE_STATUS "lora/fms/fatigue_detection/vision"
#define MQTT_TOPIC_SAFETY_CONDITION "lora/fms/safety/condition"
#define MQTT_TOPIC_BNO_DATA       "lora/fms/bno/data"
#define MQTT_TOPIC_DIAGNOSTIC     "lora/fms/diagnostic/route"

// HTTP fallback (jika USE_MQTT = false)
#define SERVER_URL          "http://192.168.1.100:3000/api/lora-data"
#define HTTP_TIMEOUT        5000

// MQTT Settings
#define MQTT_KEEPALIVE      60
#define MQTT_TIMEOUT        10000
#define MQTT_BUFFER_SIZE    512

// ================================================================
// DEBUG CONFIGURATION
// ================================================================
#define DEBUG_ENABLE        true
#define SERIAL_BAUD         115200

#if DEBUG_ENABLE
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

// ================================================================
// SENSOR DUMMY DATA RANGES (untuk simulasi)
// GPS Dummy - Area pertambangan di Indonesia (Kalimantan)
// ================================================================
#define GPS_LAT_BASE        -1.5000  // Latitude base
#define GPS_LAT_RANGE       0.01
#define GPS_LON_BASE        117.0000 // Longitude base
#define GPS_LON_RANGE       0.01
#define GPS_ALT_BASE        100.0    // Altitude base (meter)
#define GPS_ALT_RANGE       50.0

// IMU Dummy
#define IMU_ACCEL_RANGE     2.0      // ±2g
#define IMU_GYRO_RANGE      250.0    // ±250 deg/s

#endif // CONFIG_H
