/*
 * LoRa Mesh Node Configuration
 * Project: PERANCANGAN SISTEM KOMUNIKASI DATA IOT BERBASIS LORA MESH AD HOC
 * Platform: ESP32-S3 + RFM95
 * Protocol: AODV Routing
 */

#ifndef CONFIG_H
#define CONFIG_H

// ================================================================
// LORA CONFIGURATION
// ================================================================
#define LORA_FREQUENCY         921.0   // MHz - Sesuai regulasi Indonesia (Kominfo)
#define LORA_BANDWIDTH         125E3   // 125 kHz (ganti ke 250E3 untuk BW=250 kHz)
#define LORA_SPREADING_FACTOR  7       // SF7 (ganti 7-12 untuk skenario 4)
#define LORA_CODING_RATE       5       // 4/5 (tetap untuk semua skenario)
#define LORA_TX_POWER          20      // dBm (PA_BOOST)
#define LORA_USE_RFO           false   // false = PA_BOOST, true = RFO
#define LORA_PREAMBLE_LENGTH   8       // default
#define LORA_DEBUG_DUMP_REGS   false   // true untuk dump register saat boot

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
// Node IDs: 1-5 untuk mesh nodes, 0 untuk gateway
// SET NODE_ID DI SKETCH (LoRa_Mesh_Node.ino)
#define GATEWAY_ID          0
#define BROADCAST_ADDR      255

// Network parameters
#define MAX_NODES           10
#define MAX_PACKET_SIZE     250      // Maximum LoRa packet size

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
#define PKT_TYPE_FATIGUE_IMU        0x31 // IMU fatigue payload (Node -> Gateway)
#define PKT_TYPE_FATIGUE_STATUS     0x32 // Alarm/status command (Gateway -> Node)
#define PKT_TYPE_SAFETY_CONDITION   0x41 // Safety condition flags (Node -> Gateway)
#define PKT_TYPE_VEHICLE_TELEMETRY  0x09 // BNO055 + GPS + safety payload (Node -> Gateway)

// ================================================================
// DATA COLLECTION PARAMETERS
// Sesuai skenario pengujian skripsi 3.3.4.3 (QoS/PDR/Latensi)
// ================================================================
#define DATA_SEND_INTERVAL  3000     // 3 detik - interval kirim sensor (sesuai skripsi)
#define GPS_UPDATE_INTERVAL 1000     // 1 detik - update GPS dummy data
#define IMU_UPDATE_INTERVAL 1000     // 1 detik - update IMU dummy data

// ================================================================
// DEBUG CONFIGURATION
// ================================================================
#define DEBUG_ENABLE        true     // Enable/disable serial debug output
#define SERIAL_BAUD         115200   // Serial baud rate

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
// GPS Dummy - Area pertambangan di Indonesia (contoh: Kalimantan)
// ================================================================
#define GPS_LAT_BASE        -1.5000  // Latitude base
#define GPS_LAT_RANGE       0.01     // Variance range
#define GPS_LON_BASE        117.0000 // Longitude base
#define GPS_LON_RANGE       0.01     // Variance range
#define GPS_ALT_BASE        100.0    // Altitude base (meter)
#define GPS_ALT_RANGE       50.0     // Variance range

// IMU Dummy - Accelerometer & Gyroscope ranges
#define IMU_ACCEL_RANGE     2.0      // ±2g
#define IMU_GYRO_RANGE      250.0    // ±250 deg/s

#endif // CONFIG_H
