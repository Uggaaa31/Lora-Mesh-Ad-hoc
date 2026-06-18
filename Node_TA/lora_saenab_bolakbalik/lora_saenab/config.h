/*
 * LoRa Mesh Node Configuration
 * Project: PERANCANGAN SISTEM KOMUNIKASI DATA IOT BERBASIS LORA MESH AD HOC
 * Platform: ESP32-C3 Super Mini + RFM95
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
#define LORA_TX_POWER          20      // Turunkan ke 14dBm agar tidak restart/brownout saat WiFi nyala
#define LORA_USE_RFO           false   // false = PA_BOOST, true = RFO
#define LORA_PREAMBLE_LENGTH   8       // default
#define LORA_CAD_TIMEOUT_MS    800     // tunggu kanal clear sebelum TX
#define LORA_DEBUG_DUMP_REGS   false   // true untuk dump register saat boot

// ================================================================
// ESP32-C3 Super Mini PIN CONFIGURATION untuk RFM95
// ================================================================
#define LORA_SCK_PIN        4        // SPI SCK
#define LORA_MOSI_PIN       6        // SPI MOSI
#define LORA_MISO_PIN       5        // SPI MISO
#define LORA_CS_PIN         7        // LoRa SS/CS
#define LORA_RST_PIN        10       // LoRa reset (output)
#define LORA_DIO0_PIN       1        // LoRa DIO0/IRQ (input)

// ================================================================
// NETWORK CONFIGURATION
// ================================================================
// Node IDs: 1-5 untuk mesh nodes, 0 untuk gateway
#define NODE_ID             4
#define NODE_NAME           "lora_saenab"
#define GATEWAY_ID          0
#define BROADCAST_ADDR      255

// Network parameters
#define MAX_NODES           10
#define MAX_PACKET_SIZE     250      // Maximum LoRa packet size

// ================================================================
// AODV PROTOCOL PARAMETERS
// ================================================================
#define ROUTE_TIMEOUT          180000 // 3 menit - route expiration time (diperpanjang agar tidak sering re-discovery)
#define RREQ_TIMEOUT           5000   // 5 detik - timeout untuk RREQ retry
#define RREQ_RETRIES           3      // Jumlah retry untuk RREQ
#define HELLO_INTERVAL         45000  // 45 detik - periodic hello message (diperpanjang untuk kurangi beban udara)
#define MAX_HOP_COUNT          10     // Maximum hop count
#define ROUTE_CLEANUP_INTERVAL 90000  // 90 detik - garbage collection interval

// ================================================================
#define PKT_TYPE_DATA       0x01     // Data packet (sensor data)
#define PKT_TYPE_RREQ       0x02     // Route Request
#define PKT_TYPE_RREP       0x03     // Route Reply
#define PKT_TYPE_RERR       0x04     // Route Error
#define PKT_TYPE_HELLO      0x05     // Hello message
#define PKT_TYPE_TIMESYNC   0x06     // Time Sync (Gateway -> Node)
#define PKT_TYPE_ACK        0x07     // ACK data packet (app-layer reliability)
#define PKT_TYPE_FATIGUE_IMU    0x31 // IMU fatigue payload (Node -> Gateway)
#define PKT_TYPE_FATIGUE_STATUS 0x32 // Alarm/status command (Gateway -> Node)
#define PKT_TYPE_SAFETY_CONDITION 0x41 // Safety condition flags (Node -> Gateway)
#define PKT_TYPE_VEHICLE_TELEMETRY 0x09 // Legacy payload forwarding compatibility
#define PKT_TYPE_DIAGNOSTIC         0x0A // Route discovery diagnostic (Node -> Gateway)
#define PKT_TYPE_START_TEST         0x0B // Start test command (Gateway -> Nodes, broadcast)

// ================================================================
#define DATA_SEND_INTERVAL  3000     // 3 detik - interval kirim sensor (sesuai skripsi)
#define GPS_UPDATE_INTERVAL 100      // 100 ms - update GPS dummy data
#define IMU_UPDATE_INTERVAL 100      // 100 ms - update IMU dummy data
#define DATA_ACK_ENABLE     true     // true = aktifkan ACK data + retry terbatas
#define DATA_ACK_TIMEOUT_MS 2500     // Fallback timeout ACK (ms) bila SF bukan 7/9/12
#define DATA_ACK_MAX_RETRIES 1       // Fallback retry maksimum bila SF bukan 7/9/12

// Tuning ACK khusus SF untuk trade-off delay vs PDR
#define DATA_ACK_TIMEOUT_SF7_MS   1000
#define DATA_ACK_TIMEOUT_SF9_MS   1500
#define DATA_ACK_TIMEOUT_SF12_MS  7000

#define DATA_ACK_MAX_RETRIES_SF7  1
#define DATA_ACK_MAX_RETRIES_SF9  1
#define DATA_ACK_MAX_RETRIES_SF12 1
#define IMU_PUBLISH_INTERVAL_MS 3000UL // Setel ke 3 detik sesuai permintaan
#define STATUS_PRINT_INTERVAL_MS 60000UL
#define GYRO_EMA_ALPHA      0.2f

// ================================================================
// MODULE PIN & BEHAVIOR CONFIGURATION
// ================================================================
#define BUZZER_PIN          3
#define BUZZER_ACTIVE_HIGH  1
#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9

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
#define IMU_ACCEL_RANGE     2.0      // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â±2g
#define IMU_GYRO_RANGE      250.0    // ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â±250 deg/s

#endif // CONFIG_H

