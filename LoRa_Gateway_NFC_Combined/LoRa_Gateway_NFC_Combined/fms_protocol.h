#ifndef FMS_PROTOCOL_H
#define FMS_PROTOCOL_H

#define API_HOST "aispektra.com"
#define API_PATH_INGEST "/api/ingest"
#define MAX_BACKLOG_SYNC_RATE 2 // Lebih aman untuk WSS handshakes (2 data per detik)

// LCD/TFT enable switch (temporary disable for SPI isolation testing).
// 1 = LCD/TFT enabled, 0 = disabled (no init/poll).
#define FMS_LCD_ENABLED 0

// ================================================================
// LoRa BACKUP FALLBACK CONFIGURATION
// ================================================================
// Skalabilitas:
//   - Airtime 1 paket DATA (198 bytes @ SF10+CR4/8+BW125) ≈ 5.4 detik
//   - Guard time antar TX ≈ 2 detik → slot_time = 7.4 detik/node
//   - Dengan interval 60s + jitter ±15s → mendukung ~100 node/gateway
//     tanpa collision (tiap node kirim 1x/menit, tersebar random)
//   - Untuk >100 node: gunakan gateway tambahan atau naikkan interval
// ================================================================
#define FMS_LORA_BACKUP_ENABLED          1
#define FMS_LORA_BACKUP_NODE_ID          1
#define FMS_LORA_BACKUP_NODE_NAME        "TRK-002"
#define FMS_LORA_BACKUP_SEND_INTERVAL_MS 60000UL  // 60 detik: scalable untuk ~100 node/gateway
#define FMS_LORA_BACKUP_RETRY_INTERVAL_MS 10000UL // 10 detik: retry jika belum ada route
#define FMS_LORA_BACKUP_CATCHUP_INTERVAL_MS 25000UL // 25 detik: drain backlog bertahap (tetap ramah channel)
#define FMS_LORA_BACKUP_QUEUE_DEPTH      64         // Buffer lokal saat outage panjang (RAM ring buffer)
#define FMS_LORA_BACKUP_INGRESS_QUEUE_DEPTH 12      // Queue antar-task sebelum masuk ring buffer
#define FMS_LORA_BACKUP_MAX_BACKOFF_MS   90000UL    // Backoff maksimum saat channel/rute buruk
#define FMS_LORA_BACKUP_TASK_CORE        1
#define FMS_LORA_BACKUP_TASK_PRIORITY    3

/**
 * PROTOCOL SWITCHER
 * -----------------
 * 0 = HTTPS Polling Mode (Default, more stable for some networks)
 * 1 = Pure MQTT Mode (Real-time, faster responses)
 *
 * NOTE: Changing this requires a RE-COMPILE and RE-UPLOAD.
 */
#define USE_MQTT_PURE 1

#endif // FMS_PROTOCOL_H