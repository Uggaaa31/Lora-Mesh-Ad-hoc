# PERANCANGAN SISTEM KOMUNIKASI DATA IoT BERBASIS LORA MESH AD HOC
## Studi Kasus: Area Pertambangan

**Platform**: ESP32-S3 / ESP32-C3 + RFM95W  
**Protokol**: AODV (Ad-hoc On-Demand Distance Vector)  
**Frekuensi**: 921 MHz (regulasi Indonesia — Kominfo)

---

## Arsitektur Jaringan

```
┌──────────────────────────────────────────────────────────────────┐
│                        MQTT Broker                               │
│                    (wss://mqtt.aistrack.site)                     │
└────────────────────────────┬─────────────────────────────────────┘
                             │ WiFi + MQTT
                    ┌────────┴────────┐
                    │  GATEWAY (ID=0) │
                    │   ESP32-S3      │
                    │  LoRa + WiFi    │
                    └────────┬────────┘
                             │ LoRa 921 MHz
           ┌─────────────────┼─────────────────┐
           │                 │                 │
    ┌──────┴──────┐   ┌──────┴──────┐   ┌──────┴──────┐
    │ TRK-001 (1) │   │ TRK-002 (2) │   │ TRK-003 (3) │
    │  ESP32-S3   │   │  ESP32-S3   │   │  ESP32-S3   │
    │ Sensor Dummy│   │ Sensor Dummy│   │ Sensor Dummy│
    └─────────────┘   └─────────────┘   └─────────────┘
           │                                    │
    ┌──────┴──────┐                      ┌──────┴──────┐
    │lora_saenab  │                      │lora_nailah  │
    │   (ID=4)    │                      │   (ID=5)    │
    │ ESP32-C3    │                      │  ESP32-S3   │
    │ BNO055 IMU  │                      │ BNO055+GPS  │
    │ Fatigue Det │                      │Vehicle Tele │
    │ WiFi+MQTT   │                      │             │
    └─────────────┘                      └─────────────┘
```

## Struktur Direktori

```
Tugas Akhir_Lora_Gabungan_parameter_Retry/
├── Gateway_TA/
│   └── LoRa_Gateway/
│       ├── LoRa_Gateway.ino     # Firmware gateway (WiFi + MQTT + LoRa)
│       ├── config.h             # Konfigurasi gateway
│       ├── LoRa_Packet.h/.cpp   # Unified packet library v3.0
│       ├── AODV_Routing.h/.cpp  # AODV routing protocol
│       ├── WebConfig.h          # GWWebConfig (Web UI + START_TEST)
│       ├── secrets.h            # WiFi credentials (git-ignored)
│       └── secrets.example.h   # Template secrets
│
├── Node_TA/
│   ├── LoRa_Mesh_Node/          # TRK-001, ID=1, ESP32-S3
│   ├── LoRa_Node_TRK002/        # TRK-002, ID=2, ESP32-S3
│   ├── LoRa_Node_TRK003/        # TRK-003, ID=3, ESP32-S3
│   ├── lora_saenab/             # ID=4, ESP32-C3 Super Mini, BNO055
│   └── lora_nailah/             # ID=5, ESP32-S3, BNO055 + GPS NEO-6M
│
└── README.md
```

## Hardware Mapping

| Node | Board | NODE_ID | Pin CS | Pin RST | Pin DIO0 | SPI Pins |
|------|-------|---------|--------|---------|----------|----------|
| Gateway | ESP32-S3 | 0 | 5 | 7 | 6 | SCK=42, MOSI=41, MISO=40 |
| TRK-001 | ESP32-S3 | 1 | 5 | 7 | 1 | SCK=42, MOSI=41, MISO=40 |
| TRK-002 | ESP32-S3 | 2 | 5 | 7 | 1 | SCK=42, MOSI=41, MISO=40 |
| TRK-003 | ESP32-S3 | 3 | 5 | 7 | 1 | SCK=42, MOSI=41, MISO=40 |
| lora_saenab | ESP32-C3 | 4 | 7 | 10 | 1 | SCK=4, MOSI=6, MISO=5 |
| lora_nailah | ESP32-S3 | 5 | 5 | 7 | 1 | SCK=42, MOSI=41, MISO=40 |

## Parameter LoRa Default

| Parameter | Nilai | Keterangan |
|-----------|-------|------------|
| Frekuensi | 921.0 MHz | Regulasi Indonesia |
| Spreading Factor | SF7 | Dapat diubah via WebConfig/START_TEST |
| Bandwidth | 125 kHz | Dapat diubah via WebConfig/START_TEST |
| Coding Rate | 4/5 | Tetap |
| TX Power | 20 dBm (PA_BOOST) | Saenab: 14 dBm |
| Preamble | 8 | Default |

## Tipe Paket (Packet Types)

| Hex | Nama | Arah | Deskripsi |
|-----|------|------|-----------|
| 0x01 | DATA | Node → GW | Sensor dummy (GPS, IMU) |
| 0x02 | RREQ | Broadcast | AODV Route Request |
| 0x03 | RREP | Unicast | AODV Route Reply |
| 0x04 | RERR | Broadcast | AODV Route Error |
| 0x05 | HELLO | Broadcast | AODV neighbor discovery |
| 0x06 | TIMESYNC | GW → Nodes | Sinkronisasi waktu epoch |
| 0x09 | VEHICLE_TELEMETRY | Node → GW | BNO055 + GPS + safety flags |
| 0x0A | DIAGNOSTIC | Node → GW | Route discovery metrics |
| 0x0B | START_TEST | GW → Nodes | Broadcast perubahan SF/BW |
| 0x31 | FATIGUE_IMU | Node → GW | Pitch/roll/gyro dari BNO055 |
| 0x32 | FATIGUE_STATUS | GW → Node | Alarm command (lelah/tertidur) |
| 0x41 | SAFETY_CONDITION | Node → GW | Safety flags + accelerometer |

## Ukuran Paket & Payload LoRa (LoRa Packet & Payload Sizes)

Setiap paket LoRa yang dikirimkan melalui udara terdiri dari **Header** (tetap **19 Byte**) dan **Payload** (bervariasi sesuai tipe paket). Pengemasan menggunakan *struct packing* (`__attribute__((packed))`) untuk menghilangkan *padding compiler* demi meminimalkan ukuran transmisi (*airtime*).

| Tipe Paket | Nama Struct | Ukuran Payload (Byte) | Total Ukuran Paket LoRa (Header + Payload) (Byte) | Deskripsi Payload |
|---|---|---|---|---|
| **0x01 (DATA)** | `SensorDataPayload` | 70 | 89 | Data GPS (16B), IMU (28B), Battery (4B), RSSI (2B), Timestamps & Hops (9B), **Padding (11B)** |
| **0x02 (RREQ)** | `RREQPayload` | 15 | 34 | Originator & Destination ID, RREQ ID, Sequence Numbers, Hop Count |
| **0x03 (RREP)** | `RREPPayload` | 11 | 30 | Originator & Destination ID, Destination Seq, Hop Count, Lifetime |
| **0x04 (RERR)** | `RERRPayload` | 5 | 24 | Unreachable Node ID, Unreachable Sequence Number |
| **0x05 (HELLO)**| *(Tidak ada payload)* | 0 | 19 | Hanya Header LoRa (digunakan sebagai *beacon* tetangga) |
| **0x06 (TIMESYNC)**| `TimeSyncPayload` | 6 | 25 | Epoch Unix Seconds (4B) dan Milliseconds Part (2B) |
| **0x09 (VEHICLE)** | `VehicleTelemetryPayload` | 120 | 139 | GPS lengkap, IMU, Kalman yaw/pitch/roll, Safety Flags, **Padding (23B)** |
| **0x0A (DIAG)** | `DiscoveryDiagPayload` | 17 | 36 | Metrik RREQ/RREP, Latensi Pembentukan Rute (ms), Retry Count, Status Sukses |
| **0x0B (TEST)** | `StartTestPayload` | 5 | 24 | Target Spreading Factor (1B) dan Bandwidth kHz (4B) untuk perubahan dinamis |
| **0x31 (FATIGUE)** | `ImuFatiguePayload` | 30 | 49 | Node ID, Pitch & Roll (4B), Gyro 3-Axis (6B), Buzzer Status (1B), Timestamps (8B), **Padding (10B)** |
| **0x32 (FAT_STAT)**| `FatigueStatusPayload`| 3 | 22 | Target Node ID, Status Alarm (0 = Normal, 1 = Lelah, 2 = Tertidur) |
| **0x41 (SAFETY)**  | `SafetyConditionPayload`| 12 | 31 | Node ID, Timestamp (4B), Safety Flags (1B), Routing Metrik (5B) |

## Fitur Utama

### 1. Deterministic Scheduling (Fixed 3-Second Interval)
- Setiap node mengirim data setiap **3 detik** (`DATA_SEND_INTERVAL = 3000`)
- Offset awal per node: `NODE_ID × 500ms` untuk menghindari tabrakan saat boot
- **Menggantikan** Adaptive TDMA yang rumit dan rawan desinkronisasi

### 2. Duplicate Packet Detection
- Setiap node dan gateway memiliki **ring buffer cache** (32 atau 64 entry)
- Paket dari `sourceID + seqNum + packetType` yang sama **di-drop** jika diterima dalam 10 detik
- Mencegah relay loop dan duplikasi data di gateway

### 3. START_TEST Broadcast
- Gateway mengirim `PKT_TYPE_START_TEST (0x0B)` ke semua node via broadcast
- Node menerima → simpan SF/BW ke NVRAM → reboot dengan parameter baru
- **Trigger**: Serial CLI `TEST SF12 BW250` atau tombol Web UI
- Dikirim 2x untuk reliability

### 4. Hybrid Connectivity (lora_saenab & lora_nailah)
- **Mode prioritas**: MQTT/WiFi aktif → data dikirim via internet
- **Failover**: WiFi mati/MQTT disconnect → otomatis kirim via LoRa mesh
- `lora_saenab`: mendeteksi `internetOk = WiFi + MQTT`, fallback ke LoRa
- `lora_nailah`: selalu kirim via LoRa (vehicle telemetry)

### 5. Per-Packet CSV Logging (Gateway)
```
Format: [CSV] EPOCH_MS,NODE_ID,NODE_NAME,TYPE,SEQ,RSSI,SNR,HOPS,LATENCY_MS,SF,BW_KHZ
Contoh: [CSV] 1747512345,1,TRK-001,DATA,42,-87,7,1,125,7,125
```
- Setiap paket data yang diterima gateway → output satu baris CSV
- Dapat di-capture via serial monitor atau PuTTY logging
- Prefiks `~` pada latency berarti NTP belum sinkron

### 6. QoS Periodic Report
- Otomatis setiap 5 menit (`OBSERVATION_PERIOD_MS = 300000`)
- Menampilkan PDR, PLR, latency (avg/min/max) per node
- Output format tabel dan CSV

## Serial CLI Commands (Gateway)

| Command | Fungsi |
|---------|--------|
| `STATS` | Tampilkan tabel QoS |
| `CSV` | Tampilkan CSV summary per observation period |
| `RESET` / `START` | Reset counters, mulai observation baru |
| `STATUS` | Status gateway (WiFi, MQTT, NTP, queue) |
| `SYNC` | Kirim TimeSyncPacket ke semua node |
| `ROUTE` | Statistik route discovery (OK/FAIL) |
| `LELAH` | Kirim alarm LELAH ke lora_saenab (ID=4) |
| `TERTIDUR` | Kirim alarm TERTIDUR ke lora_saenab |
| `NORMAL` | Reset alarm lora_saenab |
| `TEST SF7 BW125` | Broadcast START_TEST ke semua node |
| `TEST SF12 BW250` | Contoh: ganti ke SF12, BW250 |
| `HELP` | Daftar semua command |

## WebConfig UI

### Node (http://192.168.4.1/)
- SSID: `LoRa-CFG-[NODE_NAME]` (tanpa password)
- **Mode: View Only (Hanya Lihat)**
- Menampilkan status: ID Node, SF, BW, CR, dan TX Power.
- Konfigurasi dikunci agar konsisten dengan skenario penelitian.

### Gateway (http://192.168.4.1/)
- SSID: `LoRa-Gateway-CFG` (tanpa password)
- **Mode: Full Config**
- Ubah **SF**, **BW**, dan **WiFi profile**
- **Simpan & Reboot**: hanya reboot gateway
- **📡 Broadcast & Reboot Semua**: kirim START_TEST ke semua node + reboot gateway

## Deployment

### Prasyarat
- Arduino IDE atau PlatformIO
- Library: `RadioHead`, `ArduinoJson`, `Adafruit_BNO055`, `TinyGPSPlus`
- Board package: `esp32` (Espressif)

### Langkah Upload
1. Buka folder sesuai target hardware
2. Set board:
   - ESP32-S3: `ESP32S3 Dev Module`
   - ESP32-C3: `ESP32C3 Dev Module`
3. Set `NODE_ID` dan `NODE_NAME` di awal file `.ino` (sudah ter-set)
4. Upload ke target hardware

### WiFi Credentials
```cpp
// Gateway_TA/LoRa_Gateway/secrets.h
#define HAS_WIFI_PROFILE_LIST
static const WiFiProfile WIFI_PROFILES[] = {
    {"YOUR_SSID_1", "YOUR_PASSWORD_1"},
    {"YOUR_SSID_2", "YOUR_PASSWORD_2"}
};
```

## MQTT Topics

| Topic | Sumber | Deskripsi |
|-------|--------|-----------|
| `lora/fms` | Gateway | Sensor data per node |
| `lora/fms/status` | Gateway | QoS summary |
| `lora/fms/debug` | Gateway | Debug messages |
| `lora/fms/fatigue_detection/imu` | Gateway | IMU fatigue data |
| `lora/fms/fatigue_detection/vision` | Gateway ↔ Vision | Alarm control |
| `lora/fms/safety/condition` | Gateway | Safety condition flags |
| `lora/fms/diagnostic/route` | Gateway | Route discovery metrics |

## Catatan Penting

1. **LoRa_Packet.h/.cpp** dan **AODV_Routing.h/.cpp** harus **identik** di semua 6 folder
2. **WebConfig.h** Gateway berbeda dari node (namespace `GWWebConfig` vs `WebConfig`)
3. `nodeID[16]` telah dihapus dari `SensorDataPayload` untuk optimasi airtime
4. `lora_saenab` menggunakan pin yang berbeda dari node lain (ESP32-C3)
5. Gateway beroperasi di Core 1 (LoRa task) dan Core 0 (WiFi/MQTT/WebConfig)

---

**Versi Firmware**: v3.0 (Unified Architecture)  
**Terakhir diupdate**: Mei 2026
#   L o r a - M e s h - A d - h o c - r e t r y  
 
