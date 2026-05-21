# MQTT Dashboard Localhost

Dashboard localhost untuk menerima data MQTT gateway LoRa mesh dan membantu pengujian skripsi skenario 2-5.

## Fitur Utama

- Subscribe ke topic yang sama dengan gateway:
  - `fms/lora/+`
  - `fms/lora/status`
  - `fms/lora/diagnostic/route`
  - `fms/lora/fatigue_detection/imu`
  - `fms/lora/safety/condition`
  - `fms/lora/bno/data`
- Form trial manual: `label`, `SF`, `BW`, `CR`, `hop target`, `jarak`, `durasi trial (menit)`.
- Halaman monitoring dipisah per skenario pengujian skripsi.
- Riwayat semua data MQTT masuk + export CSV per skenario.
- Readiness check: MQTT, aliran paket, route diagnostic, status gateway, sinyal latency.

Catatan:
- Fitur Automation di UI sudah dihapus.
- Endpoint automation di backend masih ada (legacy), tapi tidak dipakai untuk alur UI sekarang.

## Jalankan

```bash
cd mqtt_dashboard_localhost
npm install
npm start
```

Buka:

```text
http://localhost:3001
```

## Konfigurasi `.env` (Opsional)

Salin `.env.example` menjadi `.env`.

Contoh minimal:

```text
PORT=3001
MQTT_URI=wss://mqtt.aistrack.site:443/
MQTT_USERNAME=
MQTT_PASSWORD=
MQTT_REJECT_UNAUTHORIZED=true
NODE_SEND_INTERVAL_MS=3000
MQTT_TOPIC_DATA_PREFIX=fms/lora
MQTT_TOPIC_STATUS=fms/lora/status
MQTT_TOPIC_DIAGNOSTIC=fms/lora/diagnostic/route
MQTT_TOPIC_FATIGUE_IMU=fms/lora/fatigue_detection/imu
MQTT_TOPIC_SAFETY_CONDITION=fms/lora/safety/condition
MQTT_TOPIC_BNO_DATA=fms/lora/bno/data
```

Catatan:
- Jika broker pakai sertifikat self-signed saat uji lokal: `MQTT_REJECT_UNAUTHORIZED=false`.

## Alur Pakai Pengujian

1. Nyalakan gateway dan node.
2. Buka dashboard.
3. Isi form trial: `label`, `SF`, `BW`, `CR`, `hop target`, `jarak`, `durasi trial (menit)`.
4. Klik `Start Trial`.
5. Jalankan pengujian sesuai skenario sampai durasi habis.
6. Trial otomatis berhenti saat durasi tercapai (tetap bisa `Stop Trial` manual bila perlu).
7. Masuk halaman `Riwayat & Export` lalu export data yang dibutuhkan.

Catatan penting:
- `Start Trial` masih wajib untuk merekam metrik skenario 3/4/5.
- Skenario 2 (route diagnostic) tetap bisa tampil walau trial belum start, tetapi metadata trial akan kosong (`-`).

## Kapan Tabel Monitoring Terisi

- Skenario 2 (`Route Discovery`): terisi langsung saat ada event di topic `fms/lora/diagnostic/route`.
- Skenario 3 (`QoS & Latensi Per Node`): terisi dari trial aktif / trial terakhir.
- Skenario 4 (`Pengaruh Parameter LoRa`): terisi dari riwayat trial yang sudah pernah dibuat.
- Skenario 5 (`Jangkauan Multi-hop`): terisi dari riwayat trial yang memiliki nilai jarak/hop.

## Referensi Tabel dan Perhitungan

### 1) Skenario 2: Route Discovery (RREQ/RREP)

Kolom di UI:
- `Waktu`
- `Node`
- `Target`
- `Hop Target`
- `Hop Dilalui`
- `Waktu RREQ`
- `Waktu RREP`
- `Waktu Pembentukan Rute (ms)`
- `Result`

Sumber data:
- Topic MQTT `fms/lora/diagnostic/route`.
- `Hop Target` diambil dari metadata trial aktif (jika ada).

Arti + hitung:
- `Waktu RREQ` = nilai `rreq_at` dari payload diagnostic (epoch low32 ms).
- `Waktu RREP` = nilai `rrep_at` dari payload diagnostic (epoch low32 ms, `0`/`-` jika gagal).
- `Waktu Pembentukan Rute (ms)` = nilai `discovery_ms`.
- `Result` = `SUCCESS` jika `success=true`, selain itu `FAILED`.

### 2) Skenario 3: QoS & Latensi Per Node

Kolom di UI:
- `Node`
- `Retries ke-`
- `PDR %`
- `PLR %`
- `Delay Pengiriman Data (ms)`

Sumber data:
- Event packet (`sensor_data`, `fatigue_imu`, `safety_condition`, `vehicle_telemetry`) selama trial.

Rumus:
- `Expected per node = max(1, floor(durasi_trial_ms / NODE_SEND_INTERVAL_MS))`
- `RX = jumlah paket diterima node pada trial`
- `Retries ke- (estimasi) = max(0, Expected - RX)`
- `PDR % = min(100, RX / Expected * 100)`
- `PLR % = max(0, 100 - PDR)`
- `Delay Pengiriman Data (ms) = rata-rata latency_ms valid`

### 3) Skenario 4: Pengaruh Parameter LoRa (SF/BW/CR)

Kolom di UI:
- `SF`
- `BW`
- `CR`
- `PDR %`
- `PLR %`
- `Delay Pengiriman Data (ms)`

Sumber data:
- Ringkasan tiap trial pada history.

Rumus:
- `PDR % = min(100, total_RX_semua_node / total_Expected_semua_node * 100)`
- `PLR % = max(0, 100 - PDR)`
- `Delay Pengiriman Data (ms)` = rata-rata dari `latencyAvg` semua node yang memiliki latency.

### 4) Skenario 5: Jangkauan Multi-hop

Kolom di UI:
- `Jarak (m)`
- `Jumlah Hop Dilalui`
- `Status Pengiriman`

Sumber data:
- Riwayat trial.

Aturan nilai:
- `Jarak (m)` = input trial.
- `Jumlah Hop Dilalui` = `hopAvg` trial (jika ada), fallback ke `Hop Target`.
- `Status Pengiriman` = `BERHASIL` jika `total RX > 0`, selain itu `GAGAL`.

## Halaman Riwayat & Export

### Tabel `Semua Data MQTT Masuk`

Semua event masuk akan tercatat di sini, baik saat trial aktif maupun tidak aktif:
- data packet
- route diagnostic
- gateway status

Kolom utama sudah disesuaikan dengan metadata input trial di Overview:
- waktu, jenis event, topik MQTT, node
- kode trial, label, SF, BW, CR, hop target, jarak, durasi trial (menit)
- hop dilalui, delay, waktu RREQ, waktu RREP, waktu pembentukan rute, retries RREQ, result, RSSI

Catatan:
- Panel `Riwayat Trial Ringkas` sudah dihapus dari halaman riwayat.

### Tombol Export

Export utama:
- `/api/export/all_data.csv` -> semua event MQTT (`all_data_mqtt.csv`)
- `/api/export/pengujian2_route.csv` -> skenario 2 (`pengujian_2_route_discovery_aodv.csv`)
- `/api/export/pengujian3_qos.csv` -> skenario 3 (`pengujian_3_qos_dan_latensi.csv`)
- `/api/export/pengujian4_parameter.csv` -> skenario 4 (`pengujian_4_pengaruh_parameter_lora.csv`)
- `/api/export/pengujian5_jarak.csv` -> skenario 5 (`pengujian_5_jangkauan_multihop.csv`)

## Format CSV Agar Mudah Dibuka di Excel

- CSV default memakai delimiter `;`.
- File dikirim dengan UTF-8 BOM agar karakter terbaca baik di Excel.
- Saran impor di Excel:
  - pilih delimiter `;`
  - format kolom waktu sebagai Date/Time
  - format kolom numerik (`PDR`, `PLR`, `latency`) sebagai Number

## Mapping Pengujian Skripsi ke Tampilan

- Skenario 2: lihat tabel `Route Discovery (RREQ/RREP)`.
- Skenario 3: lihat tabel `QoS & Latensi Per Node`.
- Skenario 4: lihat tabel `Pengaruh Parameter LoRa`.
- Skenario 5: lihat tabel `Jangkauan Multi-hop`.

## Ukuran Data Per Node (LoRa) dan Contoh JSON MQTT

### Dasar hitung byte

- Ukuran paket LoRa aplikasi = `PacketHeader + payload`.
- `PacketHeader = 13 byte`.
- Total kirim radio aplikasi: `13 + payloadLength`.

### Ringkasan byte uplink

| Node | Jenis paket uplink | Payload struct | Payload (byte) | Header (byte) | Total LoRa (byte) |
|---|---|---|---:|---:|---:|
| `TRK-001` | `PKT_TYPE_DATA` | `SensorDataPayload` | 75 | 13 | **88** |
| `TRK-002` | `PKT_TYPE_DATA` | `SensorDataPayload` | 75 | 13 | **88** |
| `TRK-003` | `PKT_TYPE_DATA` | `SensorDataPayload` | 75 | 13 | **88** |
| `lora_saenab` | `PKT_TYPE_FATIGUE_IMU` | `ImuFatiguePayload` | 21 | 13 | **34** |
| `lora_nailah` | `PKT_TYPE_SAFETY_CONDITION` | `SafetyConditionPayload` | 12 | 13 | **25** |

### Paket route discovery

| Jenis paket | Payload struct | Payload (byte) | Header (byte) | Total LoRa (byte) |
|---|---|---:|---:|---:|
| `PKT_TYPE_DIAGNOSTIC` | `DiscoveryDiagPayload` | 17 | 13 | **30** |

### Contoh JSON MQTT

1) Contoh `TRK-001/002/003` di topic `fms/lora/<nodeName>`

```json
{
  "nodeId": "TRK-002",
  "timestamp": 1715512345,
  "hopCount": 2,
  "rssi": -82,
  "latency_ms": 148,
  "gps": {
    "latitude": -6.20123,
    "longitude": 106.81612,
    "altitude": 25.4
  },
  "imu": {
    "accel": {
      "x": 0.12,
      "y": -0.03,
      "z": 9.76
    }
  },
  "battery": 3.92,
  "snr": 7,
  "route_disc_ms": 412,
  "route_hops": 2
}
```

2) Contoh `lora_saenab` di topic `fms/lora/fatigue_detection/imu`

```json
{
  "nodeId": 4,
  "ts": 1715512345123,
  "pitch": 7.52,
  "roll": -1.24,
  "f_gx": 0.013,
  "f_gy": -0.008,
  "f_gz": 0.003,
  "hopCount": 2,
  "rssi": -79,
  "snr": 8,
  "route_disc_ms": 389,
  "route_hops": 2,
  "latency_ms": 133
}
```

3) Contoh `lora_nailah` di topic `fms/lora/safety/condition`

```json
{
  "nodeId": 5,
  "ts": 1715512345987,
  "gpsValid": true,
  "drActive": false,
  "rolloverRisk": false,
  "rollover": false,
  "harshBraking": false,
  "overspeed": true,
  "flags": 33,
  "hopCount": 3,
  "rssi": -85,
  "snr": 6,
  "route_disc_ms": 520,
  "route_hops": 3,
  "latency_ms": 176
}
```

4) Contoh route diagnostic di topic `fms/lora/diagnostic/route`

```json
{
  "node_asal": 5,
  "node_name": "lora_nailah",
  "target_rute": 0,
  "rreq_at": 1715512344000,
  "rrep_at": 1715512344520,
  "discovery_ms": 520,
  "hops": 3,
  "retries": 1,
  "success": true,
  "rssi": -87
}
```
