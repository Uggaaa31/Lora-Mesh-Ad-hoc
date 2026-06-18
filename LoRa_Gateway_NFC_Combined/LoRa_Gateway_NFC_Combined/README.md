# LoRa Gateway + NFC Combined

Folder ini adalah hasil penggabungan firmware `LoRa_Gateway` dan `nfc`.
Folder asli tidak diubah.

## Isi yang dipertahankan

- Seluruh flow LoRa gateway: LoRa RX, AODV routing, QoS stats, CSV output, WebConfig, WiFi, MQTT gateway, NTP sync, dan Serial CLI gateway.
- Seluruh flow NFC: PN532 handler, buzzer, RGB LED status, valid/cancel event, recovery logic, debug flag, dan publish event NFC ke broker MQTT NFC.

## Cara kerja hasil merge

- Firmware utama tetap berbasis gateway LoRa.
- Modul NFC dipisah ke `nfc_integration.cpp/.h` lalu dipanggil dari sketch utama.
- WiFi hanya dikelola oleh gateway.
- Modul NFC sekarang memakai koneksi MQTT gateway yang sama.
- Pengaturan broker dan topic MQTT dipusatkan di `config.h`.

## Pin yang dipakai

- LoRa SCK: `GPIO 42`
- LoRa MOSI: `GPIO 41`
- LoRa MISO: `GPIO 40`
- LoRa CS: `GPIO 5`
- LoRa RST: `GPIO 7`
- LoRa DIO0: `GPIO 6`
- PN532 RST: `GPIO 4`
- PN532 SDA: `GPIO 1`
- PN532 SCL: `GPIO 2`
- Buzzer: `GPIO 13`
- RGB LED: `GPIO 48`

## Perintah Serial

- Gateway:
  - `HELP`
  - `STATUS`
  - `STATS`
  - `CSV`
  - `RESET`
  - `SYNC`
  - `ROUTE`
- NFC:
  - `STATUS NFC`
  - `UID`
  - `RESET NFC`
  - `DEBUG NFC`
  - `DEBUG ALL`
  - `DEBUG STATUS`
  - `RGB ON`
  - `RGB OFF`
  - `N`
  - `V`
  - `F`
  - `D`
  - `U`
  - `S`

## Library yang tetap dibutuhkan

- ESP32 board core untuk ESP32-S3
- RadioHead (`RH_RF95`)
- PN532 library yang menyediakan `PN532.h` dan `PN532_I2C.h`
- ArduinoJson

## Catatan

- File `secrets.h` tetap dipakai untuk pilihan WiFi gateway.
- Event NFC sekarang mengikuti koneksi WiFi dan MQTT gateway yang sama, bukan koneksi terpisah seperti sketch NFC standalone.
