#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>

// Struktur untuk menyimpan kredensial WiFi
struct WiFiCredential {
  const char *ssid;
  const char *pass;
};

class WiFiHelper {
public:
  // Inisialisasi dengan multiple WiFi networks
  void begin(WiFiCredential *networks, uint8_t count) {
    _networks = networks;
    _networkCount = count;

    WiFi.mode(WIFI_AP_STA); WiFi.softAP("LoRa-Gateway-CFG");
    WiFi.persistent(false);

    // Tambahkan semua AP ke objek WiFiMulti
    for (uint8_t i = 0; i < count; i++) {
      _wifiMulti.addAP(networks[i].ssid, networks[i].pass);
      Serial.printf("WiFi: Added AP \"%s\" to scan list\n", networks[i].ssid);
    }

    Serial.println("WiFi: Connecting using WiFiMulti (Strongest Signal)...");
    _wifiMulti.run(); // Mulai percobaan koneksi pertama (non-blocking)
  }

  // Legacy: single network (backward compatible)
  void begin(const char *ssid, const char *pass) {
    _legacyCred.ssid = ssid;
    _legacyCred.pass = pass;
    _networks = &_legacyCred;
    _networkCount = 1;

    WiFi.mode(WIFI_AP_STA); WiFi.softAP("LoRa-Gateway-CFG");
    WiFi.persistent(false);
    _wifiMulti.addAP(ssid, pass);

    Serial.printf("WiFi: Connecting to \"%s\" (WiFiMulti style)...\n", ssid);
    _wifiMulti.run();
  }

  // panggil rutin di loop() (non-blocking)
  void update(unsigned long nowMs) {
    // Saat connected, jangan scan ulang AP karena bisa memicu flapping koneksi
    // dan churn pada socket/TLS.
    static unsigned long lastRun = 0;
    const bool isConnected = (WiFi.status() == WL_CONNECTED);
    if (isConnected) {
      return;
    }

    const unsigned long intervalMs = 2500UL;
    if (nowMs - lastRun >= intervalMs || lastRun == 0) {
      lastRun = nowMs;
      _wifiMulti.run();
    }
  }

  bool connected() const { return WiFi.status() == WL_CONNECTED; }
  IPAddress ip() const { return WiFi.localIP(); }

  // WiFiMulti menangani pemilihan SSID secara internal,
  // kita ambil SSID yang saat ini terkoneksi.
  const char *getCurrentSSID() const {
    if (connected()) {
      static String currentSSID;
      currentSSID = WiFi.SSID();
      return currentSSID.c_str();
    }
    return "Disconnected";
  }

  uint8_t getNetworkCount() const { return _networkCount; }

  void disconnect();

private:
  WiFiMulti _wifiMulti;
  WiFiCredential *_networks = nullptr;
  uint8_t _networkCount = 0;

  // Legacy support
  WiFiCredential _legacyCred;
};
