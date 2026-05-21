/*
 * WebConfig.h - Konfigurasi LoRa via WiFi AP + Web Browser
 * Mode    : WiFi Access Point selalu aktif selama node menyala
 * SSID    : "LoRa-CFG-[NODE_NAME]" tanpa password
 * URL     : http://192.168.4.1/
 * Simpan  : hanya SF dan BW yang diubah, parameter lain tetap dari config.h
 */
#pragma once
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// Struct runtime LoRa config (dibaca dari NVRAM atau default config.h)
struct LoRaRuntimeCfg {
    uint8_t  sf      = 7;     // Spreading Factor 7-12
    uint32_t bwKHz   = 125;   // Bandwidth: 125, 250 kHz
    uint8_t  cr      = 5;     // Coding Rate tetap dari config.h
    int8_t   txPower = 20;    // TX Power tetap dari config.h
    bool     useRFO  = false; // Path TX tetap dari config.h
};

static const char _WC_HTML[] PROGMEM = R"==(
<!DOCTYPE html><html lang="id"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LoRa Node Status</title><style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0f0f1a;color:#e2e8f0;font-family:'Segoe UI',sans-serif;
  min-height:100vh;display:flex;align-items:center;justify-content:center;padding:16px}
.card{background:linear-gradient(145deg,#1a1a2e,#16213e);border:1px solid #2d4a7a;
  border-radius:16px;padding:24px;width:100%;max-width:420px;
  box-shadow:0 20px 60px rgba(0,0,0,.5)}
h1{text-align:center;font-size:1.3em;margin-bottom:4px;
  background:linear-gradient(135deg,#667eea,#764ba2);
  -webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{text-align:center;color:#718096;font-size:.82em;margin-bottom:16px}
.badge{display:inline-block;padding:4px 12px;background:rgba(102,126,234,0.2);
  border:1px solid #667eea;color:#667eea;border-radius:20px;font-size:0.7em;
  font-weight:600;margin-bottom:15px;text-transform:uppercase;letter-spacing:1px}
.inf{background:rgba(102,126,234,.12);border:1px solid rgba(102,126,234,.3);
  border-radius:8px;padding:15px;margin-bottom:16px;font-size:.9em;line-height:2.2}
.inf b{color:#667eea;display:inline-block;width:100px}
.tmr{background:#1a1a2e;border-radius:8px;padding:10px;text-align:center;
  margin-top:14px;font-size:.8em;color:#90cdf4;border:1px dashed #2d4a7a}
</style></head><body><div class="card" style="text-align:center">
<div class="badge">View Only Mode</div>
<h1>LoRa Node Status</h1>
<div class="sub">%%NODE%%</div>
<div class="inf" style="text-align:left">
  <b>ID Node</b> : %%NODE_ID%%<br>
  <b>SF</b> : SF%%SF%%<br>
  <b>Bandwidth</b> : %%BW%% kHz<br>
  <b>Coding Rate</b> : 4/%%CR%%<br>
  <b>TX Power</b> : %%PWR%% dBm<br>
</div>
<div class="tmr">Konfigurasi dikunci. Gunakan fitur <b>START_TEST</b> di Gateway untuk mengubah parameter secara massal.</div>
</div></body></html>
)==";

static const char _WC_SAVED[] PROGMEM = R"==(
<!DOCTYPE html><html><head><meta charset="UTF-8">
<style>body{background:#0f0f1a;color:#68d391;font-family:sans-serif;
display:flex;align-items:center;justify-content:center;height:100vh;text-align:center}
.box{padding:32px}</style></head><body><div class="box">
<div style="font-size:3em">OK</div>
<h2 style="margin:12px 0">Config Tersimpan</h2>
<p style="color:#a0aec0">ESP32 reboot dalam 2 detik.<br>
Reconnect ke WiFi node setelah 10 detik.</p>
</div></body></html>
)==";

namespace WebConfig {

static WebServer*     _srv    = nullptr;
static Preferences    _prefs;
static LoRaRuntimeCfg _cfg;
static uint8_t        _myNodeID = 0;
static bool           _active = false;
static bool           _reboot = false;
static String         _nodeName;

// Baca config dari NVRAM. Hanya SF dan BW yang dipakai sebagai runtime override.
static bool _load(LoRaRuntimeCfg& out) {
    _prefs.begin("lora-cfg", true);
    bool has = _prefs.isKey("sf") || _prefs.isKey("bw");
    if (has) {
        out.sf    = _prefs.getUChar("sf", out.sf);
        out.bwKHz = _prefs.getUInt("bw", out.bwKHz);
    }
    _prefs.end();
    return has;
}

// Ambil config runtime (NVRAM > default config.h) untuk SF/BW.
// CR, TX power, dan path TX tetap mengikuti config.h.
LoRaRuntimeCfg getConfig(uint8_t defSF, uint32_t defBW, uint8_t defCR,
                         int8_t defPwr, bool defRFO) {
    LoRaRuntimeCfg cfg = {defSF, defBW / 1000, defCR, defPwr, defRFO};
    if (_load(cfg)) {
        Serial.printf("[WebConfig] NVRAM: SF=%d BW=%dkHz | CR tetap 4/%d | Pwr tetap %ddBm\n",
                      cfg.sf, cfg.bwKHz, cfg.cr, cfg.txPower);
    } else {
        Serial.println("[WebConfig] Default config.h digunakan.");
    }
    return cfg;
}

// Reset config NVRAM ke default
void resetConfig() {
    _prefs.begin("lora-cfg", false);
    _prefs.clear();
    _prefs.end();
    Serial.println("[WebConfig] NVRAM direset ke default.");
}

// Simpan SF/BW dari START_TEST broadcast ke NVRAM (dipanggil sebelum reboot)
void saveTestConfig(uint8_t sf, uint32_t bwKHz) {
    sf = constrain(sf, 7, 12);
    bwKHz = (bwKHz == 250) ? 250 : 125;
    _prefs.begin("lora-cfg", false);
    _prefs.putUChar("sf", sf);
    _prefs.putUInt("bw", bwKHz);
    _prefs.end();
    Serial.printf("[WebConfig] START_TEST saved: SF=%d BW=%lukHz\n", sf, (unsigned long)bwKHz);
}

static String _buildPage() {
    String s = String(_WC_HTML);
    s.replace("%%NODE%%", _nodeName);
    s.replace("%%NODE_ID%%", String(_myNodeID));
    s.replace("%%SF%%", String(_cfg.sf));
    s.replace("%%BW%%", String(_cfg.bwKHz));
    s.replace("%%CR%%", String(_cfg.cr));
    while (s.indexOf("%%PWR%%") != -1) s.replace("%%PWR%%", String(_cfg.txPower));
    return s;
}

static void _onRoot() {
    _srv->send(200, "text/html; charset=UTF-8", _buildPage());
}

// Handler /save dinonaktifkan di mode View-Only
static void _onSave() {
    _srv->send(403, "text/plain", "Konfigurasi dikunci. Gunakan Gateway.");
}

// Mulai WiFi AP + Web Server
void begin(const char* nodeNameStr, const LoRaRuntimeCfg& currentCfg, uint8_t id) {
    _nodeName = String(nodeNameStr);
    _cfg = currentCfg;
    _myNodeID = id;
    _reboot = false;

    String ssid = "LoRa-CFG-" + _nodeName;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid.c_str());
    delay(300);
    IPAddress ip = WiFi.softAPIP();

    Serial.println("\n[WebConfig] WiFi Config Mode AKTIF");
    Serial.printf("[WebConfig] SSID : %s\n", ssid.c_str());
    Serial.printf("[WebConfig] URL  : http://%s/\n", ip.toString().c_str());
    Serial.println("[WebConfig] Mode : tetap aktif selama node menyala, tanpa password.\n");

    _srv = new WebServer(80);
    _srv->on("/", HTTP_GET, _onRoot);
    _srv->on("/save", HTTP_POST, _onSave);
    _srv->onNotFound([]() {
        WebConfig::_srv->sendHeader("Location", "/");
        WebConfig::_srv->send(302, "text/plain", "");
    });
    _srv->begin();

    _active = true;
}

// Panggil setiap loop()
void handle() {
    if (!_active || _srv == nullptr) return;
    _srv->handleClient();
    if (_reboot) {
        delay(2000);
        ESP.restart();
    }
}

bool isActive() { return _active; }

} // namespace WebConfig
