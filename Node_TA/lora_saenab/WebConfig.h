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
<title>LoRa Config</title><style>
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
.inf{background:rgba(102,126,234,.12);border:1px solid rgba(102,126,234,.3);
  border-radius:8px;padding:11px;margin-bottom:16px;font-size:.82em;line-height:2}
.inf b{color:#667eea}
.fld{margin-bottom:13px}
label{display:block;color:#a0aec0;font-size:.78em;margin-bottom:5px;
  text-transform:uppercase;letter-spacing:.07em}
select{width:100%;padding:9px 12px;background:#0f1a33;color:#e2e8f0;
  border:1px solid #2d4a7a;border-radius:8px;font-size:.9em;outline:none}
select:focus{border-color:#667eea}
.note{color:#4a5568;font-size:.74em;margin-top:3px}
.btn{width:100%;padding:13px;
  background:linear-gradient(135deg,#667eea,#764ba2);
  color:#fff;border:none;border-radius:10px;font-size:.96em;
  font-weight:600;cursor:pointer;margin-top:8px;letter-spacing:.03em}
.btn:active{opacity:.8}
.tmr{background:#1a1a2e;border-radius:8px;padding:8px;text-align:center;
  margin-top:14px;font-size:.8em;color:#90cdf4}
</style></head><body><div class="card">
<h1>LoRa Parameter Config</h1>
<div class="sub">%%NODE%% - Ubah SF / BW saja untuk pengujian</div>
<div class="inf">
  <b>SF</b> %%SF%% &nbsp;<b>BW</b> %%BW%% kHz &nbsp;
  <b>CR</b> tetap 4/%%CR%% &nbsp;<b>PWR</b> tetap %%PWR%% dBm
</div>
<form action="/save" method="POST">
<div class="fld">
  <label>Spreading Factor (SF)</label>
  <select name="sf">
    <option value="7" %%S7%%>SF7 - Rate tinggi, jarak dekat</option>
    <option value="8" %%S8%%>SF8</option>
    <option value="9" %%S9%%>SF9 - Seimbang</option>
    <option value="10" %%S10%%>SF10</option>
    <option value="11" %%S11%%>SF11</option>
    <option value="12" %%S12%%>SF12 - Jarak jauh, rate rendah</option>
  </select>
  <div class="note">Skenario pengujian: pilih SF sesuai variasi yang diuji.</div>
</div>
<div class="fld">
  <label>Bandwidth (BW)</label>
  <select name="bw">
    <option value="125" %%B125%%>125 kHz - Jangkauan maksimal</option>
    <option value="250" %%B250%%>250 kHz - Seimbang</option>
  </select>
</div>
<button type="submit" class="btn">Simpan &amp; Reboot ESP32</button>
</form>
<div class="tmr">SSID config aktif terus dan bisa diakses tanpa password.</div>
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

static String _buildPage() {
    String s = String(_WC_HTML);
    s.replace("%%NODE%%", _nodeName);
    s.replace("%%SF%%", String(_cfg.sf));
    s.replace("%%BW%%", String(_cfg.bwKHz));
    s.replace("%%CR%%", String(_cfg.cr));
    while (s.indexOf("%%PWR%%") != -1) s.replace("%%PWR%%", String(_cfg.txPower));

    for (int i = 7; i <= 12; i++) {
        s.replace("%%S" + String(i) + "%%", _cfg.sf == i ? "selected" : "");
    }
    s.replace("%%B125%%", _cfg.bwKHz == 125 ? "selected" : "");
    s.replace("%%B250%%", _cfg.bwKHz == 250 ? "selected" : "");
    return s;
}

static void _onRoot() {
    _srv->send(200, "text/html; charset=UTF-8", _buildPage());
}

static void _onSave() {
    if (_srv->hasArg("sf")) _cfg.sf = _srv->arg("sf").toInt();
    if (_srv->hasArg("bw")) _cfg.bwKHz = _srv->arg("bw").toInt();

    _cfg.sf = constrain(_cfg.sf, 7, 12);
    _cfg.bwKHz = (_cfg.bwKHz == 250) ? 250 : 125;

    _prefs.begin("lora-cfg", false);
    _prefs.putUChar("sf", _cfg.sf);
    _prefs.putUInt("bw", _cfg.bwKHz);
    _prefs.remove("cr");
    _prefs.remove("txpwr");
    _prefs.remove("ufo");
    _prefs.end();

    Serial.printf("[WebConfig] TERSIMPAN: SF=%d BW=%dkHz | CR tetap 4/%d | Pwr tetap %ddBm\n",
                  _cfg.sf, _cfg.bwKHz, _cfg.cr, _cfg.txPower);
    _srv->send(200, "text/html; charset=UTF-8", _WC_SAVED);
    _reboot = true;
}

// Mulai WiFi AP + Web Server
void begin(const char* nodeNameStr, const LoRaRuntimeCfg& currentCfg) {
    _nodeName = String(nodeNameStr);
    _cfg = currentCfg;
    _reboot = false;

    String ssid = "LoRa-CFG-" + _nodeName;
    
    Serial.println("[WebConfig] Memulai WiFi AP...");
    
    // 1. Bersihkan sisa WiFi untuk stabilitas
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
    
    // 2. Set mode ke Access Point
    WiFi.mode(WIFI_AP);
    
    // 3. Konfigurasi IP Statis
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    
    // 4. Jalankan SoftAP di Channel 6 (Max 2 koneksi saja agar ringan)
    if (WiFi.softAP(ssid.c_str(), NULL, 6, 0, 2)) {
        IPAddress ip = WiFi.softAPIP();
        Serial.println("[WebConfig] SUCCESS! WiFi Config Mode AKTIF");
        Serial.printf("[WebConfig] SSID : %s\n", ssid.c_str());
        Serial.printf("[WebConfig] URL  : http://%s/\n", ip.toString().c_str());
    } else {
        Serial.println("[WebConfig] FATAL ERROR: WiFi SoftAP failed to start.");
    }

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
