/*
 * WebConfig.h - Konfigurasi LoRa Gateway via Web Browser
 * Mode : WiFi AP+STA
 * Port : 80
 * URL  : http://192.168.4.1/ atau http://[ip-gateway]/
 * Akses dari SSID gateway config tanpa password atau dari router yang sama
 *
 * Catatan:
 * - Parameter LoRa yang dapat diubah hanya SF dan BW.
 * - SSID/password WiFi internet diatur di secrets.h.
 * - Koneksi WiFi dipilih otomatis dari profile aktif (auto-scan).
 * - SSID config gateway bernama LoRa-Gateway-CFG tanpa password.
 * - Gateway tetap bisa auto-scan SSID aktif dari daftar profile di secrets.h.
 */
#pragma once
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

struct LoRaRuntimeCfg {
    uint8_t  sf              = 7;
    uint32_t bwKHz           = 125;
    uint8_t  cr              = 5;
    int8_t   txPower         = 20;
    bool     useRFO          = false;
    uint8_t  wifiProfileIdx  = 0;
};

static const char _GW_HTML[] PROGMEM = R"==(
<!DOCTYPE html><html lang="id"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Gateway LoRa Config</title><style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0f0f1a;color:#e2e8f0;font-family:'Segoe UI',sans-serif;
  min-height:100vh;display:flex;align-items:center;justify-content:center;padding:16px}
.card{background:linear-gradient(145deg,#1a1a2e,#16213e);border:1px solid #2d4a7a;
  border-radius:16px;padding:24px;width:100%;max-width:460px;
  box-shadow:0 20px 60px rgba(0,0,0,.5)}
h1{text-align:center;font-size:1.3em;margin-bottom:4px;
  background:linear-gradient(135deg,#f093fb,#f5576c);
  -webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{text-align:center;color:#718096;font-size:.82em;margin-bottom:16px}
.inf{background:rgba(240,147,251,.1);border:1px solid rgba(240,147,251,.3);
  border-radius:8px;padding:11px;margin-bottom:16px;font-size:.82em;line-height:1.8}
.inf b{color:#f093fb}
.fld{margin-bottom:13px}
label{display:block;color:#a0aec0;font-size:.78em;margin-bottom:5px;
  text-transform:uppercase;letter-spacing:.07em}
select{width:100%;padding:10px 12px;background:#0f1a33;color:#e2e8f0;
  border:1px solid #2d4a7a;border-radius:8px;font-size:.9em;outline:none}
select:focus{border-color:#f093fb}
.note{color:#4a5568;font-size:.74em;margin-top:3px}
.warn{background:rgba(252,129,74,.1);border:1px solid rgba(252,129,74,.4);
  border-radius:8px;padding:10px;margin-bottom:14px;font-size:.8em;color:#fc814a}
.btn{width:100%;padding:13px;
  background:linear-gradient(135deg,#f093fb,#f5576c);
  color:#fff;border:none;border-radius:10px;font-size:.96em;
  font-weight:600;cursor:pointer;margin-top:8px}
.btn:active{opacity:.8}
.sep{border:0;border-top:1px solid #2d4a7a;margin:18px 0}
.btn-test{background:linear-gradient(135deg,#48bb78,#38a169);
  width:100%;padding:13px;color:#fff;border:none;border-radius:10px;
  font-size:.96em;font-weight:600;cursor:pointer;margin-top:8px}
.btn-test:active{opacity:.8}
.test-info{background:rgba(72,187,120,.1);border:1px solid rgba(72,187,120,.3);
  border-radius:8px;padding:10px;margin-top:12px;font-size:.78em;color:#68d391}
</style></head><body><div class="card">
<h1>Gateway LoRa Config</h1>
<div class="sub">GATEWAY (ID=0) - Hanya SF/BW yang bisa diubah</div>
<div class="inf">
  <b>SF</b> %%SF%% &nbsp;<b>BW</b> %%BW%% kHz &nbsp;
  <b>CR</b> tetap 4/%%CR%% &nbsp;<b>PWR</b> tetap %%PWR%% dBm<br>
  <b>WiFi aktif</b> %%WIFI_SSID%%<br>
  <b>SSID Config</b> %%CFG_SSID%% (tanpa password)
</div>
<div class="warn">Masuk ke halaman ini lewat SSID config gateway tanpa password. Simpan akan mereboot Gateway.</div>
<form action="/save" method="POST">
<div class="fld"><label>Spreading Factor (SF)</label>
<select name="sf">
<option value="7" %%S7%%>SF7</option>
<option value="8" %%S8%%>SF8</option>
<option value="9" %%S9%%>SF9</option>
<option value="10" %%S10%%>SF10</option>
<option value="11" %%S11%%>SF11</option>
<option value="12" %%S12%%>SF12</option>
</select><div class="note">Harus sama dengan semua node.</div>
</div>
<div class="fld"><label>Bandwidth (BW)</label>
<select name="bw">
<option value="125" %%B125%%>125 kHz</option>
<option value="250" %%B250%%>250 kHz</option>
</select></div>
<button type="submit" class="btn">Simpan &amp; Reboot Gateway</button>
</form>
<hr class="sep">
<form action="/start_test" method="POST" id="frmTest">
<input type="hidden" name="sf" id="test_sf" value="%%SF%%">
<input type="hidden" name="bw" id="test_bw" value="%%BW%%">
<div class="fld"><label>&#x1F4E1; Broadcast START_TEST ke Semua Node</label>
<div class="note">Kirim parameter SF/BW terpilih di atas ke semua node via LoRa. Semua node (termasuk Gateway) akan reboot dengan config baru.</div>
</div>
<button type="submit" class="btn-test">&#x26A1; Broadcast &amp; Reboot Semua</button>
<div class="test-info">Gateway broadcast 2x untuk reliability, lalu reboot dalam 3 detik.</div>
</form>
<script>
// Sync SF/BW dropdown ke hidden fields form start_test
var sfSel=document.querySelector('select[name="sf"]');
var bwSel=document.querySelector('select[name="bw"]');
if(sfSel){sfSel.addEventListener('change',function(){document.getElementById('test_sf').value=this.value});}
if(bwSel){bwSel.addEventListener('change',function(){document.getElementById('test_bw').value=this.value});}
</script>
</div></body></html>
)==";

static const char _GW_SAVED[] PROGMEM = R"==(
<!DOCTYPE html><html><head><meta charset="UTF-8">
<style>body{background:#0f0f1a;color:#68d391;font-family:sans-serif;
display:flex;align-items:center;justify-content:center;height:100vh;text-align:center}</style>
</head><body><div><div style="font-size:3em">OK</div>
<h2 style="margin:12px 0">Config Tersimpan</h2>
<p style="color:#a0aec0">Gateway reboot dalam 2 detik.</p></div></body></html>
)==";

namespace GWWebConfig {

static WebServer*     _srv     = nullptr;
static Preferences    _prefs;
static LoRaRuntimeCfg _cfg;
static bool           _started = false;
static bool           _reboot  = false;

// Callback untuk broadcast START_TEST packet via LoRa
typedef void (*StartTestBroadcastFn)(uint8_t sf, uint32_t bwKHz);
static StartTestBroadcastFn _onStartTest = nullptr;
void saveTestConfig(uint8_t sf, uint32_t bwKHz); // Forward declaration

void setStartTestCallback(StartTestBroadcastFn fn) {
    _onStartTest = fn;
}

static uint8_t _sanitizeProfileIndex(int idx) {
    if (idx < 0 || idx >= (int)WIFI_PROFILE_COUNT) return 0;
    return (uint8_t)idx;
}

static String _htmlEscape(const String& in) {
    String out = in;
    out.replace("&", "&amp;");
    out.replace("\"", "&quot;");
    out.replace("'", "&#39;");
    out.replace("<", "&lt;");
    out.replace(">", "&gt;");
    return out;
}

uint8_t getWiFiProfileIndex(const LoRaRuntimeCfg& cfg) {
    return _sanitizeProfileIndex(cfg.wifiProfileIdx);
}

const WiFiProfile& getWiFiProfile(const LoRaRuntimeCfg& cfg) {
    return WIFI_PROFILES[getWiFiProfileIndex(cfg)];
}

const char* getWiFiSSID(const LoRaRuntimeCfg& cfg) {
    return getWiFiProfile(cfg).ssid;
}

const char* getWiFiPassword(const LoRaRuntimeCfg& cfg) {
    return getWiFiProfile(cfg).password;
}

static bool _load(LoRaRuntimeCfg& out) {
    _prefs.begin("lora-cfg", true);
    bool has = _prefs.isKey("sf") || _prefs.isKey("bw");
    if (has) {
        out.sf             = _prefs.getUChar("sf", out.sf);
        out.bwKHz          = _prefs.getUInt("bw", out.bwKHz);
    }
    _prefs.end();
    return has;
}

LoRaRuntimeCfg getConfig(uint8_t defSF, uint32_t defBW, uint8_t defCR,
                         int8_t defPwr, bool defRFO, uint8_t defProfileIdx = 0) {
    LoRaRuntimeCfg cfg = {
        defSF,
        defBW / 1000,
        defCR,
        defPwr,
        defRFO,
        _sanitizeProfileIndex(defProfileIdx)
    };

    if (_load(cfg)) {
        Serial.printf("[GWConfig] NVRAM: SF=%d BW=%dkHz | WiFi=%s\n",
                      cfg.sf, cfg.bwKHz, getWiFiSSID(cfg));
    } else {
        Serial.println("[GWConfig] Default config.h / secrets.h digunakan.");
    }
    return cfg;
}

static String _build() {
    String s = String(_GW_HTML);
    s.replace("%%SF%%", String(_cfg.sf));
    s.replace("%%BW%%", String(_cfg.bwKHz));
    s.replace("%%CR%%", String(_cfg.cr));
    s.replace("%%WIFI_SSID%%", _htmlEscape(String(getWiFiSSID(_cfg))));
    s.replace("%%CFG_SSID%%", _htmlEscape(String(GATEWAY_CONFIG_SSID)));
    while (s.indexOf("%%PWR%%") != -1) s.replace("%%PWR%%", String(_cfg.txPower));

    for (int i = 7; i <= 12; ++i) {
        s.replace("%%S" + String(i) + "%%", _cfg.sf == i ? "selected" : "");
    }
    s.replace("%%B125%%", _cfg.bwKHz == 125 ? "selected" : "");
    s.replace("%%B250%%", _cfg.bwKHz == 250 ? "selected" : "");
    return s;
}

static void _onConfig() {
    _srv->send(200, "text/html; charset=UTF-8", _build());
}

static void _onConfigSave() {
    if (_srv->hasArg("sf")) _cfg.sf = _srv->arg("sf").toInt();
    if (_srv->hasArg("bw")) _cfg.bwKHz = _srv->arg("bw").toInt();

    _cfg.sf = constrain(_cfg.sf, 7, 12);
    _cfg.bwKHz = (_cfg.bwKHz == 250) ? 250 : 125;

    _prefs.begin("lora-cfg", false);
    _prefs.putUChar("sf", _cfg.sf);
    _prefs.putUInt("bw", _cfg.bwKHz);
    _prefs.remove("wprof");
    _prefs.remove("cr");
    _prefs.remove("txpwr");
    _prefs.remove("ufo");
    _prefs.remove("wssid");
    _prefs.remove("wpass");
    _prefs.end();

    Serial.printf("[GWConfig] TERSIMPAN: SF=%d BW=%dkHz | WiFi=%s\n",
                  _cfg.sf, _cfg.bwKHz, getWiFiSSID(_cfg));
    _srv->send(200, "text/html; charset=UTF-8", _GW_SAVED);
    _reboot = true;
}

void begin(const String& stationIP, const String& apIP, const LoRaRuntimeCfg& currentCfg) {
    _cfg = currentCfg;
    _reboot = false;
    _started = true;

    _srv = new WebServer(80);
    _srv->on("/", HTTP_GET, _onConfig);
    _srv->on("/save", HTTP_POST, _onConfigSave);
    _srv->on("/start_test", HTTP_POST, []() {
        // Ambil SF/BW dari form (sama dengan yang ditampilkan di halaman)
        uint8_t sf = _cfg.sf;
        uint32_t bw = _cfg.bwKHz;
        if (_srv->hasArg("sf")) sf = _srv->arg("sf").toInt();
        if (_srv->hasArg("bw")) bw = _srv->arg("bw").toInt();
        // Tapi karena form START_TEST tidak punya field sendiri,
        // gunakan config yang sudah tersimpan di _cfg
        sf = constrain(sf, 7, 12);
        bw = (bw == 250) ? 250 : 125;

        Serial.printf("[GWConfig] START_TEST broadcast: SF=%d BW=%lukHz\n", sf, (unsigned long)bw);

        if (_onStartTest) {
            _onStartTest(sf, bw);
        }

        _srv->send(200, "text/html; charset=UTF-8",
            "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
            "<style>body{background:#0f0f1a;color:#68d391;font-family:sans-serif;"
            "display:flex;align-items:center;justify-content:center;height:100vh;text-align:center}"
            "</style></head><body><div><div style='font-size:3em'>&#x1F4E1;</div>"
            "<h2 style='margin:12px 0'>START_TEST Broadcast Terkirim</h2>"
            "<p style='color:#a0aec0'>Semua node + gateway reboot dalam 3 detik.</p>"
            "</div></body></html>");

        // Simpan & reboot gateway juga
        saveTestConfig(sf, bw);
        delay(3000);
        ESP.restart();
    });
    _srv->onNotFound([]() {
        GWWebConfig::_srv->sendHeader("Location", "/");
        GWWebConfig::_srv->send(302, "", "");
    });
    _srv->begin();

    Serial.println("\n[GWConfig] Web Config Server aktif (port 80)");
    Serial.printf("[GWConfig] SSID Config: %s (tanpa password)\n", GATEWAY_CONFIG_SSID);
    Serial.printf("[GWConfig] URL AP : http://%s/\n", apIP.c_str());
    if (stationIP != "0.0.0.0") {
        Serial.printf("[GWConfig] URL STA: http://%s/\n", stationIP.c_str());
    }
}

void handle() {
    if (!_started || _srv == nullptr) return;
    _srv->handleClient();
    if (_reboot) {
        delay(2000);
        ESP.restart();
    }
}

bool isStarted() { return _started; }

// Simpan SF/BW dari START_TEST ke NVRAM (dipanggil sebelum reboot)
void saveTestConfig(uint8_t sf, uint32_t bwKHz) {
    sf = constrain(sf, 7, 12);
    bwKHz = (bwKHz == 250) ? 250 : 125;
    _prefs.begin("lora-cfg", false);
    _prefs.putUChar("sf", sf);
    _prefs.putUInt("bw", bwKHz);
    _prefs.end();
    Serial.printf("[GWConfig] START_TEST saved: SF=%d BW=%lukHz\n", sf, (unsigned long)bwKHz);
}

} // namespace GWWebConfig
