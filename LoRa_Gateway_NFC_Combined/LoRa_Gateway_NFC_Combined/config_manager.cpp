#include "config_manager.h"

#include <Preferences.h>

ConfigManager configManager;

ConfigManager::ConfigManager() {
  memset(&_config, 0, sizeof(_config));
  _config.debug_all = false;
  _config.debug_nfc = true;
  _config.vehicle_id = "NFC-001";
  _config.device_id = "DEV-001";
  _config.version = "1.0.0-standalone";
}

void ConfigManager::begin() {
  Preferences prefs;
  if (prefs.begin("nfc_cfg", true)) {
    _config.debug_all = prefs.getBool("debug_all", false);
    _config.debug_nfc = prefs.getBool("debug_nfc", true);
    prefs.end();
  }
  printConfig();
}

const FMSConfig &ConfigManager::getConfig() const { return _config; }

void ConfigManager::setDebugAll(bool enable) { _config.debug_all = enable; }

void ConfigManager::setDebugNfc(bool enable) { _config.debug_nfc = enable; }

void ConfigManager::saveDebugFlags() {
  Preferences prefs;
  if (!prefs.begin("nfc_cfg", false)) {
    Serial.println("[CONFIG] Failed to open NVS namespace nfc_cfg");
    return;
  }
  prefs.putBool("debug_all", _config.debug_all);
  prefs.putBool("debug_nfc", _config.debug_nfc);
  prefs.end();
}

String ConfigManager::getDebugFlagsJson() const {
  String json = "{";
  json += "\"all\":";
  json += _config.debug_all ? "true" : "false";
  json += ",\"nfc\":";
  json += _config.debug_nfc ? "true" : "false";
  json += "}";
  return json;
}

void ConfigManager::printConfig() const {
  Serial.println("[CONFIG] NFC standalone config loaded");
  Serial.printf("[CONFIG] DEBUG_ALL=%s, DEBUG_NFC=%s\n",
                _config.debug_all ? "ON" : "OFF",
                _config.debug_nfc ? "ON" : "OFF");
}
