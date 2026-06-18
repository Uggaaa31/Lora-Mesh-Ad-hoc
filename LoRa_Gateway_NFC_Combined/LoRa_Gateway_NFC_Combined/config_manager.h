#ifndef NFC_CONFIG_MANAGER_H
#define NFC_CONFIG_MANAGER_H

#include <Arduino.h>

struct FMSConfig {
  bool debug_all;
  bool debug_nfc;

  // Added members for MQTT and system config
  const char *vehicle_id;
  const char *device_id;
  const char *version;

  float fuel_freq_empty;
  float fuel_freq_full;
  float fuel_tank_capacity;

  float engine_on_voltage;
  float moving_speed_kph;
  float stopped_speed_kph;
  float moving_accel_g;
  float stopped_accel_g;

  float siphon_drop_pct;
  float slow_theft_5m_pct;
  float slow_theft_10m_pct;
  float sudden_drop_pct;
  float excessive_drop_pct;

  uint32_t data_send_interval_ms;
  uint16_t log_batch_size;
  uint32_t log_send_timeout_ms;
};

class ConfigManager {
public:
  ConfigManager();

  void begin();
  const FMSConfig &getConfig() const;

  void setDebugAll(bool enable);
  void setDebugNfc(bool enable);
  void saveDebugFlags();
  String getDebugFlagsJson() const;
  void printConfig() const;

private:
  FMSConfig _config;
};

extern ConfigManager configManager;

#endif // NFC_CONFIG_MANAGER_H
