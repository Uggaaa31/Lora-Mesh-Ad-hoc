#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

/**
 * TelemetryData structure used by FMS protocol.
 * Reconstructed from fms_mqtt.cpp usage.
 */
struct TelemetryData {
  const char *vehicle_id;
  const char *device_id;
  uint32_t seq;
  uint32_t timestamp_ms;

  // Datetime
  String datetime_rtc;
  String datetime_gps;
  String datetime_best;
  bool rtc_valid;
  bool gps_time_valid;

  // GPS
  double gps_lat;
  double gps_lon;
  float gps_speed_kph;
  float gps_altitude_m;
  float gps_hdop;
  uint8_t gps_satellites;
  bool gps_valid;

  // Geofence
  int geofence_id;
  const char *geofence_name;

  // Barometer
  float baro_temp_c;
  int box_temp_status;
  float baro_pressure_hpa;
  float baro_alt_abs_m;
  float baro_alt_rel_m;
  float baro_vspeed_ms;
  int baro_motion_state;

  // IMU
  bool imu_ready;
  float imu_accel_x, imu_accel_y, imu_accel_z;
  float imu_gyro_x, imu_gyro_y, imu_gyro_z;
  float imu_mag_x, imu_mag_y, imu_mag_z;
  float imu_pitch, imu_roll, imu_heading;
  float imu_lin_acc_x, imu_lin_acc_y, imu_lin_acc_z;
  uint8_t imu_cal_sys, imu_cal_gyro, imu_cal_accel, imu_cal_mag;
  float imu_accel_rms;

  // Power (INA)
  float ina0_ch1_voltage, ina0_ch1_current;
  float ina0_ch2_voltage, ina0_ch2_current;
  float ina0_ch3_voltage, ina0_ch3_current;
  float ina1_ch1_voltage, ina1_ch1_current;
  float ina1_ch2_voltage, ina1_ch2_current;
  float ina1_ch3_voltage, ina1_ch3_current;

  // Fuel
  float fuel_frequency_hz;
  float fuel_percent;
  float fuel_volume_l;
  float fuel_consumption_l;
  float fuel_rate_lph;
  float fuel_runtime_h;
  float fuel_stolen_l;
  int fuel_anomaly;
  bool fuel_signal_ok;

  // NFC
  String nfc_last_uid;
  uint32_t nfc_last_tap_ms;
  bool nfc_tag_present;

  // Vehicle Status
  bool engine_on;
  bool vehicle_moving;

  // Operator Input
  String lokasi_awal;
  String lokasi_akhir;
  String jenis_muatan;
  String status_trip;

  // Storage
  int sd_status;
  uint32_t sd_free_mb;
};

#endif
