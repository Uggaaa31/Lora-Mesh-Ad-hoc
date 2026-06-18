#ifndef PN532_HANDLER_H
#define PN532_HANDLER_H

#include <Arduino.h>
#include <PN532.h>
#include <PN532_I2C.h>
#include <Wire.h>

// Definisi state machine PN532
enum NfcState { NFC_STATE_SCAN, NFC_STATE_HOLD, NFC_STATE_TAG_PRESENT };

class PN532Handler {
public:
  static const uint8_t MAX_UID_BYTES = 10; // Mifare/NFC UID max (4/7/10 byte)

  // Konfigurasi timing (ms) - OPTIMIZED untuk responsif
  static const unsigned long POLL_INTERVAL_MS =
      30; // Scan lebih sering (was 100)
  static const unsigned long READ_TIMEOUT_MS =
      20; // Timeout lebih cepat dan kurangi blocking buzzer/UI
  static const unsigned long MAX_HOLD_MS =
      5000; // Maksimal hold sebelum soft recovery
  static const unsigned long REMOVE_TIMEOUT_MS =
      300; // Deteksi kartu dilepas lebih cepat (was 1200ms)
  static const unsigned long SOFT_RECOVERY_COOLDOWN =
      1000; // Kurangi dari 2000ms
  static const unsigned long HEARTBEAT_INTERVAL =
      1000; // Cek tiap 1 detik (was 5000)
  static const unsigned long PN532_TIMEOUT_MS =
      3000; // 3 detik tanpa I2C = hard reset (was 10000)
  static const unsigned long RESET_COOLDOWN_MS =
      2000; // Jeda antar reset (was 5000)
  static const unsigned long RESET_LOW_TIME =
      100; // Reset pin LOW duration (was 150)
  static const unsigned long RESET_WAIT_TIME =
      500; // Wait after reset HIGH (was 1000)
  static const unsigned long RF_HEALTH_CHECK_INTERVAL =
      10000; // Cek kesehatan RF tiap 10 detik

  // Logika Hold Validation - OPTIMIZED untuk quick tap
  static const unsigned long MIN_HOLD_TIME = 500;      // Quick tap (was 2000ms)
  static const unsigned long HOLD_LOG_INTERVAL = 1000; // Log kartu masih tempel
  static const unsigned long HOLD_PROGRESS_LOG = 500;  // Log progres "."
  static const unsigned long IDLE_LOG_INTERVAL = 10000; // Log standby

  PN532Handler(TwoWire &wire, uint8_t resetPin);

  bool begin();
  void loop();

  // Status untuk fms.ino
  bool isTagAvailable();
  String getLastUID();
  void printLastUID();
  void hardwareReset();

  // Public flags untuk event handling di fms.ino
  bool justReinitialized;
  bool justRecovered;
  bool justDetected;  // True sesaat saat awal tempel
  bool justValidated; // True sesaat saat sukses 2 detik

  // === NEW: Flags untuk LED Status ===
  bool justStandbyOK;      // True saat health check OK
  bool i2cMissed;          // True saat I2C tidak merespon (ongoing)
  bool validationCanceled; // True saat kartu diangkat sebelum 2 detik

  // Status kartu (realtime)
  bool isCardPresent;
  bool isValidated;

private:
  PN532_I2C pn532_i2c;
  PN532 nfc;
  uint8_t _resetPin;
  TwoWire *_wire;

  NfcState _state;
  bool isResetting;
  bool _newTagFlag;
  bool _softRecoveryNotified;
  bool _alreadyValidated; // Flag sticky tag

  uint8_t lastUid[MAX_UID_BYTES];
  uint8_t lastUidLength;
  uint8_t activeUid[MAX_UID_BYTES];
  uint8_t activeUidLength;

  unsigned long tagStartTime;
  unsigned long lastSeenTime;
  unsigned long lastPollTime;
  unsigned long lastSoftRecovery;
  unsigned long lastHeartbeat;
  unsigned long lastI2COK;
  unsigned long lastRfOK;
  unsigned long lastResetAction;
  unsigned long lastResetCooldown;

  // Timers untuk logging
  unsigned long _lastHoldLog;
  unsigned long _lastProgressLog;
  unsigned long lastRFHealthCheck; // NEW: Timer for RF diagnostic check

  // Recovery tracking
  uint8_t _consecutiveFailures; // Track consecutive I2C failures
  uint8_t _heartbeatCount;      // NEW: Counter for LED notification throttling
  static const uint8_t MAX_SOFT_FAILURES =
      2; // Escalate to hard reset after 2 soft failures (was 3)

  void scanForTag();
  void handleTagPresent();
  void softRfRecover();
  void handleHeartbeat();
  void handleAutoReset();
  bool checkRFHealth(); // NEW: In-depth HW diagnostic
  bool initPN532();
  bool compareUid(uint8_t *uid1, uint8_t len1, uint8_t *uid2, uint8_t len2);
  void i2cBusRecovery(); // NEW: I2C bus stuck recovery
};

#endif
