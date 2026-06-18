#include "pn532_handler.h"
#include "config_manager.h"
#include "log_manager.h"

// Extern debug flag dari fms.ino
extern bool DEBUG_ALL;
extern bool DEBUG_NFC;
// Note: We will prioritize using configManager.getConfig().debug_nfc or
// debug_all

// === Constructor ===
PN532Handler::PN532Handler(TwoWire &wire, uint8_t resetPin)
    : pn532_i2c(wire), nfc(pn532_i2c), _resetPin(resetPin), _wire(&wire) {
  // Set state awal
  _state = NFC_STATE_SCAN;
  isResetting = false;
  justReinitialized = false;
  justRecovered = false;
  _newTagFlag = false;
  _softRecoveryNotified = false; // Belum print log soft recovery
  lastUidLength = 0;
  activeUidLength = 0;
  lastPollTime = 0;
  lastSoftRecovery = 0;
  lastI2COK = 0;
  lastRfOK = 0;
  isCardPresent = false;
  isValidated = false;
  justDetected = false;
  justValidated = false;
  _alreadyValidated = false;
  _lastProgressLog = 0;
  _consecutiveFailures = 0;
  _heartbeatCount = 0;
  lastRFHealthCheck = 0; // Initialize diagnostic timer
  // NEW LED flags
  justStandbyOK = false;
  i2cMissed = false;
  validationCanceled = false;
}

// === Public: begin ===
bool PN532Handler::begin() {
  pinMode(_resetPin, OUTPUT);
  digitalWrite(_resetPin, HIGH);

  nfc.begin();
  delay(100);

  bool success = initPN532();
  if (success) {
    lastI2COK = millis(); // I2C sukses saat init
    _state = NFC_STATE_SCAN;
  }
  return success;
}

// === Public: loop ===
void PN532Handler::loop() {
  if (!isResetting) {
    // === PERBAIKAN: Poll dengan interval, bukan tiap loop ===
    if (millis() - lastPollTime >= POLL_INTERVAL_MS) {
      lastPollTime = millis();

      // State machine untuk handle tag
      switch (_state) {
      case NFC_STATE_SCAN:
        scanForTag();
        break;
      case NFC_STATE_HOLD:
      case NFC_STATE_TAG_PRESENT:
        handleTagPresent();
        break;
      }
    }

    // === PERBAIKAN: Hapus refresh SAMConfig berkala (mengganggu flow) ===
    // SAMConfig hanya dipanggil saat init atau soft recovery
  }

  handleHeartbeat();
  handleAutoReset();
}

// === Public: isTagAvailable ===
bool PN532Handler::isTagAvailable() {
  if (_newTagFlag) {
    _newTagFlag = false;
    return true;
  }
  return false;
}

// === Public: printLastUID ===
void PN532Handler::printLastUID() {
  Serial.println("\n----------------------------------");
  Serial.println("New NFC Tag Detected!");
  Serial.print("UID (Hex): ");
  unsigned long long uidInt = 0;
  uint8_t len = (lastUidLength > MAX_UID_BYTES) ? MAX_UID_BYTES : lastUidLength;
  for (uint8_t i = 0; i < len; i++) {
    Serial.print("0x");
    Serial.print(lastUid[i], HEX);
    Serial.print(" ");
    uidInt = (uidInt << 8) | lastUid[i];
  }
  Serial.print("\nUID (Decimal): ");
  Serial.println(uidInt);
  Serial.println("----------------------------------");
}

// === Public: getLastUID ===
String PN532Handler::getLastUID() {
  String uidString = "";
  uint8_t len = (lastUidLength > MAX_UID_BYTES) ? MAX_UID_BYTES : lastUidLength;
  for (uint8_t i = 0; i < len; i++) {
    if (lastUid[i] < 0x10)
      uidString += "0";
    uidString += String(lastUid[i], HEX);
  }
  uidString.toUpperCase();
  return uidString;
}

// === Public: hardwareReset ===
// Manual hardware reset - dipanggil dari serial command "reset nfc"
void PN532Handler::hardwareReset() {
  Serial.println("[NFC] Manual hardware reset triggered...");

  // Toggle reset pin
  digitalWrite(_resetPin, LOW);
  delay(150); // Hold low for 150ms
  digitalWrite(_resetPin, HIGH);
  delay(500); // Wait for chip to boot

  // Re-initialize
  if (initPN532()) {
    const char *msg = "PN532 now ready!";
    logManager.info("NFC", msg);
    if (configManager.getConfig().debug_nfc ||
        configManager.getConfig().debug_all) {
      Serial.println(msg);
    }
    _state = NFC_STATE_SCAN;
    activeUidLength = 0;
    lastUidLength = 0;
    lastI2COK = millis();
    justReinitialized = true;
  } else {
    const char *msg = "PN532 still failed - check wiring!";
    logManager.error("NFC", msg);
    if (configManager.getConfig().debug_nfc ||
        configManager.getConfig().debug_all) {
      Serial.println(msg);
    }
  }
}

// === Private: scanForTag ===
// State SCAN: Mencari tag baru
void PN532Handler::scanForTag() {
  uint8_t uid[MAX_UID_BYTES] = {0};
  uint8_t uidLength;

  // === PERBAIKAN: Timeout yang lebih realistis (100ms, bukan 10ms atau
  // 150ms)
  // ===
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength,
                              READ_TIMEOUT_MS)) {
    if (uidLength > MAX_UID_BYTES) {
      uidLength = MAX_UID_BYTES;
    }

    lastI2COK = millis(); // I2C sukses
    lastRfOK = millis();  // RF juga sukses

    // === PERBAIKAN: Cek apakah ini tag yang sama dengan sebelumnya ===
    // Jika UID sama dengan lastUid (kartu masih nempel setelah soft
    // recovery), JANGAN trigger tag baru, tapi tetap masuk TAG_PRESENT untuk
    // monitor
    bool isSameAsLast = compareUid(uid, uidLength, lastUid, lastUidLength);

    // Simpan ke activeUid untuk tracking
    memcpy(activeUid, uid, uidLength);
    activeUidLength = uidLength;
    tagStartTime = millis();
    lastSeenTime = millis();
    isCardPresent = true;
    _lastProgressLog = millis();

    // === Penanganan Sticky Tag & Buzzer Tap ===
    if (isSameAsLast && _alreadyValidated) {
      // Kartu sudah valid sebelumnya (misal paska recovery)
      // Langsung ke TAG_PRESENT tanpa validasi ulang
      _state = NFC_STATE_TAG_PRESENT;
    } else {
      // Kartu baru (atau kartu lama tapi belum valid)
      memcpy(activeUid, uid, uidLength);
      activeUidLength = uidLength;
      isValidated = false;
      _alreadyValidated = false;
      justDetected = true; // Trigger buzzer tap di fms.ino
      _state = NFC_STATE_HOLD;

      const FMSConfig &cfg = configManager.getConfig();
      if (cfg.debug_all || cfg.debug_nfc) {
        Serial.println("NFC: Tag detected, holding for validation...");
      }
    }
  } else {
    isCardPresent = false;
    isValidated = false;
    _alreadyValidated = false; // Reset jika benar-benar tidak ada kartu
  }
}

// === Private: handleTagPresent ===
// State TAG_PRESENT: Tag sedang nempel, monitor sampai lepas atau timeout
void PN532Handler::handleTagPresent() {
  uint8_t uid[MAX_UID_BYTES] = {0};
  uint8_t uidLength;

  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength,
                              READ_TIMEOUT_MS)) {
    if (uidLength > MAX_UID_BYTES) {
      uidLength = MAX_UID_BYTES;
    }

    lastI2COK = millis();
    lastRfOK = millis();
    isCardPresent = true;

    if (compareUid(uid, uidLength, activeUid, activeUidLength)) {
      lastSeenTime = millis();
      unsigned long holdDuration = millis() - tagStartTime;

      if (_state == NFC_STATE_HOLD) {
        if (holdDuration >= MIN_HOLD_TIME) {
          // VALIDASI SUKSES
          isValidated = true;
          _alreadyValidated = true; // Set sticky tag
          _newTagFlag = true;
          justValidated = true; // Trigger buzzer valid di fms.ino
          memcpy(lastUid, activeUid, activeUidLength);
          lastUidLength = activeUidLength;
          _state = NFC_STATE_TAG_PRESENT;
          _lastHoldLog = millis();

          String uidStr = getLastUID();
          const char *msg = "NFC: VALIDATION SUCCESS (500ms reached)";
          logManager.info("NFC", msg, uidStr.c_str());

          if (configManager.getConfig().debug_nfc ||
              configManager.getConfig().debug_all) {
            Serial.println(msg);
            printLastUID();
          }
        } else {
          // Sedang menunggu validasi
          if (millis() - _lastProgressLog >= HOLD_PROGRESS_LOG) {
            _lastProgressLog = millis();
            Serial.print(".");
          }
        }
      } else if (_state == NFC_STATE_TAG_PRESENT) {
        // Kartu sudah valid dan masih menempel
        isValidated = true; // Tetap true selama menempel

        if (millis() - _lastHoldLog >= HOLD_LOG_INTERVAL) {
          _lastHoldLog = millis();
          if (configManager.getConfig().debug_nfc ||
              configManager.getConfig().debug_all) {
            Serial.println("kartu masih tertempel");
          }
        }

        // Anti iseng tahan terlalu lama (sama seperti sebelumnya)
        if (holdDuration > MAX_HOLD_MS) {
          if (!_softRecoveryNotified) {
            if (configManager.getConfig().debug_nfc ||
                configManager.getConfig().debug_all) {
              Serial.println("Tag held too long, soft recovery...");
            }
            _softRecoveryNotified = true;
            justRecovered = true;
          }
          softRfRecover();
          _state = NFC_STATE_SCAN;
        }
      }
    } else {
      // Tag berbeda terdeteksi
      _state = NFC_STATE_SCAN; // Reset ke scan untuk identitas baru
    }
  } else {
    // Tag tidak terlihat
    if (millis() - lastSeenTime > REMOVE_TIMEOUT_MS) {
      if (_state == NFC_STATE_TAG_PRESENT) {
        const char *msg = "Tag removed, ready for next";
        logManager.info("NFC", msg);
        if (configManager.getConfig().debug_nfc ||
            configManager.getConfig().debug_all) {
          Serial.println(msg);
        }
      } else {
        const char *msg = "Validation cancelled (tag removed)";
        logManager.warn("NFC", msg);
        if (configManager.getConfig().debug_nfc ||
            configManager.getConfig().debug_all) {
          Serial.println(msg);
        }
        validationCanceled = true; // LED flag
      }
      _state = NFC_STATE_SCAN;
      isCardPresent = false;
      isValidated = false;
      _alreadyValidated = false; // Reset sticky
      activeUidLength = 0;
      _softRecoveryNotified = false;
    }
  }
}

// === Private: softRfRecover ===
// Recovery dengan I2C bus recovery dan verification
void PN532Handler::softRfRecover() {
  // Cek cooldown
  if (millis() - lastSoftRecovery < SOFT_RECOVERY_COOLDOWN) {
    return;
  }
  lastSoftRecovery = millis();

  Serial.println("[NFC] Attempting soft recovery...");

  // Step 1: I2C bus recovery (clock stretching)
  i2cBusRecovery();

  // Step 2: Re-configure SAM
  nfc.SAMConfig();
  nfc.setPassiveActivationRetries(0x01);
  delay(50);

  // Step 3: Verify recovery berhasil
  uint32_t fw = nfc.getFirmwareVersion();
  if (fw) {
    logManager.info("NFC", "Soft recovery SUCCESS");
    if (configManager.getConfig().debug_nfc ||
        configManager.getConfig().debug_all) {
      Serial.println("[NFC] Soft recovery SUCCESS");
    }
    lastI2COK = millis();     // UPDATE lastI2COK!
    _consecutiveFailures = 0; // Reset failure counter
  } else {
    logManager.warn("NFC", "Soft recovery FAILED");
    if (configManager.getConfig().debug_nfc ||
        configManager.getConfig().debug_all) {
      Serial.println("[NFC] Soft recovery FAILED");
    }
    _consecutiveFailures++;

    // Escalate ke hardware reset jika sudah berkali-kali gagal
    if (_consecutiveFailures >= MAX_SOFT_FAILURES) {
      logManager.error("NFC", "Critical: Soft recovery failed repeatedly, "
                              "escalating to hard reset");
      if (configManager.getConfig().debug_nfc ||
          configManager.getConfig().debug_all) {
        Serial.println("[NFC] Critical: Soft recovery failed repeatedly, "
                       "escalating to hard reset");
      }
      lastI2COK = 0; // Force trigger hardware reset
    }
  }
}

// === Private: handleHeartbeat ===
// Cek I2C alive dan trigger recovery jika perlu
void PN532Handler::handleHeartbeat() {
  if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = millis();
    uint32_t fw = nfc.getFirmwareVersion();

    if (fw) {
      // I2C masih hidup - UPDATE lastI2COK!
      lastI2COK = millis();
      _consecutiveFailures = 0; // Reset failure counter

      const FMSConfig &cfg = configManager.getConfig();
      if (cfg.debug_all || cfg.debug_nfc) {
        Serial.print("[Heartbeat] PN532 I2C OK | State=");
        Serial.print(_state == NFC_STATE_SCAN
                         ? "SCAN"
                         : (_state == NFC_STATE_HOLD ? "HOLD" : "TAG_PRESENT"));
        Serial.print(" | activeUidLen=");
        Serial.print(activeUidLength);
        Serial.print(" | lastUidLen=");
        Serial.println(lastUidLength);
      }

      // Log standby jika SCAN
      if (_state == NFC_STATE_SCAN) {
        _heartbeatCount++;
        if (_heartbeatCount >= 5) {
          if (configManager.getConfig().debug_nfc ||
              configManager.getConfig().debug_all) {
            Serial.println("[NFC] Standby - Health Check OK (5s report)");
          }

          // === NEW: Periodically check true RF Health (beyond I2C) ===
          if (!checkRFHealth()) {
            logManager.error("NFC",
                             "RF Backend locked! Forcing hardware reset");
            lastI2COK = 0; // Trigger hard reset
            return;
          }

          justStandbyOK = true; // LED flag triggered every 5s
          _heartbeatCount = 0;
        }
      }
      i2cMissed = false; // Clear missed flag
    } else {
      logManager.warn("NFC", "I2C heartbeat missed, attempting recovery");
      if (configManager.getConfig().debug_nfc ||
          configManager.getConfig().debug_all) {
        Serial.println("[NFC] I2C heartbeat missed, attempting recovery...");
      }
      _heartbeatCount = 0; // Reset counter on failure
      _consecutiveFailures++;
      i2cMissed = true; // LED flag

      // Langsung coba recovery, jangan hanya retry getFirmwareVersion
      softRfRecover();
    }
  }
}

// === Private: checkRFHealth ===
// Diagnostics yang lebih dalam dari sekadar I2C (cek register antena)
bool PN532Handler::checkRFHealth() {
  // 1. Cek CIU_Control (0x6331) - Bit 4 (Initiator)
  // Register ini menunjukkan mode operasi internal
  uint32_t valCtrl = nfc.readRegister(0x6331);
  uint8_t ciuCtrl = (uint8_t)(valCtrl & 0xFF);

  // 2. Cek CIU_TxControl (0x633C) - Antenna status
  uint32_t valTx = nfc.readRegister(0x633C);
  uint8_t txCtrl = (uint8_t)(valTx & 0xFF);

  // Forced logging untuk debug user (abaikan DEBUG_PN532 sementara)
  if (configManager.getConfig().debug_nfc ||
      configManager.getConfig().debug_all) {
    Serial.print("[NFC] HW Diag -> Control:0x");
    Serial.print(ciuCtrl, HEX);
    Serial.print(" | Tx:0x");
    Serial.println(txCtrl, HEX);
  }

  // Jika I2C gagal (readRegister return 0, tapi 0 juga bisa nilai TxControl)
  // Dalam mode SCAN normal, txCtrl biasanya 0x80 atau 0x83 atau 0x4x
  // Jika ciuCtrl 0x00 DAN txCtrl 0x00, chip kemungkinan pingsan (paska voltage
  // drop)
  if (valCtrl == 0x00 && valTx == 0x00) {
    if (configManager.getConfig().debug_nfc ||
        configManager.getConfig().debug_all) {
      Serial.println("[NFC] HW Diag RESULT: FAILED (Backend is SILENT)");
    }
    return false;
  }

  // Jika register mengembalikan 0xFF via I2C (ini jarang tapi mungkin jika bus
  // corrupt)
  if (ciuCtrl == 0xFF) {
    if (configManager.getConfig().debug_nfc ||
        configManager.getConfig().debug_all) {
      Serial.println("[NFC] HW Diag RESULT: FAILED (Read Error)");
    }
    return false;
  }

  return true;
}

// === Private: handleAutoReset ===
// Hard reset hanya jika I2C benar-benar tidak ada respons dalam waktu lama
void PN532Handler::handleAutoReset() {
  // Cek berdasarkan lastI2COK, bukan lastRfOK!
  if (!isResetting && (millis() - lastI2COK > PN532_TIMEOUT_MS) &&
      (millis() - lastResetCooldown > RESET_COOLDOWN_MS)) {

    logManager.error("NFC",
                     "Non-responsive (TIMEOUT), triggering instand hard reset");
    digitalWrite(_resetPin, LOW);
    lastResetAction = millis();
    lastResetCooldown = millis();
    isResetting = true;
  }

  if (isResetting && millis() - lastResetAction > RESET_LOW_TIME &&
      digitalRead(_resetPin) == LOW) {
    digitalWrite(_resetPin, HIGH);
    lastResetAction = millis();
  }

  if (isResetting && millis() - lastResetAction > RESET_WAIT_TIME &&
      digitalRead(_resetPin) == HIGH) {

    const char *msg = "Reinitializing after hardware reset";
    logManager.info("NFC", msg);
    if (configManager.getConfig().debug_nfc ||
        configManager.getConfig().debug_all) {
      Serial.println(msg);
    }
    if (initPN532()) {
      isResetting = false;
      justReinitialized = true;
      _state = NFC_STATE_SCAN;
    } else {
      const char *errMsg = "Failed to re-init after reset";
      logManager.error("NFC", errMsg);
      if (configManager.getConfig().debug_nfc ||
          configManager.getConfig().debug_all) {
        Serial.println(errMsg);
      }
      lastI2COK = millis(); // Reset timer agar tidak langsung reset lagi
    }
  }
}

// === Private: initPN532 ===
bool PN532Handler::initPN532() {
  delay(50);

  // Wake-up dummy write
  for (int i = 0; i < 2; i++) {
    _wire->beginTransmission(0x24);
    _wire->write(0x00);
    _wire->endTransmission();
    delay(30);
  }

  uint8_t retry = 0;
  uint32_t versiondata = 0;
  while (retry < 5 && !versiondata) {
    versiondata = nfc.getFirmwareVersion();
    if (!versiondata) {
      if (configManager.getConfig().debug_nfc ||
          configManager.getConfig().debug_all) {
        Serial.print("PN532 not ready, retry ");
        Serial.println(retry + 1);
      }
      delay(300);
      retry++;
    }
  }

  if (!versiondata) {
    const char *msg = "PN532 not found after retries!";
    logManager.error("NFC", msg);
    if (configManager.getConfig().debug_nfc ||
        configManager.getConfig().debug_all) {
      Serial.println(msg);
    }
    return false;
  }

  if (configManager.getConfig().debug_nfc ||
      configManager.getConfig().debug_all) {
    Serial.print("Found PN5");
    Serial.println((versiondata >> 24) & 0xFF, HEX);
    Serial.print("Firmware ver. ");
    Serial.print((versiondata >> 16) & 0xFF);
    Serial.print('.');
    Serial.println((versiondata >> 8) & 0xFF);
  }

  // Configure SAM
  nfc.SAMConfig();

  // === PERBAIKAN: Set retries kecil untuk mode non-blocking yang sehat ===
  nfc.setPassiveActivationRetries(0x01); // 1 retry saja

  if (configManager.getConfig().debug_nfc ||
      configManager.getConfig().debug_all) {
    Serial.println("Ready for NFC tag...");
  }
  lastPollTime = millis();
  lastSoftRecovery = millis();
  return true;
}

// === Private: compareUid ===
// Helper untuk bandingkan dua UID
bool PN532Handler::compareUid(uint8_t *uid1, uint8_t len1, uint8_t *uid2,
                              uint8_t len2) {
  if (len1 > MAX_UID_BYTES || len2 > MAX_UID_BYTES) {
    return false;
  }
  if (len1 != len2)
    return false;
  for (uint8_t i = 0; i < len1; i++) {
    if (uid1[i] != uid2[i])
      return false;
  }
  return true;
}

// === Private: i2cBusRecovery ===
// Recovery I2C bus yang stuck dengan clock stretching
void PN532Handler::i2cBusRecovery() {
  Serial.println("[NFC] I2C bus recovery - toggling clock...");

  _wire->end();
  delay(10);

  // Re-initialize Wire dengan pin yang sama
  _wire->begin();
  _wire->setClock(50000); // 50kHz untuk PN532

  delay(50);

  // Wake-up PN532 dengan dummy write
  for (int i = 0; i < 3; i++) {
    _wire->beginTransmission(0x24);
    _wire->write(0x00);
    _wire->endTransmission();
    delay(20);
  }

  Serial.println("[NFC] I2C bus recovery complete");
}
