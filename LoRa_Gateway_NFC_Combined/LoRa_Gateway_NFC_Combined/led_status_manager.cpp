#include "led_status_manager.h"

// === Constructor ===
LedStatusManager::LedStatusManager(RGBLedHandler &led) : _led(led) {
  // Clear all states
  for (int i = 0; i < STATE_COUNT; i++) {
    _stateActive[i] = false;
  }

  // Set pattern durations (ms) - complete cycle per state
  _stateDuration[STATE_WIFI_DISCONNECTED] = 3000; // Red breathing 3 sec
  _stateDuration[STATE_WIFI_CONNECTED] = 3000;    // Purple breathing 3 sec
  _stateDuration[STATE_GPS_SEARCHING] = 3000;     // 1 breathing cycle
  _stateDuration[STATE_GPS_SIGNAL_NO_FIX] = 2000; // 1 double-pulse cycle
  _stateDuration[STATE_GPS_FIX_OK] = 500;         // Cyan 2x fast blink
  _stateDuration[STATE_NFC_STANDBY_OK] = 200;     // Blue 2x fast blink (0.2s)
  _stateDuration[STATE_NFC_MISSED] = 1000;        // 1 red double-fast cycle
  _stateDuration[STATE_NFC_VALID_TAP] = 500;      // Green 2x blink
  _stateDuration[STATE_NFC_CANCELED] = 500;       // Orange 2x fast blink
  _stateDuration[STATE_NFC_HARDWARE_RESET] = 400; // Red blink

  _currentStateIndex = 0;
  _stateStartTime = 0;
  _lastUpdate = 0;

  _wifiConnected = false;
  _gpsHasSignal = false;
  _gpsHasFix = false;
  _nfcMissed = false;
  _wifiConnectedLastTrigger = 0;

  // Fade transition init
  _fadePhase = 0; // 0=normal, 1=fade out, 2=fade in
  _fadeStartTime = 0;
  _fadeBrightness = 1.0f;
}

// === Public: update() ===
void LedStatusManager::update() {
  uint32_t now = millis();

  // Rate limit updates
  if (now - _lastUpdate < UPDATE_INTERVAL)
    return;
  _lastUpdate = now;

  // Update which states are active
  updateActiveStates();

  // Handle fade transitions
  if (_fadePhase == 1) {
    // Fade out phase
    uint32_t elapsed = now - _fadeStartTime;
    if (elapsed >= FADE_DURATION) {
      _fadePhase = 2; // Switch to fade in
      _fadeStartTime = now;
      advanceToNextState();
      _stateStartTime = now;
    } else {
      _fadeBrightness = 1.0f - ((float)elapsed / FADE_DURATION);
    }
  } else if (_fadePhase == 2) {
    // Fade in phase
    uint32_t elapsed = now - _fadeStartTime;
    if (elapsed >= FADE_DURATION) {
      _fadePhase = 0; // Back to normal
      _fadeBrightness = 1.0f;
    } else {
      _fadeBrightness = (float)elapsed / FADE_DURATION;
    }
  } else {
    // Normal operation - check if state time is up
    if (now - _stateStartTime >= _stateDuration[_currentStateIndex]) {
      _fadePhase = 1; // Start fade out
      _fadeStartTime = now;
    }
  }

  // Play current state pattern
  playCurrentState();
}

// === Private: updateActiveStates ===
void LedStatusManager::updateActiveStates() {
  // Reset all first
  for (int i = 0; i < STATE_COUNT; i++) {
    _stateActive[i] = false;
  }

  // WiFi states (mutually exclusive)
  if (!_wifiConnected) {
    _stateActive[STATE_WIFI_DISCONNECTED] = true;
  } else {
    _stateActive[STATE_WIFI_CONNECTED] = true; // Purple breathing continuous
  }

  // GPS states (always show one)
  if (_gpsHasFix) {
    _stateActive[STATE_GPS_FIX_OK] = true;
  } else if (_gpsHasSignal) {
    _stateActive[STATE_GPS_SIGNAL_NO_FIX] = true;
  } else {
    _stateActive[STATE_GPS_SEARCHING] = true;
  }

  // NFC missed (ongoing error)
  if (_nfcMissed) {
    _stateActive[STATE_NFC_MISSED] = true;
  }

  // Note: One-shot events (standby OK, valid tap, etc) are handled
  // by triggers which temporarily activate their state
}

// === Private: advanceToNextState ===
void LedStatusManager::advanceToNextState() {
  // Find next active state in round-robin
  uint8_t startIndex = _currentStateIndex;

  do {
    _currentStateIndex = (_currentStateIndex + 1) % STATE_COUNT;

    // Skip states with 0 duration
    if (_stateDuration[_currentStateIndex] == 0)
      continue;

    // Found active state
    if (_stateActive[_currentStateIndex]) {
      return;
    }

  } while (_currentStateIndex != startIndex);

  // If no active states, stay on current or default
  if (!_stateActive[_currentStateIndex]) {
    _currentStateIndex = STATE_GPS_SEARCHING; // Default
  }
}

// === Private: playCurrentState ===
void LedStatusManager::playCurrentState() {
  switch (_currentStateIndex) {
  case STATE_WIFI_DISCONNECTED:
    playRedBreathing(); // Red breathing 3 sec
    break;
  case STATE_WIFI_CONNECTED:
    playPurpleBreathing(); // Purple breathing 3 sec (continuous)
    break;
  case STATE_GPS_SEARCHING:
    playYellowBreathing();
    break;
  case STATE_GPS_SIGNAL_NO_FIX:
    playYellowDoublePulse();
    break;
  case STATE_GPS_FIX_OK:
    playCyanDoubleFast();
    break;
  case STATE_NFC_STANDBY_OK:
    playBlueDoubleFast();
    if (millis() - _stateStartTime >= _stateDuration[STATE_NFC_STANDBY_OK]) {
      _stateActive[STATE_NFC_STANDBY_OK] = false; // One-shot done
    }
    break;
  case STATE_NFC_MISSED:
    playRedDoubleFast();
    break;
  case STATE_NFC_VALID_TAP:
    playGreenDoubleFast();
    if (millis() - _stateStartTime >= _stateDuration[STATE_NFC_VALID_TAP]) {
      _stateActive[STATE_NFC_VALID_TAP] = false;
    }
    break;
  case STATE_NFC_CANCELED:
    playOrangeDoubleFast();
    if (millis() - _stateStartTime >= _stateDuration[STATE_NFC_CANCELED]) {
      _stateActive[STATE_NFC_CANCELED] = false;
    }
    break;
  case STATE_NFC_HARDWARE_RESET:
    playRedSingleBlink();
    if (millis() - _stateStartTime >=
        _stateDuration[STATE_NFC_HARDWARE_RESET]) {
      _stateActive[STATE_NFC_HARDWARE_RESET] = false;
    }
    break;
  default:
    _led.setColor(0, 0, 0); // Off
    break;
  }
}

// === Public: Set Status Functions ===
void LedStatusManager::setWiFiStatus(bool connected) {
  _wifiConnected = connected;
}

void LedStatusManager::setGPSStatus(bool hasSignal, bool hasFix) {
  _gpsHasSignal = hasSignal;
  _gpsHasFix = hasFix;
}

// === NFC Event Triggers ===
void LedStatusManager::triggerNFCStandbyOK() {
  _stateActive[STATE_NFC_STANDBY_OK] = true;
  // Jump to this state immediately if not playing critical
  if (_currentStateIndex != STATE_NFC_VALID_TAP &&
      _currentStateIndex != STATE_NFC_MISSED) {
    _currentStateIndex = STATE_NFC_STANDBY_OK;
    _stateStartTime = millis();
  }
}

void LedStatusManager::triggerNFCValidTap() {
  _stateActive[STATE_NFC_VALID_TAP] = true;
  // High priority - jump immediately
  _currentStateIndex = STATE_NFC_VALID_TAP;
  _stateStartTime = millis();
}

void LedStatusManager::triggerNFCCanceled() {
  _stateActive[STATE_NFC_CANCELED] = true;
  _currentStateIndex = STATE_NFC_CANCELED;
  _stateStartTime = millis();
}

void LedStatusManager::triggerNFCHardwareReset() {
  _stateActive[STATE_NFC_HARDWARE_RESET] = true;
  _currentStateIndex = STATE_NFC_HARDWARE_RESET;
  _stateStartTime = millis();
}

void LedStatusManager::triggerNFCMissed(bool missed) { _nfcMissed = missed; }

// === Pattern Functions ===

// Apply fade multiplier to colors before sending to LED
void LedStatusManager::applyFade(uint8_t &r, uint8_t &g, uint8_t &b) {
  r = (uint8_t)(r * _fadeBrightness);
  g = (uint8_t)(g * _fadeBrightness);
  b = (uint8_t)(b * _fadeBrightness);
}

void LedStatusManager::playRainbowSmooth() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint16_t hue = (elapsed / 6) % 360; // Faster rainbow within 2 sec

  uint8_t r, g, b;
  uint8_t region = hue / 60;
  uint8_t remainder = (hue % 60) * 255 / 60;
  switch (region) {
  case 0:
    r = 255;
    g = remainder;
    b = 0;
    break;
  case 1:
    r = 255 - remainder;
    g = 255;
    b = 0;
    break;
  case 2:
    r = 0;
    g = 255;
    b = remainder;
    break;
  case 3:
    r = 0;
    g = 255 - remainder;
    b = 255;
    break;
  case 4:
    r = remainder;
    g = 0;
    b = 255;
    break;
  default:
    r = 255;
    g = 0;
    b = 255 - remainder;
    break;
  }
  uint8_t fr = (uint8_t)(r * _fadeBrightness);
  uint8_t fg = (uint8_t)(g * _fadeBrightness);
  uint8_t fb = (uint8_t)(b * _fadeBrightness);
  _led.setColor(fr, fg, fb);
}

void LedStatusManager::playRedBreathing() {
  uint32_t elapsed = millis() - _stateStartTime;
  float phase = (float)(elapsed % 3000) / 3000.0f;
  float brightness =
      (sin(phase * 2 * PI - PI / 2) + 1.0f) / 2.0f * _fadeBrightness;
  uint8_t br = (uint8_t)(brightness * 255);
  _led.setColor(br, 0, 0); // Red
}

void LedStatusManager::playPurpleBreathing() {
  uint32_t elapsed = millis() - _stateStartTime;
  float phase = (float)(elapsed % 3000) / 3000.0f; // 3 sec cycle
  float brightness =
      (sin(phase * 2 * PI - PI / 2) + 1.0f) / 2.0f * _fadeBrightness;
  uint8_t br = (uint8_t)(brightness * 255);
  _led.setColor((150 * br) / 255, 0, br); // Purple
}

void LedStatusManager::playYellowBreathing() {
  uint32_t elapsed = millis() - _stateStartTime;
  float phase = (float)(elapsed % 3000) / 3000.0f;
  float brightness =
      (sin(phase * 2 * PI - PI / 2) + 1.0f) / 2.0f * _fadeBrightness;
  uint8_t br = (uint8_t)(brightness * 255);
  _led.setColor(br, (180 * br) / 255, 0); // Yellow
}

void LedStatusManager::playBlueBreathing() {
  uint32_t elapsed = millis() - _stateStartTime;
  float phase = (float)(elapsed % 3000) / 3000.0f; // 3 sec cycle
  float brightness =
      (sin(phase * 2 * PI - PI / 2) + 1.0f) / 2.0f * _fadeBrightness;
  uint8_t br = (uint8_t)(brightness * 255);
  _led.setColor(0, 0, br); // Blue
}

void LedStatusManager::playYellowDoublePulse() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint32_t cyclePos = elapsed % 2000;
  uint8_t r = 0, g = 0, b = 0;

  if (cyclePos < 100) {
    r = 255;
    g = 180;
    b = 0;
  } else if (cyclePos < 200) {
    r = 0;
    g = 0;
    b = 0;
  } else if (cyclePos < 300) {
    r = 255;
    g = 180;
    b = 0;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

void LedStatusManager::playCyanDoubleBlink() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint32_t cyclePos = elapsed % 3000;
  uint8_t r = 0, g = 0, b = 0;

  if (cyclePos < 80) {
    r = 0;
    g = 255;
    b = 200;
  } else if (cyclePos < 160) {
    r = 0;
    g = 0;
    b = 0;
  } else if (cyclePos < 240) {
    r = 0;
    g = 255;
    b = 200;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

void LedStatusManager::playBlueSingleBlink() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint8_t r = 0, g = 0, b = 0;
  if (elapsed < 200) {
    r = 0;
    g = 100;
    b = 255;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

void LedStatusManager::playRedDoubleFast() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint32_t cyclePos = elapsed % 1000;
  uint8_t r = 0, g = 0, b = 0;

  if (cyclePos < 80) {
    r = 255;
    g = 0;
    b = 0;
  } else if (cyclePos < 160) {
    r = 0;
    g = 0;
    b = 0;
  } else if (cyclePos < 240) {
    r = 255;
    g = 0;
    b = 0;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

void LedStatusManager::playGreenDoubleFast() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint8_t r = 0, g = 0, b = 0;

  if (elapsed < 100) {
    r = 0;
    g = 255;
    b = 0;
  } else if (elapsed < 200) {
    r = 0;
    g = 0;
    b = 0;
  } else if (elapsed < 300) {
    r = 0;
    g = 255;
    b = 0;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

// Blue 2x fast blink for NFC Standby OK
void LedStatusManager::playBlueDoubleFast() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint8_t r = 0, g = 0, b = 0;
  if (elapsed < 100) {
    r = 0;
    g = 0;
    b = 255; // Blue
  } else if (elapsed < 200) {
    r = 0;
    g = 0;
    b = 0;
  } else if (elapsed < 300) {
    r = 0;
    g = 0;
    b = 255; // Blue
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

void LedStatusManager::playRedSingleBlink() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint8_t r = 0, g = 0, b = 0;
  if (elapsed < 200) {
    r = 255;
    g = 0;
    b = 0;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

void LedStatusManager::playOrangeSingleBlink() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint8_t r = 0, g = 0, b = 0;
  if (elapsed < 200) {
    r = 255;
    g = 128;
    b = 0;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

// Pink 3x fast blink for WiFi Connected
void LedStatusManager::playPinkTripleFast() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint8_t r = 0, g = 0, b = 0;

  if (elapsed < 80) {
    r = 255;
    g = 50;
    b = 150; // Pink
  } else if (elapsed < 160) {
    r = 0;
    g = 0;
    b = 0;
  } else if (elapsed < 240) {
    r = 255;
    g = 50;
    b = 150;
  } else if (elapsed < 320) {
    r = 0;
    g = 0;
    b = 0;
  } else if (elapsed < 400) {
    r = 255;
    g = 50;
    b = 150;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

// Cyan 2x fast blink for GPS Fix OK
void LedStatusManager::playCyanDoubleFast() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint8_t r = 0, g = 0, b = 0;

  if (elapsed < 80) {
    r = 0;
    g = 255;
    b = 200; // Cyan
  } else if (elapsed < 160) {
    r = 0;
    g = 0;
    b = 0;
  } else if (elapsed < 240) {
    r = 0;
    g = 255;
    b = 200;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}

// Orange 2x fast blink for NFC Canceled
void LedStatusManager::playOrangeDoubleFast() {
  uint32_t elapsed = millis() - _stateStartTime;
  uint8_t r = 0, g = 0, b = 0;

  if (elapsed < 80) {
    r = 255;
    g = 128;
    b = 0; // Orange
  } else if (elapsed < 160) {
    r = 0;
    g = 0;
    b = 0;
  } else if (elapsed < 240) {
    r = 255;
    g = 128;
    b = 0;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  applyFade(r, g, b);
  _led.setColor(r, g, b);
}
