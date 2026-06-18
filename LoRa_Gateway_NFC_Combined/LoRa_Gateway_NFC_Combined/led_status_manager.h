#ifndef LED_STATUS_MANAGER_H
#define LED_STATUS_MANAGER_H

#include "rgb_led_handler.h"
#include <Arduino.h>

// === LED State Types ===
enum LedStateType {
  STATE_WIFI_DISCONNECTED,  // Rainbow smooth
  STATE_WIFI_CONNECTED,     // (none - skip to GPS)
  STATE_GPS_SEARCHING,      // Yellow breathing
  STATE_GPS_SIGNAL_NO_FIX,  // Yellow double-pulse
  STATE_GPS_FIX_OK,         // Cyan 2x fast
  STATE_NFC_STANDBY_OK,     // Blue 1x blink
  STATE_NFC_MISSED,         // Red 2x fast
  STATE_NFC_VALID_TAP,      // Green 2x fast
  STATE_NFC_CANCELED,       // Orange 1x
  STATE_NFC_HARDWARE_RESET, // Red 1x
  STATE_COUNT               // Total count
};

class LedStatusManager {
public:
  LedStatusManager(RGBLedHandler &led);

  // Call in loop() - NON-BLOCKING
  void update();

  // === Set Background Status ===
  void setWiFiStatus(bool connected);
  void setGPSStatus(bool hasSignal, bool hasFix);

  // === NFC Event Triggers ===
  void triggerNFCStandbyOK();
  void triggerNFCValidTap();
  void triggerNFCCanceled();
  void triggerNFCHardwareReset();
  void triggerNFCMissed(bool missed);

private:
  RGBLedHandler &_led;

  // === State Activation Flags ===
  bool _stateActive[STATE_COUNT];

  // === Round-Robin Control ===
  uint8_t _currentStateIndex;
  unsigned long _stateStartTime;
  unsigned long _stateDuration[STATE_COUNT]; // Duration per state pattern

  // Background status
  bool _wifiConnected;
  bool _gpsHasSignal;
  bool _gpsHasFix;
  bool _nfcMissed;
  unsigned long
      _wifiConnectedLastTrigger; // Timer for WiFi Connected (every 15s)

  // Timing
  unsigned long _lastUpdate;
  static const uint32_t UPDATE_INTERVAL = 16; // ~60 FPS

  // Fade transition (500ms total: 250ms out + 250ms in)
  uint8_t _fadePhase; // 0=normal, 1=fade out, 2=fade in
  unsigned long _fadeStartTime;
  float _fadeBrightness;                     // 0.0 - 1.0
  static const uint32_t FADE_DURATION = 250; // ms per phase

  // Internal functions
  void updateActiveStates();
  void advanceToNextState();
  void playCurrentState();
  bool isStateActive(LedStateType state);
  uint8_t getActiveStateCount();
  void applyFade(uint8_t &r, uint8_t &g, uint8_t &b);

  // Pattern functions
  void playRainbowSmooth();
  void playRedBreathing();
  void playPurpleBreathing();
  void playYellowBreathing();
  void playBlueBreathing();
  void playYellowDoublePulse();
  void playCyanDoubleBlink();
  void playBlueSingleBlink();
  void playBlueDoubleFast();
  void playRedDoubleFast();
  void playGreenDoubleFast();
  void playRedSingleBlink();
  void playOrangeSingleBlink();
  void playPinkTripleFast();
  void playCyanDoubleFast();
  void playOrangeDoubleFast();
};

#endif
