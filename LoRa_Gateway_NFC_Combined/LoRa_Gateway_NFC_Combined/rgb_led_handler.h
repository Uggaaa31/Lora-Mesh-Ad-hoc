#ifndef RGB_LED_HANDLER_H
#define RGB_LED_HANDLER_H

#include <Arduino.h>

// ESP32-S3 Built-in RGB LED
#define RGB_LED_PIN 48

// === Animation Types ===
enum RGBAnimationType {
  ANIM_NONE = 0,    // Solid color (no animation)
  ANIM_RAINBOW,     // Rainbow cycle through all hues
  ANIM_BREATHING,   // Fade in/out single color
  ANIM_PULSE,       // Fast pulse effect
  ANIM_BLINK,       // On/off blink
  ANIM_FADE_TO,     // Smooth transition to target color
  ANIM_COLOR_WAVE,  // Smooth wave between two colors
  ANIM_STROBE,      // Fast strobe effect
  ANIM_CANDLE,      // Candle flicker effect
  ANIM_NOTIFICATION // Temporary notification (returns to previous)
};

// === Notification Priority ===
enum RGBNotifyPriority {
  PRIORITY_LOW = 0,     // Can be overridden
  PRIORITY_NORMAL = 1,  // Standard notification
  PRIORITY_HIGH = 2,    // Important alert
  PRIORITY_CRITICAL = 3 // Cannot be interrupted
};

class RGBLedHandler {
public:
  RGBLedHandler(uint8_t pin = RGB_LED_PIN);
  void begin();

  // === MUST call in loop() - NON-BLOCKING ===
  void update(); // Call this every loop iteration!

  // === Immediate Color (No Animation) ===
  void setColor(uint8_t r, uint8_t g, uint8_t b);
  void setColorHSV(uint16_t hue, uint8_t sat, uint8_t val); // Hue: 0-360
  void setBrightness(uint8_t brightness);
  void off();

  // === Preset Colors (Immediate) ===
  void red();
  void green();
  void blue();
  void white();
  void yellow();
  void cyan();
  void magenta();
  void orange();
  void purple();

  // === Start Animation (Non-Blocking) ===
  void startRainbow(uint32_t periodMs = 5000); // Full rainbow cycle
  void startBreathing(uint8_t r, uint8_t g, uint8_t b,
                      uint32_t periodMs = 2000);
  void startPulse(uint8_t r, uint8_t g, uint8_t b, uint32_t periodMs = 500);
  void startBlink(uint8_t r, uint8_t g, uint8_t b, uint32_t onMs = 500,
                  uint32_t offMs = 500);
  void startFadeTo(uint8_t r, uint8_t g, uint8_t b, uint32_t durationMs = 1000);
  void startColorWave(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2,
                      uint8_t g2, uint8_t b2, uint32_t periodMs = 2000);
  void startStrobe(uint8_t r, uint8_t g, uint8_t b, uint32_t flashMs = 50);
  void startCandle(); // Warm flicker effect

  // === Custom Animation ===
  void startCustom(RGBAnimationType type, uint8_t r, uint8_t g, uint8_t b,
                   uint32_t param1 = 1000, uint32_t param2 = 0);

  // === Animation Control ===
  void stopAnimation();
  void pauseAnimation();
  void resumeAnimation();
  bool isAnimating();
  RGBAnimationType getCurrentAnimation();

  // === Notification System (Temporary Alert) ===
  void notify(uint8_t r, uint8_t g, uint8_t b, uint32_t durationMs = 1000,
              RGBNotifyPriority priority = PRIORITY_NORMAL);
  void notifyBlink(uint8_t r, uint8_t g, uint8_t b, int times = 3,
                   uint32_t intervalMs = 200,
                   RGBNotifyPriority priority = PRIORITY_NORMAL);
  void notifyPulse(uint8_t r, uint8_t g, uint8_t b, int times = 2,
                   RGBNotifyPriority priority = PRIORITY_NORMAL);

  // === Status Indicators (Preset Notifications) ===
  void statusOK();           // Green blink
  void statusWarning();      // Yellow pulse
  void statusError();        // Red fast blink
  void statusBusy();         // Blue breathing
  void statusIdle();         // Dim cyan
  void statusConnected();    // Green fade
  void statusDisconnected(); // Red fade
  void statusNFC();          // Blue flash
  void statusGPS();          // Cyan flash

  // === Getters ===
  uint8_t getBrightness();
  uint8_t getCurrentR();
  uint8_t getCurrentG();
  uint8_t getCurrentB();

private:
  uint8_t _pin;
  uint8_t _brightness;
  uint8_t _currentR, _currentG, _currentB;

  // Animation state
  RGBAnimationType _animType;
  bool _animPaused;
  uint32_t _animStartTime;
  uint32_t _animPeriod;
  uint32_t _animParam2;
  uint8_t _animR, _animG, _animB;
  uint8_t _animR2, _animG2, _animB2; // Second color for wave
  uint8_t _startR, _startG, _startB; // For fade transitions

  // Notification state
  bool _notifying;
  uint32_t _notifyEndTime;
  RGBNotifyPriority _notifyPriority;
  RGBAnimationType _savedAnimType;
  uint8_t _savedR, _savedG, _savedB;
  uint32_t _savedPeriod;

  // Timing
  uint32_t _lastUpdateTime;
  static const uint32_t UPDATE_INTERVAL = 16; // ~60 FPS max

  // Internal functions
  void sendPixel(uint8_t r, uint8_t g, uint8_t b);
  uint8_t applyBrightness(uint8_t value);
  void hsvToRgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g,
                uint8_t *b);
  void updateAnimation();
  void endNotification();
};

#endif
