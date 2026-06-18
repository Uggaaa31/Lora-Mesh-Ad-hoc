#include "rgb_led_handler.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"


// === WS2812 Timing Macros ===
#define NOP1 __asm__ __volatile__("nop")
#define NOP10                                                                  \
  NOP1;                                                                        \
  NOP1;                                                                        \
  NOP1;                                                                        \
  NOP1;                                                                        \
  NOP1;                                                                        \
  NOP1;                                                                        \
  NOP1;                                                                        \
  NOP1;                                                                        \
  NOP1;                                                                        \
  NOP1

// === Constructor ===
RGBLedHandler::RGBLedHandler(uint8_t pin) : _pin(pin) {
  _currentR = _currentG = _currentB = 0;
  _brightness = 255;
  _animType = ANIM_NONE;
  _animPaused = false;
  _notifying = false;
  _lastUpdateTime = 0;
}

// === Public: begin ===
void RGBLedHandler::begin() {
  gpio_set_direction((gpio_num_t)_pin, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)_pin, 0);
  delay(1);
  off();
  Serial.println(
      "[RGB] LED handler initialized (non-blocking animation engine)");
}

// === Private: applyBrightness ===
uint8_t RGBLedHandler::applyBrightness(uint8_t value) {
  return (uint8_t)(((uint16_t)value * _brightness) / 255);
}

// === Private: HSV to RGB conversion ===
void RGBLedHandler::hsvToRgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r,
                             uint8_t *g, uint8_t *b) {
  if (s == 0) {
    *r = *g = *b = v;
    return;
  }

  h = h % 360;
  uint8_t region = h / 60;
  uint8_t remainder = (h % 60) * 255 / 60;

  uint8_t p = (v * (255 - s)) / 255;
  uint8_t q = (v * (255 - (s * remainder) / 255)) / 255;
  uint8_t t = (v * (255 - (s * (255 - remainder)) / 255)) / 255;

  switch (region) {
  case 0:
    *r = v;
    *g = t;
    *b = p;
    break;
  case 1:
    *r = q;
    *g = v;
    *b = p;
    break;
  case 2:
    *r = p;
    *g = v;
    *b = t;
    break;
  case 3:
    *r = p;
    *g = q;
    *b = v;
    break;
  case 4:
    *r = t;
    *g = p;
    *b = v;
    break;
  default:
    *r = v;
    *g = p;
    *b = q;
    break;
  }
}

// === Private: Send pixel (High GPIO 32-48) ===
static void IRAM_ATTR sendByteHigh(uint8_t pin, uint8_t data) {
  uint32_t mask = 1UL << (pin - 32);
  for (int bit = 7; bit >= 0; bit--) {
    if (data & (1 << bit)) {
      GPIO.out1_w1ts.val = mask;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      GPIO.out1_w1tc.val = mask;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
    } else {
      GPIO.out1_w1ts.val = mask;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      GPIO.out1_w1tc.val = mask;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
    }
  }
}

static void IRAM_ATTR sendByteLow(uint8_t pin, uint8_t data) {
  uint32_t mask = 1UL << pin;
  for (int bit = 7; bit >= 0; bit--) {
    if (data & (1 << bit)) {
      GPIO.out_w1ts = mask;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      GPIO.out_w1tc = mask;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
    } else {
      GPIO.out_w1ts = mask;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      GPIO.out_w1tc = mask;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
      NOP10;
    }
  }
}

void RGBLedHandler::sendPixel(uint8_t r, uint8_t g, uint8_t b) {
  portDISABLE_INTERRUPTS();
  if (_pin >= 32) {
    sendByteHigh(_pin, g);
    sendByteHigh(_pin, r);
    sendByteHigh(_pin, b);
  } else {
    sendByteLow(_pin, g);
    sendByteLow(_pin, r);
    sendByteLow(_pin, b);
  }
  portENABLE_INTERRUPTS();
  delayMicroseconds(80);
  _currentR = r;
  _currentG = g;
  _currentB = b;
}

// === Public: update() - CALL IN LOOP ===
void RGBLedHandler::update() {
  uint32_t now = millis();

  // Rate limit updates (~60 FPS max)
  if (now - _lastUpdateTime < UPDATE_INTERVAL)
    return;
  _lastUpdateTime = now;

  // Check notification timeout
  if (_notifying && now >= _notifyEndTime) {
    endNotification();
  }

  // Update animation if active and not paused
  if (_animType != ANIM_NONE && !_animPaused) {
    updateAnimation();
  }
}

// === Private: updateAnimation ===
void RGBLedHandler::updateAnimation() {
  uint32_t elapsed = millis() - _animStartTime;
  float phase = (float)(elapsed % _animPeriod) / _animPeriod; // 0.0 - 1.0
  uint8_t r, g, b;

  switch (_animType) {
  case ANIM_RAINBOW: {
    uint16_t hue = (uint16_t)(phase * 360);
    hsvToRgb(hue, 255, 255, &r, &g, &b);
    sendPixel(applyBrightness(r), applyBrightness(g), applyBrightness(b));
    break;
  }

  case ANIM_BREATHING: {
    // Smooth sine wave for natural breathing
    float breath = (sin(phase * 2 * PI - PI / 2) + 1.0f) / 2.0f;
    uint8_t br = (uint8_t)(breath * 255);
    r = (_animR * br) / 255;
    g = (_animG * br) / 255;
    b = (_animB * br) / 255;
    sendPixel(applyBrightness(r), applyBrightness(g), applyBrightness(b));
    break;
  }

  case ANIM_PULSE: {
    // Sharp pulse up then fade down
    float pulse;
    if (phase < 0.1f) {
      pulse = phase / 0.1f; // Quick rise
    } else {
      pulse = 1.0f - ((phase - 0.1f) / 0.9f); // Slow fall
    }
    r = (_animR * (uint8_t)(pulse * 255)) / 255;
    g = (_animG * (uint8_t)(pulse * 255)) / 255;
    b = (_animB * (uint8_t)(pulse * 255)) / 255;
    sendPixel(applyBrightness(r), applyBrightness(g), applyBrightness(b));
    break;
  }

  case ANIM_BLINK: {
    float onRatio = (float)_animPeriod / (_animPeriod + _animParam2);
    if (phase < onRatio) {
      sendPixel(applyBrightness(_animR), applyBrightness(_animG),
                applyBrightness(_animB));
    } else {
      sendPixel(0, 0, 0);
    }
    break;
  }

  case ANIM_FADE_TO: {
    if (elapsed >= _animPeriod) {
      // Transition complete
      sendPixel(applyBrightness(_animR), applyBrightness(_animG),
                applyBrightness(_animB));
      _animType = ANIM_NONE;
    } else {
      r = _startR + ((_animR - _startR) * elapsed) / _animPeriod;
      g = _startG + ((_animG - _startG) * elapsed) / _animPeriod;
      b = _startB + ((_animB - _startB) * elapsed) / _animPeriod;
      sendPixel(applyBrightness(r), applyBrightness(g), applyBrightness(b));
    }
    break;
  }

  case ANIM_COLOR_WAVE: {
    // Smooth transition between two colors
    float wave = (sin(phase * 2 * PI) + 1.0f) / 2.0f;
    r = _animR + ((_animR2 - _animR) * (uint8_t)(wave * 255)) / 255;
    g = _animG + ((_animG2 - _animG) * (uint8_t)(wave * 255)) / 255;
    b = _animB + ((_animB2 - _animB) * (uint8_t)(wave * 255)) / 255;
    sendPixel(applyBrightness(r), applyBrightness(g), applyBrightness(b));
    break;
  }

  case ANIM_STROBE: {
    uint32_t strobePhase = elapsed % (_animPeriod * 2);
    if (strobePhase < _animPeriod) {
      sendPixel(applyBrightness(_animR), applyBrightness(_animG),
                applyBrightness(_animB));
    } else {
      sendPixel(0, 0, 0);
    }
    break;
  }

  case ANIM_CANDLE: {
    // Random flicker effect (warm colors)
    static uint32_t lastFlicker = 0;
    static uint8_t flickerLevel = 200;
    if (millis() - lastFlicker > 50 + random(100)) {
      flickerLevel = 150 + random(105);
      lastFlicker = millis();
    }
    r = flickerLevel;
    g = flickerLevel / 3;
    b = 0;
    sendPixel(applyBrightness(r), applyBrightness(g), applyBrightness(b));
    break;
  }

  case ANIM_NOTIFICATION: {
    // Same as breathing for notification
    float breath = (sin(phase * 2 * PI - PI / 2) + 1.0f) / 2.0f;
    uint8_t br = (uint8_t)(breath * 255);
    r = (_animR * br) / 255;
    g = (_animG * br) / 255;
    b = (_animB * br) / 255;
    sendPixel(applyBrightness(r), applyBrightness(g), applyBrightness(b));
    break;
  }

  default:
    break;
  }
}

// === Private: endNotification ===
void RGBLedHandler::endNotification() {
  _notifying = false;
  // Restore previous animation
  _animType = _savedAnimType;
  _animR = _savedR;
  _animG = _savedG;
  _animB = _savedB;
  _animPeriod = _savedPeriod;
  _animStartTime = millis();
}

// === Public: Immediate Color ===
void RGBLedHandler::setColor(uint8_t r, uint8_t g, uint8_t b) {
  _animType = ANIM_NONE;
  sendPixel(applyBrightness(r), applyBrightness(g), applyBrightness(b));
}

void RGBLedHandler::setColorHSV(uint16_t hue, uint8_t sat, uint8_t val) {
  uint8_t r, g, b;
  hsvToRgb(hue, sat, val, &r, &g, &b);
  setColor(r, g, b);
}

void RGBLedHandler::setBrightness(uint8_t brightness) {
  _brightness = brightness;
}

void RGBLedHandler::off() {
  _animType = ANIM_NONE;
  sendPixel(0, 0, 0);
}

// === Preset Colors ===
void RGBLedHandler::red() { setColor(255, 0, 0); }
void RGBLedHandler::green() { setColor(0, 255, 0); }
void RGBLedHandler::blue() { setColor(0, 0, 255); }
void RGBLedHandler::white() { setColor(255, 255, 255); }
void RGBLedHandler::yellow() { setColor(255, 255, 0); }
void RGBLedHandler::cyan() { setColor(0, 255, 255); }
void RGBLedHandler::magenta() { setColor(255, 0, 255); }
void RGBLedHandler::orange() { setColor(255, 128, 0); }
void RGBLedHandler::purple() { setColor(128, 0, 255); }

// === Start Animation Functions ===
void RGBLedHandler::startRainbow(uint32_t periodMs) {
  _animType = ANIM_RAINBOW;
  _animPeriod = periodMs;
  _animStartTime = millis();
}

void RGBLedHandler::startBreathing(uint8_t r, uint8_t g, uint8_t b,
                                   uint32_t periodMs) {
  _animType = ANIM_BREATHING;
  _animR = r;
  _animG = g;
  _animB = b;
  _animPeriod = periodMs;
  _animStartTime = millis();
}

void RGBLedHandler::startPulse(uint8_t r, uint8_t g, uint8_t b,
                               uint32_t periodMs) {
  _animType = ANIM_PULSE;
  _animR = r;
  _animG = g;
  _animB = b;
  _animPeriod = periodMs;
  _animStartTime = millis();
}

void RGBLedHandler::startBlink(uint8_t r, uint8_t g, uint8_t b, uint32_t onMs,
                               uint32_t offMs) {
  _animType = ANIM_BLINK;
  _animR = r;
  _animG = g;
  _animB = b;
  _animPeriod = onMs;
  _animParam2 = offMs;
  _animStartTime = millis();
}

void RGBLedHandler::startFadeTo(uint8_t r, uint8_t g, uint8_t b,
                                uint32_t durationMs) {
  _animType = ANIM_FADE_TO;
  _startR = _currentR;
  _startG = _currentG;
  _startB = _currentB;
  _animR = r;
  _animG = g;
  _animB = b;
  _animPeriod = durationMs;
  _animStartTime = millis();
}

void RGBLedHandler::startColorWave(uint8_t r1, uint8_t g1, uint8_t b1,
                                   uint8_t r2, uint8_t g2, uint8_t b2,
                                   uint32_t periodMs) {
  _animType = ANIM_COLOR_WAVE;
  _animR = r1;
  _animG = g1;
  _animB = b1;
  _animR2 = r2;
  _animG2 = g2;
  _animB2 = b2;
  _animPeriod = periodMs;
  _animStartTime = millis();
}

void RGBLedHandler::startStrobe(uint8_t r, uint8_t g, uint8_t b,
                                uint32_t flashMs) {
  _animType = ANIM_STROBE;
  _animR = r;
  _animG = g;
  _animB = b;
  _animPeriod = flashMs;
  _animStartTime = millis();
}

void RGBLedHandler::startCandle() {
  _animType = ANIM_CANDLE;
  _animPeriod = 100;
  _animStartTime = millis();
}

void RGBLedHandler::startCustom(RGBAnimationType type, uint8_t r, uint8_t g,
                                uint8_t b, uint32_t param1, uint32_t param2) {
  _animType = type;
  _animR = r;
  _animG = g;
  _animB = b;
  _animPeriod = param1;
  _animParam2 = param2;
  _animStartTime = millis();
}

// === Animation Control ===
void RGBLedHandler::stopAnimation() {
  _animType = ANIM_NONE;
  _notifying = false;
}

void RGBLedHandler::pauseAnimation() { _animPaused = true; }
void RGBLedHandler::resumeAnimation() { _animPaused = false; }
bool RGBLedHandler::isAnimating() { return _animType != ANIM_NONE; }
RGBAnimationType RGBLedHandler::getCurrentAnimation() { return _animType; }

// === Notification System ===
void RGBLedHandler::notify(uint8_t r, uint8_t g, uint8_t b, uint32_t durationMs,
                           RGBNotifyPriority priority) {
  if (_notifying && priority < _notifyPriority)
    return;

  // Save current state
  if (!_notifying) {
    _savedAnimType = _animType;
    _savedR = _animR;
    _savedG = _animG;
    _savedB = _animB;
    _savedPeriod = _animPeriod;
  }

  _notifying = true;
  _notifyPriority = priority;
  _notifyEndTime = millis() + durationMs;

  _animType = ANIM_NOTIFICATION;
  _animR = r;
  _animG = g;
  _animB = b;
  _animPeriod = 500; // Fast breathing
  _animStartTime = millis();
}

void RGBLedHandler::notifyBlink(uint8_t r, uint8_t g, uint8_t b, int times,
                                uint32_t intervalMs,
                                RGBNotifyPriority priority) {
  if (_notifying && priority < _notifyPriority)
    return;

  if (!_notifying) {
    _savedAnimType = _animType;
    _savedR = _animR;
    _savedG = _animG;
    _savedB = _animB;
    _savedPeriod = _animPeriod;
  }

  _notifying = true;
  _notifyPriority = priority;
  _notifyEndTime = millis() + (times * intervalMs * 2);

  startBlink(r, g, b, intervalMs, intervalMs);
}

void RGBLedHandler::notifyPulse(uint8_t r, uint8_t g, uint8_t b, int times,
                                RGBNotifyPriority priority) {
  if (_notifying && priority < _notifyPriority)
    return;

  if (!_notifying) {
    _savedAnimType = _animType;
    _savedR = _animR;
    _savedG = _animG;
    _savedB = _animB;
    _savedPeriod = _animPeriod;
  }

  _notifying = true;
  _notifyPriority = priority;
  _notifyEndTime = millis() + (times * 500);

  startPulse(r, g, b, 500);
}

// === Status Indicators ===
void RGBLedHandler::statusOK() { notifyBlink(0, 255, 0, 2, 150); }
void RGBLedHandler::statusWarning() { notifyPulse(255, 200, 0, 3); }
void RGBLedHandler::statusError() {
  notifyBlink(255, 0, 0, 5, 100, PRIORITY_HIGH);
}
void RGBLedHandler::statusBusy() { startBreathing(0, 100, 255, 1500); }
void RGBLedHandler::statusIdle() { setColor(0, 50, 50); }
void RGBLedHandler::statusConnected() { notifyPulse(0, 255, 0, 2); }
void RGBLedHandler::statusDisconnected() { notifyPulse(255, 0, 0, 2); }
void RGBLedHandler::statusNFC() { notifyBlink(0, 100, 255, 1, 100); }
void RGBLedHandler::statusGPS() { notifyBlink(0, 255, 200, 1, 100); }

// === Getters ===
uint8_t RGBLedHandler::getBrightness() { return _brightness; }
uint8_t RGBLedHandler::getCurrentR() { return _currentR; }
uint8_t RGBLedHandler::getCurrentG() { return _currentG; }
uint8_t RGBLedHandler::getCurrentB() { return _currentB; }
