#include "nfc_integration.h"

#include "buzzer_types.h"
#include "config_manager.h"
#include "config.h"
#include "led_status_manager.h"
#include "log_manager.h"
#include "pn532_handler.h"
#include "rgb_led_handler.h"
#include <WiFi.h>

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "fms_mqtt.h"
extern "C" {
#include "esp_event.h"
}

// Pin dan bus dibuat sama dengan firmware FMS utama.
#define BUZZER_PIN 13
#define LEDC_RESOLUTION 8
#define PWM_FREQ 2000

#define PN532_RESET_PIN 4
#define PN532_SDA_PIN 1
#define PN532_SCL_PIN 2

#define RGB_LED_PIN_STANDALONE 48

bool DEBUG_ALL = false;
bool DEBUG_NFC = true;
bool DEBUG_NETWORK = true;

// extern esp_mqtt_client_handle_t mqtt_client; // REMOVED
extern volatile bool mqttConnected;

TwoWire Wire1_Custom = TwoWire(1);
PN532Handler pn532(Wire1_Custom, PN532_RESET_PIN);

RGBLedHandler rgbLed(RGB_LED_PIN_STANDALONE);
LedStatusManager ledStatus(rgbLed);

bool g_pn532Ready = false;
bool rgbEnabled = true;

static const bool NFC_POPUP_ENABLED = true;
static const uint32_t NFC_POPUP_SHOW_DURATION_MS = 3000;
static const uint32_t NFC_POPUP_POST_BUZZER_DELAY_MS = 120;
static bool g_nfcPopupPending = false;
static bool g_nfcPopupPendingValid = false;
static uint32_t g_nfcPopupPendingDurationMs = NFC_POPUP_SHOW_DURATION_MS;
static bool g_nfcPopupWaitingBuzzerDone = false;
static uint32_t g_nfcPopupReadyAtMs = 0;

bool shouldPlayPN532Beep = false;
unsigned long pn532BeepTime = 0;
unsigned long lastNFCTapMs = 0;

const BuzzerNote SEQ_SUCCESS_BEEP[] = {
  {2500, 100, 150},
  {2500, 100, 0},
};

const BuzzerNote SEQ_SINGLE_BEEP[] = {
  {2500, 80, 0},
};

const BuzzerNote SEQ_DOUBLE_BEEP[] = {
  {2500, 80, 80},
  {2500, 80, 0},
};

const BuzzerNote SEQ_NFC_TAP[] = {
  {2700, 50, 0},
};

const BuzzerNote SEQ_NFC_VALID[] = {
  {2200, 50, 30},
  {2200, 50, 40},
  {2600, 90, 0},
};

const BuzzerNote SEQ_NFC_FAIL[] = {
  {2700, 160, 60},
  {2000, 70, 0},
};

enum BuzzerState {
  BUZZER_IDLE,
  BUZZER_PLAYING_NOTE,
  BUZZER_PLAYING_GAP
};

struct BuzzerController {
  BuzzerState state;
  const BuzzerNote *sequence;
  size_t sequenceLength;
  size_t currentNoteIndex;
  unsigned long stateStartTime;
  bool isPlaying;

  BuzzerController()
      : state(BUZZER_IDLE), sequence(nullptr), sequenceLength(0),
        currentNoteIndex(0), stateStartTime(0), isPlaying(false) {}
};

BuzzerController buzzer;
SemaphoreHandle_t buzzerMutex = nullptr;

static bool publishNfcPayload(const String &payload) {
  if (!fmsMqtt.isConnected()) {
    return false;
  }

  return fmsMqtt.publishToTopic(MQTT_TOPIC_MASTER, payload);
}

void syncDebugFlags() {
  const FMSConfig &cfg = configManager.getConfig();
  DEBUG_ALL = cfg.debug_all;
  DEBUG_NFC = cfg.debug_nfc;
}

void initBuzzer() {
  ledcAttach(BUZZER_PIN, PWM_FREQ, LEDC_RESOLUTION);
  ledcWrite(BUZZER_PIN, 0);
  buzzerMutex = xSemaphoreCreateMutex();
  if (buzzerMutex == nullptr) {
    Serial.println("[BUZZER] Failed to create mutex");
  }
}

void playTone(unsigned int freq) {
  if (freq == 0) {
    ledcWrite(BUZZER_PIN, 0);
    return;
  }
  ledcChangeFrequency(BUZZER_PIN, freq, LEDC_RESOLUTION);
  ledcWrite(BUZZER_PIN, 128);
}

void stopTone() { ledcWrite(BUZZER_PIN, 0); }

void startBuzzerSequence(const BuzzerNote *seq, size_t len) {
  if (seq == nullptr || len == 0 || buzzerMutex == nullptr) {
    return;
  }

  if (xSemaphoreTake(buzzerMutex, portMAX_DELAY) == pdTRUE) {
    buzzer.isPlaying = false;
    buzzer.state = BUZZER_IDLE;
    stopTone();

    buzzer.sequence = seq;
    buzzer.sequenceLength = len;
    buzzer.currentNoteIndex = 0;
    buzzer.isPlaying = true;
    buzzer.state = BUZZER_PLAYING_NOTE;
    buzzer.stateStartTime = micros();

    playTone(seq[0].freq);
    xSemaphoreGive(buzzerMutex);
  }
}

void updateBuzzer() {
  if (buzzerMutex == nullptr || xSemaphoreTake(buzzerMutex, 0) != pdTRUE) {
    return;
  }

  if (!buzzer.isPlaying || buzzer.sequence == nullptr ||
      buzzer.currentNoteIndex >= buzzer.sequenceLength) {
    buzzer.isPlaying = false;
    buzzer.state = BUZZER_IDLE;
    stopTone();
    xSemaphoreGive(buzzerMutex);
    return;
  }

  const unsigned long now = micros();
  const BuzzerNote &currentNote = buzzer.sequence[buzzer.currentNoteIndex];

  switch (buzzer.state) {
  case BUZZER_PLAYING_NOTE:
    if (now - buzzer.stateStartTime >= (currentNote.dur_ms * 1000UL)) {
      stopTone();
      if (currentNote.gap_ms > 0) {
        buzzer.state = BUZZER_PLAYING_GAP;
        buzzer.stateStartTime = now;
      } else {
        buzzer.currentNoteIndex++;
        if (buzzer.currentNoteIndex < buzzer.sequenceLength) {
          buzzer.state = BUZZER_PLAYING_NOTE;
          buzzer.stateStartTime = now;
          playTone(buzzer.sequence[buzzer.currentNoteIndex].freq);
        } else {
          buzzer.isPlaying = false;
          buzzer.state = BUZZER_IDLE;
          stopTone();
        }
      }
    }
    break;

  case BUZZER_PLAYING_GAP:
    if (now - buzzer.stateStartTime >= (currentNote.gap_ms * 1000UL)) {
      buzzer.currentNoteIndex++;
      if (buzzer.currentNoteIndex < buzzer.sequenceLength) {
        buzzer.state = BUZZER_PLAYING_NOTE;
        buzzer.stateStartTime = now;
        playTone(buzzer.sequence[buzzer.currentNoteIndex].freq);
      } else {
        buzzer.isPlaying = false;
        buzzer.state = BUZZER_IDLE;
        stopTone();
      }
    }
    break;

  default:
    buzzer.isPlaying = false;
    buzzer.state = BUZZER_IDLE;
    stopTone();
    break;
  }

  xSemaphoreGive(buzzerMutex);
}

void stopBuzzer() {
  if (buzzerMutex == nullptr) {
    return;
  }
  if (xSemaphoreTake(buzzerMutex, portMAX_DELAY) == pdTRUE) {
    buzzer.isPlaying = false;
    buzzer.state = BUZZER_IDLE;
    stopTone();
    xSemaphoreGive(buzzerMutex);
  }
}

void playSuccessBeep() {
  startBuzzerSequence(SEQ_SUCCESS_BEEP,
                      sizeof(SEQ_SUCCESS_BEEP) / sizeof(BuzzerNote));
}

void playSingleBeep() {
  startBuzzerSequence(SEQ_SINGLE_BEEP,
                      sizeof(SEQ_SINGLE_BEEP) / sizeof(BuzzerNote));
}

void playDoubleBeep() {
  startBuzzerSequence(SEQ_DOUBLE_BEEP,
                      sizeof(SEQ_DOUBLE_BEEP) / sizeof(BuzzerNote));
}

void playTagTapTone() {
  startBuzzerSequence(SEQ_NFC_TAP, sizeof(SEQ_NFC_TAP) / sizeof(BuzzerNote));
}

void playTagValidTone() {
  startBuzzerSequence(SEQ_NFC_VALID,
                      sizeof(SEQ_NFC_VALID) / sizeof(BuzzerNote));
}

void playTagFailTone() {
  startBuzzerSequence(SEQ_NFC_FAIL, sizeof(SEQ_NFC_FAIL) / sizeof(BuzzerNote));
}

void sendUiEventJson(const char *eventName) {
  Serial.printf("[DATA]{\"type\":\"event\",\"event\":\"%s\",\"timestamp_ms\":%lu}\n",
                eventName, millis());
}

void showNfcPopup(bool valid, uint32_t durationMs = NFC_POPUP_SHOW_DURATION_MS) {
  Serial.printf("[NFC][POPUP] NFC CARD %s (%lu ms)\n",
                valid ? "valid" : "cancel", (unsigned long)durationMs);
}

void hideNfcPopup() {
  // Placeholder agar alur event sama dengan firmware utama saat LCD tidak ada.
}

void lcdQueueIncomingChatMessage(const char *msg, size_t len) {
  Serial.printf("[MQTT][CHAT] %.*s\n", (int)len, msg);
}

void nfcPrintStatus() {
  Serial.println("\n[NFC] === STATUS ===");
  Serial.printf("[NFC] ready=%s, card_present=%s, validated=%s\n",
                g_pn532Ready ? "YES" : "NO",
                pn532.isCardPresent ? "YES" : "NO",
                pn532.isValidated ? "YES" : "NO");
  Serial.printf("[NFC] last_uid=%s, last_tap_ms=%lu\n",
                pn532.getLastUID().c_str(), lastNFCTapMs);
  Serial.printf("[NFC] debug_all=%s, debug_nfc=%s\n",
                DEBUG_ALL ? "ON" : "OFF", DEBUG_NFC ? "ON" : "OFF");
  Serial.printf("[NFC] pins: SDA=%d, SCL=%d, RST=%d, buzzer=%d, rgb=%d\n",
                PN532_SDA_PIN, PN532_SCL_PIN, PN532_RESET_PIN, BUZZER_PIN,
                RGB_LED_PIN_STANDALONE);
}

void nfcPrintHelp() {
  Serial.println("\n=== NFC Commands ===");
  Serial.println("  h / help        - Show this help");
  Serial.println("  status nfc      - Show NFC status");
  Serial.println("  uid             - Print last UID");
  Serial.println("  reset nfc       - Hardware reset PN532");
  Serial.println("  debug nfc       - Toggle NFC debug and save");
  Serial.println("  debug all       - Toggle master debug and save");
  Serial.println("  debug status    - Show debug flags");
  Serial.println("  rgb on          - Enable RGB status LED");
  Serial.println("  rgb off         - Disable RGB status LED");
  Serial.println("  n               - Play NFC tap tone");
  Serial.println("  v               - Play NFC valid tone");
  Serial.println("  f               - Play NFC fail tone");
  Serial.println("  d               - Play double beep");
  Serial.println("  u               - Play success beep");
  Serial.println("  s               - Stop buzzer");
}

bool nfcHandleSerialCommand(const String &input) {
  if (input.equalsIgnoreCase("h") || input.equalsIgnoreCase("help")) {
    nfcPrintHelp();
  } else if (input.equalsIgnoreCase("status")) {
    nfcPrintStatus();
  } else if (input.equalsIgnoreCase("status nfc") ||
             input.equalsIgnoreCase("nfc status")) {
    nfcPrintStatus();
  } else if (input.equalsIgnoreCase("uid")) {
    pn532.printLastUID();
  } else if (input.equalsIgnoreCase("reset nfc")) {
    Serial.println("[NFC] Manual reset command");
    if (g_pn532Ready) {
      pn532.hardwareReset();
    } else {
      g_pn532Ready = pn532.begin();
      Serial.println(g_pn532Ready ? "[NFC] PN532 now ready"
                                  : "[NFC] PN532 still failed");
    }
  } else if (input.equalsIgnoreCase("debug nfc")) {
    configManager.setDebugNfc(!configManager.getConfig().debug_nfc);
    configManager.saveDebugFlags();
    syncDebugFlags();
    Serial.printf("[NFC] DEBUG_NFC=%s\n", DEBUG_NFC ? "ON" : "OFF");
  } else if (input.equalsIgnoreCase("debug all")) {
    configManager.setDebugAll(!configManager.getConfig().debug_all);
    configManager.saveDebugFlags();
    syncDebugFlags();
    Serial.printf("[NFC] DEBUG_ALL=%s\n", DEBUG_ALL ? "ON" : "OFF");
  } else if (input.equalsIgnoreCase("debug status")) {
    Serial.println(configManager.getDebugFlagsJson());
  } else if (input.equalsIgnoreCase("rgb on")) {
    rgbEnabled = true;
    Serial.println("[RGB] enabled");
  } else if (input.equalsIgnoreCase("rgb off")) {
    rgbEnabled = false;
    rgbLed.off();
    Serial.println("[RGB] disabled");
  } else if (input.equalsIgnoreCase("n")) {
    playTagTapTone();
  } else if (input.equalsIgnoreCase("v")) {
    playTagValidTone();
  } else if (input.equalsIgnoreCase("f")) {
    playTagFailTone();
  } else if (input.equalsIgnoreCase("d")) {
    playDoubleBeep();
  } else if (input.equalsIgnoreCase("u")) {
    playSuccessBeep();
  } else if (input.equalsIgnoreCase("s")) {
    stopBuzzer();
  } else {
    return false;
  }

  return true;
}

void handleNfcEvents() {
  bool tagJustValidated = pn532.isTagAvailable();
  if (tagJustValidated) {
    lastNFCTapMs = millis();
  }

  if (shouldPlayPN532Beep && millis() >= pn532BeepTime) {
    Serial.println("[NFC] PN532 startup success beep");
    playSuccessBeep();
    shouldPlayPN532Beep = false;
  }

  if (pn532.justDetected) {
    pn532.justDetected = false;
    playTagTapTone();
  }

  if (pn532.justValidated) {
    pn532.justValidated = false;
    sendUiEventJson("nfc_valid");
    playTagValidTone();
    ledStatus.triggerNFCValidTap();
    Serial.printf("[NFC] VALID UID=%s\n", pn532.getLastUID().c_str());

    if (fmsMqtt.isConnected()) {
      DynamicJsonDocument doc(256);
      doc["type"] = "nfc_valid";
      doc["uid"] = pn532.getLastUID();
      doc["timestamp_ms"] = millis();
      
      String payload;
      serializeJson(doc, payload);
      if (publishNfcPayload(payload)) {
        Serial.printf("[MQTT] Published NFC UID to %s\n", MQTT_TOPIC_MASTER);
      } else {
        Serial.println("[MQTT] Publish NFC failed");
      }
    } else {
      Serial.println("[MQTT] Not connected, skipped publishing");
    }

    if (NFC_POPUP_ENABLED) {
      g_nfcPopupPending = true;
      g_nfcPopupPendingValid = true;
      g_nfcPopupPendingDurationMs = NFC_POPUP_SHOW_DURATION_MS;
      g_nfcPopupWaitingBuzzerDone = true;
      g_nfcPopupReadyAtMs = 0;
      hideNfcPopup();
    }
  }

  if (pn532.justReinitialized) {
    pn532.justReinitialized = false;
    Serial.println("[NFC] PN532 has been re-initialized");
    playDoubleBeep();
    ledStatus.triggerNFCHardwareReset();
  }

  if (pn532.justRecovered) {
    pn532.justRecovered = false;
    Serial.println("[NFC] PN532 soft RF recovery performed");
  }

  if (pn532.justStandbyOK) {
    pn532.justStandbyOK = false;
    ledStatus.triggerNFCStandbyOK();
  }

  if (pn532.i2cMissed) {
    ledStatus.triggerNFCMissed(true);
  } else {
    ledStatus.triggerNFCMissed(false);
  }

  if (pn532.validationCanceled) {
    pn532.validationCanceled = false;
    sendUiEventJson("nfc_cancel");
    ledStatus.triggerNFCCanceled();
    if (NFC_POPUP_ENABLED) {
      g_nfcPopupPending = true;
      g_nfcPopupPendingValid = false;
      g_nfcPopupPendingDurationMs = NFC_POPUP_SHOW_DURATION_MS;
      g_nfcPopupWaitingBuzzerDone = true;
      g_nfcPopupReadyAtMs = 0;
      hideNfcPopup();
    }
  }

  if (NFC_POPUP_ENABLED && g_nfcPopupPending) {
    if (g_nfcPopupWaitingBuzzerDone) {
      if (!buzzer.isPlaying) {
        g_nfcPopupWaitingBuzzerDone = false;
        g_nfcPopupReadyAtMs = millis() + NFC_POPUP_POST_BUZZER_DELAY_MS;
      }
    } else if ((int32_t)(millis() - g_nfcPopupReadyAtMs) >= 0) {
      showNfcPopup(g_nfcPopupPendingValid, g_nfcPopupPendingDurationMs);
      g_nfcPopupPending = false;
    }
  }
}

bool nfcIsBrokerConnected() {
  return fmsMqtt.isConnected();
}

void nfcSubsystemSetup() {
  configManager.begin();
  syncDebugFlags();
  logManager.begin();

  initBuzzer();

  Wire1_Custom.begin(PN532_SDA_PIN, PN532_SCL_PIN);
  Wire1_Custom.setClock(50000);
  Serial.printf("[NFC] I2C Wire1 on SDA=%d, SCL=%d, clock=50kHz\n",
                PN532_SDA_PIN, PN532_SCL_PIN);

  rgbLed.begin();
  ledStatus.setWiFiStatus(false);
  ledStatus.setGPSStatus(false, false);

  g_pn532Ready = pn532.begin();
  if (g_pn532Ready) {
    Serial.println("[NFC] PN532 handler initialized on isolated Wire1");
    shouldPlayPN532Beep = true;
    pn532BeepTime = millis() + 5000;
  } else {
    Serial.println("[NFC] CRITICAL: PN532 handler FAILED");
  }

}

void nfcSubsystemLoop(bool wifiConnected) {
  updateBuzzer();

  if (g_pn532Ready) {
    pn532.loop();
  }

  handleNfcEvents();

  ledStatus.setWiFiStatus(wifiConnected);

  if (rgbEnabled) {
    ledStatus.update();
  } else {
    rgbLed.off();
  }
}
