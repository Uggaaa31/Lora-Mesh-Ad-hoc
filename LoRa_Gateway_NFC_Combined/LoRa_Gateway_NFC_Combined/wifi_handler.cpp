#include "wifi_handler.h"

void WiFiHelper::disconnect() {
  WiFi.disconnect(false, false); // disconnect, erase wifi config
  Serial.println("WiFi: disconnected manually.");
}