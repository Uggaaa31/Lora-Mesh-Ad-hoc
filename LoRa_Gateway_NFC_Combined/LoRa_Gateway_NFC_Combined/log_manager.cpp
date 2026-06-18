#include "log_manager.h"

LogManager logManager;

void LogManager::begin() { Serial.println("[LOG] NFC standalone logger ready"); }

void LogManager::debug(const char *source, const char *message,
                       const char *extra) {
  print("DEBUG", source, message, extra);
}

void LogManager::info(const char *source, const char *message,
                      const char *extra) {
  print("INFO", source, message, extra);
}

void LogManager::warn(const char *source, const char *message,
                      const char *extra) {
  print("WARN", source, message, extra);
}

void LogManager::error(const char *source, const char *message,
                       const char *extra) {
  print("ERROR", source, message, extra);
}

void LogManager::print(const char *level, const char *source,
                       const char *message, const char *extra) {
  Serial.printf("[%s][%s] %s", level, source ? source : "-", message ? message : "");
  if (extra != nullptr && extra[0] != '\0') {
    Serial.printf(" | %s", extra);
  }
  Serial.println();
}
