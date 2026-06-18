#ifndef NFC_LOG_MANAGER_H
#define NFC_LOG_MANAGER_H

#include <Arduino.h>

#define FMS_LOG_DEBUG 0
#define FMS_LOG_INFO 1
#define FMS_LOG_WARN 2
#define FMS_LOG_ERROR 3
#define FMS_LOG_CRITICAL 4

struct FMSLogEntry {
  uint32_t timestamp_ms;
  const char *datetime;
  int level;
  const char *source;
  const char *message;
  char extra[64];
};

class LogManager {
public:
  void begin();
  void debug(const char *source, const char *message,
             const char *extra = nullptr);
  void info(const char *source, const char *message,
            const char *extra = nullptr);
  void warn(const char *source, const char *message,
            const char *extra = nullptr);
  void error(const char *source, const char *message,
             const char *extra = nullptr);

private:
  void print(const char *level, const char *source, const char *message,
             const char *extra);
};

extern LogManager logManager;

#endif // NFC_LOG_MANAGER_H
