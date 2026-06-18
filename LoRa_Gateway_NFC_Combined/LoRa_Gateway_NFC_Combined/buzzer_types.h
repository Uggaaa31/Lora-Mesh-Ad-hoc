#ifndef BUZZER_TYPES_H
#define BUZZER_TYPES_H

#include <Arduino.h>

struct BuzzerNote {
  unsigned int freq;
  unsigned int dur_ms;
  unsigned int gap_ms;
};

#endif // BUZZER_TYPES_H
