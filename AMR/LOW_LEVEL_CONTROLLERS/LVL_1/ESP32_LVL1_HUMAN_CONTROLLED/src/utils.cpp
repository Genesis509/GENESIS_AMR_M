#include "utils.h"

void blinkLED(BlinkConfig_t config) {
  for (int i = 0; i < config.times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(config.onDuration);
    digitalWrite(LED_BUILTIN, LOW);
    if (i < config.times - 1 || config.offDuration > 0) {
      delay(config.offDuration);
    }
  }
}