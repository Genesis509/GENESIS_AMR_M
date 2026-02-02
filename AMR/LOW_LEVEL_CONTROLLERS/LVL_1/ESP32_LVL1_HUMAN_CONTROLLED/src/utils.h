#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>


#define LED_BUILTIN 2 // Built-in LED pin for ESP32

typedef struct BlinkConfig_t {
    int times;        // Number of times to blink
    int onDuration;  // Duration for which the LED is ON in milliseconds
    int offDuration; // Duration for which the LED is OFF in milliseconds
} BlinkConfig_t;

void blinkLED(BlinkConfig_t config);



#endif // UTILS_H
