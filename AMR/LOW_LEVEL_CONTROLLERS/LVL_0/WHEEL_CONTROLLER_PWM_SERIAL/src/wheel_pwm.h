#ifndef WHEEL_PWM_SERIAL_H
#define WHEEL_PWM_SERIAL_H

#include <Arduino.h>

#define MY_PI 3.14159265358979323846 // PI 3.14159265358979323846
#define SPEED_FILTER_SIZE 5
#define LED_BUILTIN 2 // Built-in LED pin for ESP32

// Motor A
#define MA_ENA  32  // Speed control for Motor A1
#define MA_IN1  33
#define MA_IN2  25
#define PWM_CHANNELfrontLeft  0
#define MA_ENB  14  // Speed control for Motor A2
#define MA_IN3  26
#define MA_IN4  27
#define PWM_CHANNELfrontRight  1

// Motor B
#define MB_ENA  23  // Speed control for Motor B1                                                                                                                                                                                           
#define MB_IN1  22
#define MB_IN2  21
#define PWM_CHANNELrearLeft  2
#define MB_ENB  5   // Speed control for Motor B2
#define MB_IN3  19
#define MB_IN4  18
#define PWM_CHANNELrearRight  3

typedef struct WheelPins_t {
  int IN1;
  int IN2;
  int EN;
  int channel;
}WheelPins_t;

typedef enum
{FrontLeft = 0,
  FrontRight,
  RearLeft,
  RearRight}
  WheelName_t;

typedef enum {Forward, Backward, Stop} WheelState_t;

class WheelPWM {
  private:
    WheelPins_t wheel_pins;
    int deadband = 10; // Deadband for PWM output
    const int MAX_PWM = 255; // Maximum PWM value
    const unsigned long MAX_UPDATE_INTERVAL  = 1000; // Update interval in milliseconds
    const float freq = 1000; // Frequency for PWM
    const int resolution = 8; // Resolution for PWM (8 bits)
    WheelName_t wheel_name;

  public:
    WheelState_t wheel_state;
    float Current_PWM;
    float Last_PWM;
    unsigned long last_update_time;

    // Constructor to initialize the wheel with its characteristics
    WheelPWM(WheelPins_t pins,WheelName_t wheel_name);
    void apply_Motor_PWM(int pwm);
    void Init_Motor();
    void stop();
    WheelName_t getWheelName() const { return wheel_name; }
};


#endif // WHEEL_PWM_SERIAL_H