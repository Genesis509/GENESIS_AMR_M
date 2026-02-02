#include "wheel_pwm.h"

WheelPWM::WheelPWM(WheelPins_t pins, WheelName_t wheel_name) {
    // Initialize wheel pins and state
    wheel_pins = pins;
    wheel_state = Stop;
    Current_PWM = 0.0;
    Last_PWM = 0.0;
    last_update_time = millis();
    this->wheel_name = wheel_name;
    Init_Motor();
}

void WheelPWM::apply_Motor_PWM(int pwm) {


  if (pwm > 0) 
    {
        // Forward direction
        pwm += deadband;
        if (pwm > MAX_PWM) pwm = 255;
        ledcWrite(this->wheel_pins.channel, pwm);
        digitalWrite(this->wheel_pins.IN1, LOW);
        digitalWrite(this->wheel_pins.IN2, HIGH);
        this->wheel_state = WheelState_t::Forward;
    } 
  else if (pwm < 0) {
    // Reverse direction
    pwm -= deadband;
    if (pwm < -255) pwm = -MAX_PWM;
    ledcWrite(this->wheel_pins.channel, abs(pwm));
    digitalWrite(this->wheel_pins.IN1,          HIGH);
    digitalWrite(this->wheel_pins.IN2, LOW);
    this->wheel_state = WheelState_t::Backward;
    
  }
  else {
    // Stop
    ledcWrite(this->wheel_pins.channel, 0);
    digitalWrite(this->wheel_pins.IN1, LOW);
    digitalWrite(this->wheel_pins.IN2, LOW);
    this->wheel_state = WheelState_t::Stop;
  }
    this->Current_PWM = pwm;

    this->last_update_time = millis();
}

void WheelPWM::Init_Motor() {
  
  
  // Set motor control pins as outputs
  pinMode(this->wheel_pins.IN1, OUTPUT);
  pinMode(this->wheel_pins.IN2, OUTPUT);
  
  // Initialize PWM for motor control
  ledcSetup(this->wheel_pins.channel, this->freq, this->resolution);
  ledcAttachPin(this->wheel_pins.EN, this->wheel_pins.channel);
}

void WheelPWM::stop() {
  apply_Motor_PWM(0);
  wheel_state = Stop;
  Current_PWM = 0.0;
  Last_PWM = 0.0;
}

