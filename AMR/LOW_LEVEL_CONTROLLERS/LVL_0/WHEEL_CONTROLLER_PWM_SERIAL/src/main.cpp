#include <Arduino.h>
#include "wheel_pwm.h"
#include "utils.h"
#include <esp_task_wdt.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>


// Define wheel pins -----------------------------------------------------------------------------------------------
WheelPins_t rearRightPins  = {MA_IN1, MA_IN2, MA_ENA,PWM_CHANNELrearRight };
WheelPins_t rearLeftPins = {MA_IN3, MA_IN4, MA_ENB, PWM_CHANNELrearLeft};
WheelPins_t  frontRightPins = {MB_IN1, MB_IN2, MB_ENA, PWM_CHANNELfrontRight };
WheelPins_t  frontLeftPins = {MB_IN3, MB_IN4, MB_ENB,PWM_CHANNELfrontLeft };

WheelPWM frontLeftWheel  (frontLeftPins , WheelName_t::FrontLeft );
WheelPWM frontRightWheel(frontRightPins, WheelName_t::FrontRight);
WheelPWM rearLeftWheel(rearLeftPins, WheelName_t::RearLeft);
WheelPWM rearRightWheel(rearRightPins, WheelName_t::RearRight);

 BlinkConfig_t Init_Blink = {3, 250, 250}; // Global configuration for blinking
 BlinkConfig_t Error_Blink = {5, 100, 100}; // Global configuration for error blinking
 BlinkConfig_t Success_Blink = {2, 200, 200}; // Global configuration for success blinking
 BlinkConfig_t waiting_Blink = {3, 600, 600}; // Global configuration for long blinking
 BlinkConfig_t retyr_Blink = {2, 100, 100}; // Global configuration for heartbeat blinking
 BlinkConfig_t heartbeat_Blink = {1, 50, 0}; // Global configuration for heartbeat blinking

// ESP_NOW communication setup --------------------------------------------------------------------------------

// Data structure
typedef struct __attribute__((packed)) PWM_data_t {
  int frontLeftPWM;
  int frontRightPWM;
  int rearLeftPWM;
  int rearRightPWM;
} PWM_data_t;


bool isConnected = false;

// LED state management  
bool ledState = false;
unsigned long lastLedUpdate = 0;
const unsigned long LED_UPDATE_INTERVAL = 100;
unsigned long lastPacketTime = 0;
const unsigned long CONNECTION_TIMEOUT = 200; // Short timeout
bool debugmode = false;

PWM_data_t wheel_pwm = {0, 0, 0, 0}; // PWM values for each wheel


void IRAM_ATTR onTimer();
void checkConnection();
void set_motor_speeds();
void emergency_stop();
void test_motors();
void updateLED() ;
void checkTimeout();
// Timer interrupt handler



void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(PWM_data_t)) return;
  
  PWM_data_t temp;
  memcpy(&temp, incomingData, sizeof(PWM_data_t));
  
  // Validate and apply
  if (temp.frontLeftPWM >= -255 && temp.frontLeftPWM <= 255 &&
      temp.frontRightPWM >= -255 && temp.frontRightPWM <= 255 &&
      temp.rearLeftPWM >= -255 && temp.rearLeftPWM <= 255 &&
      temp.rearRightPWM >= -255 && temp.rearRightPWM <= 255) {
    wheel_pwm = temp;
    lastPacketTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    ESP.restart();
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  
  // Initialize wheels
  frontLeftWheel.stop();
  frontRightWheel.stop();
  rearLeftWheel.stop();
  rearRightWheel.stop();
  
  Serial.println("Receiver ready");
  Serial.println(WiFi.macAddress());
  blinkLED(Init_Blink);
}

void loop() {
  esp_task_wdt_reset();
  
  // Check for timeout - zero PWM if no data
  checkTimeout();
  
  // Apply motor speeds
  set_motor_speeds();
  
  // Update LED state (debounced)
  updateLED();
  
  // Debug print every 500ms
  static unsigned long lastPrint = 0;
  if (debugmode && millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf(">FL:%d >FR:%d >RL:%d >RR:%d\n",
      wheel_pwm.frontLeftPWM, wheel_pwm.frontRightPWM,
      wheel_pwm.rearLeftPWM, wheel_pwm.rearRightPWM);
  }
  
  delay(10);
}

void updateLED() {
  unsigned long now = millis();
  if (now - lastLedUpdate < LED_UPDATE_INTERVAL) return;
  lastLedUpdate = now;
  
  // LED ON if we received data within timeout
  bool shouldBeOn = (now - lastPacketTime < CONNECTION_TIMEOUT);
  if (shouldBeOn != ledState) {
    ledState = shouldBeOn;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }
}

void checkTimeout() {
  // If no data received recently, set PWM to zero
  if (millis() - lastPacketTime > CONNECTION_TIMEOUT) {
    wheel_pwm = {0, 0, 0, 0};
  }
}

void checkConnection() {
  unsigned long currentTime = millis();
  static unsigned long lastBlinkTime = 0;

  // Check if connection timed out
  if (isConnected && (currentTime - lastPacketTime > CONNECTION_TIMEOUT)) {
    isConnected = false;
    Serial.println("Connection lost! Stopping all wheels.");
    emergency_stop();
    //digitalWrite(LED_BUILTIN, LOW); // LED OFF when disconnected
    digitalWrite(LED_BUILTIN, LOW); // LED OFF when disconnected
    
    //blinkLED(Error_Blink); // Rapid blinking indicates error
  }
  
  // If disconnected, blink LED periodically (heartbeat pattern)
  
  else if (!isConnected && (currentTime - lastBlinkTime > 1000)) {
    lastBlinkTime = currentTime;
    //blinkLED(heartbeat_Blink); // Quick single blink for heartbeat
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if (isConnected) {
    // If connected, keep LED on
    digitalWrite(LED_BUILTIN, HIGH); // LED ON when connected
    lastBlinkTime = currentTime; // Reset blink time
  }
}

void set_motor_speeds() {
    frontLeftWheel.apply_Motor_PWM(wheel_pwm.frontLeftPWM);
    frontRightWheel.apply_Motor_PWM(wheel_pwm.frontRightPWM);
    rearLeftWheel.apply_Motor_PWM(wheel_pwm.rearLeftPWM);
    rearRightWheel.apply_Motor_PWM(wheel_pwm.rearRightPWM); 
    
}

void test_motors() {
  frontLeftWheel.apply_Motor_PWM(100);
  frontRightWheel.apply_Motor_PWM(0);
  rearLeftWheel.apply_Motor_PWM(0);
  rearRightWheel.apply_Motor_PWM(0);
  delay(1000);
  frontLeftWheel.apply_Motor_PWM(0);
  frontRightWheel.apply_Motor_PWM(100);
  rearLeftWheel.apply_Motor_PWM(0);
  rearRightWheel.apply_Motor_PWM(0);
  delay(1000);
  frontLeftWheel.apply_Motor_PWM(0);
  frontRightWheel.apply_Motor_PWM(0);
  rearLeftWheel.apply_Motor_PWM(100);
  rearRightWheel.apply_Motor_PWM(0);
  delay(1000);
  frontLeftWheel.apply_Motor_PWM(0);
  frontRightWheel.apply_Motor_PWM(0);
  rearLeftWheel.apply_Motor_PWM(0);
  rearRightWheel.apply_Motor_PWM(100);
  delay(1000);
}


void emergency_stop() {
  frontLeftWheel.stop();
  frontRightWheel.stop();
  rearLeftWheel.stop();
  rearRightWheel.stop();
}
