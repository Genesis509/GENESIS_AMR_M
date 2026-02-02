#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_now.h>
#include <WiFi.h>
#include "utils.h" 
#include <esp_wifi.h>
#define RXD_PIN 16  // GPIO 16 as RX
#define TXD_PIN 17  // GPIO 17 as TX

BlinkConfig_t Init_Blink = {3, 250, 250}; // Global configuration for blinking

#define LED_PIN 2
// Create hardware serial instance
const uint8_t ESP_LVL_0_MAC_Address[] = {0x3C, 0x8A, 0x1F, 0x5C, 0xA0, 0xB4}; // Receiver MAC address

// Data structure (exact match with receiver)
typedef struct __attribute__((packed)) PWM_data_t {
  int frontLeftPWM;
  int frontRightPWM;
  int rearLeftPWM;
  int rearRightPWM;
} PWM_data_t;

 typedef struct Speed_vector_t {
  float vx;
  float vy;
  float vz;
} Speed_vector_t;

Speed_vector_t speed_vector = {0.0, 0.0, 0.0}; // Initialize speed vector

//Stop if message not received in this time (ms)
const unsigned long MESSAGE_TIMEOUT = 1000;
unsigned long lastMessage_received_Time = 0;
// LED state management
bool ledState = false;
unsigned long lastLedUpdate = 0;
const unsigned long LED_UPDATE_INTERVAL = 100;
unsigned long lastSuccessfulSend = 0;
bool debugmode = false;  // Set to true for debugging, false for normal operation

PWM_data_t wheel_pwm; // PWM values for each wheel

esp_now_peer_info_t peerInfo;
void ESP_NOW_SENDER_INIT();
bool ESP_NOW_ADD_PEER();
void SendData_ESP_NOW_PWM();
Speed_vector_t get_SpeedVector_from_string(String command);
void inverse_kinematics();
void stop_wheels();
void updateLED();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSuccessfulSend = millis();
  }
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Set LED pin as output
  digitalWrite(LED_BUILTIN, HIGH); // Turn off LED
  
  Serial.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);
  delay(100);  // Give serial time to initialize

  // Set up watchdog timer
  esp_task_wdt_init(30, true); // 30 second timeout, reboot on timeout
  esp_task_wdt_add(NULL);      // Add current thread to watchdog


  ESP_NOW_SENDER_INIT();
  blinkLED(Init_Blink);
  esp_task_wdt_reset();

  Serial.println(WiFi.macAddress());

  delay(500);

}

void loop() {
  esp_task_wdt_reset();
  unsigned long currentMillis = millis();

  if (currentMillis - lastMessage_received_Time > MESSAGE_TIMEOUT) {
    stop_wheels();
  }
  
  // Read serial if available, otherwise speed stays at last value (or 0)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    speed_vector = get_SpeedVector_from_string(command);
    inverse_kinematics();
    lastMessage_received_Time = currentMillis;
  }
  
  // Always send data
  SendData_ESP_NOW_PWM();
  
  // Update LED based on transmission status
  updateLED();
}

void updateLED() {
  unsigned long now = millis();
  if (now - lastLedUpdate < LED_UPDATE_INTERVAL) return;
  lastLedUpdate = now;
  
  // LED ON if we had successful send within last 200ms
  bool shouldBeOn = (now - lastSuccessfulSend < 200);
  if (shouldBeOn != ledState) {
    ledState = shouldBeOn;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }
}

void ESP_NOW_SENDER_INIT() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    ESP.restart();
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  // Add peer once
  memcpy(peerInfo.peer_addr, ESP_LVL_0_MAC_Address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }
  
  Serial.println("ESP-NOW initialized");
  Serial.println(WiFi.macAddress());
}



void SendData_ESP_NOW_PWM() {
  static unsigned long lastSendTime = 0;
  const unsigned long SEND_INTERVAL = 50; // 20Hz
  
  if (millis() - lastSendTime < SEND_INTERVAL) return;
  lastSendTime = millis();
  
  esp_now_send(ESP_LVL_0_MAC_Address, (uint8_t *)&wheel_pwm, sizeof(PWM_data_t));
  
  if (debugmode) {
    Serial.printf("TX: FL=%d FR=%d RL=%d RR=%d\n", 
      wheel_pwm.frontLeftPWM, wheel_pwm.frontRightPWM,
      wheel_pwm.rearLeftPWM, wheel_pwm.rearRightPWM);
  }
}

Speed_vector_t get_SpeedVector_from_string(String command) {
  Speed_vector_t result = {0.0, 0.0, 0.0}; // Initialize with zeros
    
    // Find comma positions
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    
    // Check if we have valid format (two commas found)
    if (firstComma != -1 && secondComma != -1) {
        // Parse vx (first value)
        result.vx = command.substring(0, firstComma).toFloat();
        
        // Parse vy (second value)
        result.vy = command.substring(firstComma + 1, secondComma).toFloat();
        
        // Parse vz/rz (third value) - rotation around z-axis
        result.vz = command.substring(secondComma + 1).toFloat();
        if(result.vz > 0.6) result.vz = 0.6;
        else if(result.vz < -0.6) result.vz = -0.6;
        
        if (debugmode) {
            Serial.print("Parsed vector: vx=");
            Serial.print(result.vx);
            Serial.print(", vy=");
            Serial.print(result.vy);
            Serial.print(", vz=");
            Serial.println(result.vz);
        }
    } else {
        if (debugmode) {
            Serial.println("Invalid vector format received");
        }
    }
    
    return result;

}

void inverse_kinematics() {
  float max_speed = 200.0; // Your current max PWM value
    
    // Mecanum wheel kinematics: vx=forward/back, vy=strafe left/right, vz=rotation
    float fl = speed_vector.vx + speed_vector.vy + speed_vector.vz;
    float fr = speed_vector.vx - speed_vector.vy - speed_vector.vz;
    float rl = speed_vector.vx - speed_vector.vy + speed_vector.vz;
    float rr = speed_vector.vx + speed_vector.vy - speed_vector.vz;
    
    // Convert to PWM values
    wheel_pwm.frontLeftPWM = (int)(fl * max_speed);
    wheel_pwm.frontRightPWM = (int)(fr * max_speed);
    wheel_pwm.rearLeftPWM = (int)(rl * max_speed);
    wheel_pwm.rearRightPWM = (int)(rr * max_speed);
    
    // Clamp to valid range
    wheel_pwm.frontLeftPWM = constrain(wheel_pwm.frontLeftPWM, -255, 255);
    wheel_pwm.frontRightPWM = constrain(wheel_pwm.frontRightPWM, -255, 255);
    wheel_pwm.rearLeftPWM = constrain(wheel_pwm.rearLeftPWM, -255, 255);
    wheel_pwm.rearRightPWM = constrain(wheel_pwm.rearRightPWM, -255, 255);
}

void stop_wheels() {
  wheel_pwm.frontLeftPWM = 0;
  wheel_pwm.frontRightPWM = 0;
  wheel_pwm.rearLeftPWM = 0;
  wheel_pwm.rearRightPWM = 0;
}