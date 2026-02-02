#include <Arduino.h>
#include "wheel_speel_control.h"
#include <esp_task_wdt.h>
#include <esp_now.h>
#include <WiFi.h>
#include "utils.h"
#include "encoder_handler.h"
#include "interupt_interval_handler.h"



// ---------------------------ENCODER READING RELATED STUFF STARTS HERE ----------------------------------------
//--------------------------------------------------------------------------------------------------------------

// Encoder pins definition
#define FRONT_LEFT_S1  26  // Motor 1
#define FRONT_LEFT_S2  25
#define FRONT_RIGHT_S1 33  // Motor 2
#define FRONT_RIGHT_S2 32
#define REAR_LEFT_S1   14  // Motor 3
#define REAR_LEFT_S2   27
#define REAR_RIGHT_S1  13  // Motor 4
#define REAR_RIGHT_S2  12

//

// Encoder counters
volatile long frontLeftCount = 0;
volatile long frontRightCount = 0;
volatile long rearLeftCount = 0;
volatile long rearRightCount = 0;

// Previous states for encoders
volatile byte frontLeftLastState = 0;
volatile byte frontRightLastState = 0;
volatile byte rearLeftLastState = 0;
volatile byte rearRightLastState = 0;

// Timer variables for printing
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100; // Print every 100ms

EncoderHandler encoderHandler(&frontLeftCount, &frontRightCount, &rearLeftCount, &rearRightCount);

// Encoder interrupt handlers
void IRAM_ATTR updateFrontLeftEncoder() {
  byte currentState = (digitalRead(FRONT_LEFT_S1) << 1) | digitalRead(FRONT_LEFT_S2);
  
  if(frontLeftLastState == 0b00) {
    if(currentState == 0b01) frontLeftCount--;
    else if(currentState == 0b10) frontLeftCount++;
  }
  else if(frontLeftLastState == 0b01) {
    if(currentState == 0b11) frontLeftCount--;
    else if(currentState == 0b00) frontLeftCount++;
  }
  else if(frontLeftLastState == 0b11) {
    if(currentState == 0b10) frontLeftCount--;
    else if(currentState == 0b01) frontLeftCount++;
  }
  else if(frontLeftLastState == 0b10) {
    if(currentState == 0b00) frontLeftCount--;
    else if(currentState == 0b11) frontLeftCount++;
  }
  
  frontLeftLastState = currentState;
}

void IRAM_ATTR updateFrontRightEncoder() {
  byte currentState = (digitalRead(FRONT_RIGHT_S1) << 1) | digitalRead(FRONT_RIGHT_S2);
  
  if(frontRightLastState == 0b00) {
    if(currentState == 0b01) frontRightCount++;
    else if(currentState == 0b10) frontRightCount--;
  }
  else if(frontRightLastState == 0b01) {
    if(currentState == 0b11) frontRightCount++;
    else if(currentState == 0b00) frontRightCount--;
  }
  else if(frontRightLastState == 0b11) {
    if(currentState == 0b10) frontRightCount++;
    else if(currentState == 0b01) frontRightCount--;
  }
  else if(frontRightLastState == 0b10) {
    if(currentState == 0b00) frontRightCount++;
    else if(currentState == 0b11) frontRightCount--;
  }
  
  frontRightLastState = currentState;
}

void IRAM_ATTR updateRearLeftEncoder() {
  byte currentState = (digitalRead(REAR_LEFT_S1) << 1) | digitalRead(REAR_LEFT_S2);
  
  if(rearLeftLastState == 0b00) {
    if(currentState == 0b01) rearLeftCount++;
    else if(currentState == 0b10) rearLeftCount--;
  }
  else if(rearLeftLastState == 0b01) {
    if(currentState == 0b11) rearLeftCount++;
    else if(currentState == 0b00) rearLeftCount--;
  }
  else if(rearLeftLastState == 0b11) {
    if(currentState == 0b10) rearLeftCount++;
    else if(currentState == 0b01) rearLeftCount--;
  }
  else if(rearLeftLastState == 0b10) {
    if(currentState == 0b00) rearLeftCount++;
    else if(currentState == 0b11) rearLeftCount--;
  }
  
  rearLeftLastState = currentState;
}

void IRAM_ATTR updateRearRightEncoder() {
  byte currentState = (digitalRead(REAR_RIGHT_S1) << 1) | digitalRead(REAR_RIGHT_S2);
  
  if(rearRightLastState == 0b00) {
    if(currentState == 0b01) rearRightCount++;
    else if(currentState == 0b10) rearRightCount--;
  }
  else if(rearRightLastState == 0b01) {
    if(currentState == 0b11) rearRightCount++;
    else if(currentState == 0b00) rearRightCount--;
  }
  else if(rearRightLastState == 0b11) {
    if(currentState == 0b10) rearRightCount++;
    else if(currentState == 0b01) rearRightCount--;
  }
  else if(rearRightLastState == 0b10) {
    if(currentState == 0b00) rearRightCount++;
    else if(currentState == 0b11) rearRightCount--;
  }
  
  rearRightLastState = currentState;
}

// Function prototypes ------------------------------------------------------------
void configureEncoderPins();
void readInitialStates();
void attachInterrupts();
void INIT_ENCODER_READING();
void print_pwm_values();

// Object Declaration --------------------------------------------------------------

WheelPIDSpeedControl FrontLeftControl(encoderHandler.get_EncoderCount_ptr(EncoderName_t::FRONT_LEFT)); // Pass the encoder Processed count pointer
WheelPIDSpeedControl FrontRightControl(encoderHandler.get_EncoderCount_ptr(EncoderName_t::FRONT_RIGHT)); // Pass the encoder Processed count pointer
WheelPIDSpeedControl RearLeftControl(encoderHandler.get_EncoderCount_ptr(EncoderName_t::REAR_LEFT)); // Pass the encoder Processed count pointer
WheelPIDSpeedControl RearRightControl(encoderHandler.get_EncoderCount_ptr(EncoderName_t::REAR_RIGHT)); // Pass the encoder Processed count pointer
PID_TUNNER pidTuner(FrontLeftControl.get_gain_scheduler());



// ---------------------------ENCODER READING RELATED STUFF ENDS HERE ----------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------



// ---------------------------ESPNOW RELATED STUFF STARTS HERE -----------------------------------------------
//------------------------------------------------------------------------------------------------------------

 
BlinkConfig_t Init_Blink = {3, 250, 250}; // Global configuration for blinking
BlinkConfig_t Error_Blink = {3, 100, 100}; // Global configuration for error blinking
BlinkConfig_t Success_Blink = {3, 200, 200}; // Global configuration for success blinking
BlinkConfig_t waiting_Blink = {3, 600, 600}; // Global configuration for long blinking
BlinkConfig_t flash = {1, 100, 100}; // Global configuration for long blinking
BlinkConfig_t heartbeat_Blink = {1, 50, 0}; // Global configuration for heartbeat blinking


hw_timer_t * timer = NULL;
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


bool isPeerConnected = false;
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 501; // Try to reconnect every 5 ms
bool debugmode = false;  // Set to true for debugging, false for normal operation

PWM_data_t wheel_pwm; // PWM values for each wheel

esp_now_peer_info_t peerInfo;
void ESP_NOW_SENDER_INIT();
bool ESP_NOW_ADD_PEER();
void SendData_ESP_NOW_PWM();
Speed_vector_t get_SpeedVector_from_string(String command);
void inverse_kinematics();

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    isPeerConnected = true;
    digitalWrite(LED_BUILTIN, HIGH);  // Keep LED on when connection is good
  } else {
    isPeerConnected = false;
    digitalWrite(LED_BUILTIN, LOW);   // Turn off LED on failed send
  }
}


// ---------------------------ESPNOW RELATED STUFF ENDS HERE -------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

//-------------------------- Serial related stuff starts here -------------------------------------------------




void print_serial_plotter() {

    Serial.print(">FL Speed:");
    Serial.println(FrontLeftControl.get_speed());
    
    Serial.print(">FL PWM Output:");
    Serial.println(wheel_pwm.frontLeftPWM);

    Serial.print(">FL Raw Count:");
    Serial.println(frontLeftCount);
    //--------------------------
    Serial.print(">FR Speed:");
    Serial.println(FrontRightControl.get_speed());

    Serial.print(">FR PWM Output:");
    Serial.println(wheel_pwm.frontRightPWM);

    Serial.print(">FR Raw Count:");
    Serial.println(frontRightCount);
    //--------------------------
    Serial.print(">RL Speed:");
    Serial.println(RearLeftControl.get_speed());

    Serial.print(">RL PWM Output:");
    Serial.println(wheel_pwm.rearLeftPWM);

    Serial.print(">RL Raw Count:");
    Serial.println(rearLeftCount);
    //--------------------------
    Serial.print(">RR Speed:");
    Serial.println(RearRightControl.get_speed());

    Serial.print(">RR PWM Output:");
    Serial.println(wheel_pwm.rearRightPWM);

    Serial.print(">RR Raw Count:");
    Serial.println(rearRightCount);
}

void structured_motor_test() {
   
  Serial.print(millis()); Serial.print(",");
  Serial.print(wheel_pwm.frontLeftPWM); Serial.print(",");
  Serial.print(FrontLeftControl.get_speed(), 4); Serial.print(",");
  Serial.print(frontLeftCount); Serial.print(",");
  Serial.print(FrontLeftControl.get_encoder_diff()); Serial.print(",");
            
}

float target_speed = 0.0;  // Current target speed

// Add this function before setup()
void handle_serial_input() {
    pidTuner.handle_serial_input();
}

// -------------------------- Serial related stuff ends here -------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

// -------------------------- Interupt Interval Handler related stuff starts here ------------------------------
//--------------------------------------------------------------------------------------------------------------

bool timerFlag = false;

InteruptIntervalHandler interuptIntervalHandler(INTERUPT_TIMER_INTERVAL_MS);

const char* Check_Encoder_count_name = "Encoder_Count_Check";
const int Check_Encoder_count_Interal_MS = 30000;

const char* sample_Encoder_Count_name = "Sample_Encoder_Count";
const int sample_Encoder_Count_Interal_MS = 1;

const char* update_motors_speed_name = "Update_Motors_Speed";
const int update_motors_speed_Interal_MS = 5; // Will be dynamically set based on the current speed

const char* calculate_pwm_and_send_name = "Calculate_PWM_and_Send";
const int calculate_pwm_and_send_Interal_MS = 5;

const char* print_serial_plotter_name = "Print_Serial_Plotter";
const int print_serial_plotter_Interal_MS = 100;

const char* Change_PWM_name = "Change_PWM";
const int Change_PWM_Interal_MS = 500;
 
int pwm_inccrement = 0;

void INIT_INTERUPT_INTERRVALS() { //MAX 10
    interuptIntervalHandler.add_interval(Check_Encoder_count_name, Check_Encoder_count_Interal_MS);
    interuptIntervalHandler.add_interval(sample_Encoder_Count_name, sample_Encoder_Count_Interal_MS);
    interuptIntervalHandler.add_interval(update_motors_speed_name, update_motors_speed_Interal_MS);
    interuptIntervalHandler.add_interval(calculate_pwm_and_send_name, calculate_pwm_and_send_Interal_MS);
    interuptIntervalHandler.add_interval(print_serial_plotter_name, print_serial_plotter_Interal_MS);
    interuptIntervalHandler.add_interval(Change_PWM_name, Change_PWM_Interal_MS);
}


  int pwm_relay = 0;


void handleTimerEvent() {

    interuptIntervalHandler.update_intervals();
    //I'll use if else in another better version
    if(interuptIntervalHandler.interval_reached(Check_Encoder_count_name)) {
         encoderHandler.check_raw_encoders_count_and_reset(); // Check and reset encoder counts if necessary
    }

    if(interuptIntervalHandler.interval_reached(sample_Encoder_Count_name)) {
        encoderHandler.update_EncoderCount_Samples_and_raw_average(); // Update encoder count samples and calculate average
    }

    if(interuptIntervalHandler.interval_reached(update_motors_speed_name)) {
        FrontLeftControl.update_current_speed(); // Update current speed with interval check
    }

     if(interuptIntervalHandler.interval_reached(calculate_pwm_and_send_name)) {
      /* //wheel_pwm.frontLeftPWM = FrontLeftControl.PID_CONTROLLER(target_speed); // Get PWM output from PID controller
      wheel_pwm.frontRightPWM = 0;
      wheel_pwm.rearLeftPWM = 0;
      wheel_pwm.rearRightPWM = 0; */

      SendData_ESP_NOW_PWM(); // Send PWM data via ESP-NOW
    } 

    if(interuptIntervalHandler.interval_reached(print_serial_plotter_name)) {
        print_serial_plotter(); // Print encoder values for Teleplot
       /*  pwm_inccrement++;
        if(pwm_inccrement == 255 ) pwm_inccrement = 0;
        //structured_motor_test(); */
    }

    if(interuptIntervalHandler.interval_reached(Change_PWM_name)) {
      
        
      
    } 
     

    timerFlag = false;  // Clear the flag after processing
}

void IRAM_ATTR onTimer() {
    timerFlag = true;  // Only set flag  no complex operations in ISR just learned this
    //handleTimerEvent();
}

// Timer setup function
void setupTimerInterrupt(uint32_t intervalMs) {
    // Create timer: (timer_id, prescaler, count_up)
    // Prescaler 80: 80MHz CPU / 80 = 1MHz = 1µs per tick
    timer = timerBegin(0, 80, true);
    
    // Attach interrupt function to timer
    timerAttachInterrupt(timer, &onTimer, true);
    
    // Set alarm value (microseconds) and auto-reload
    timerAlarmWrite(timer, intervalMs * 1000, true);  // ms to µs conversion
    
    // Enable the timer alarm
    timerAlarmEnable(timer);
}

void setTimerInterval(uint32_t intervalMs) {
    if (timer != NULL) {
        timerAlarmDisable(timer);                        // Stop current alarm
        timerAlarmWrite(timer, intervalMs * 1000, true); // Set new interval
        timerAlarmEnable(timer);                         // Restart alarm
    }
}



// -------------------------- Interupt Interval Handler related stuff ends here ---------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // Turn on LED during initialization
  // Initialize Serial
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ; // Wait for serial port to connect or timeout after 3 seconds
  }
  // --------------Encoder Reading Initialization----------------------------------

  INIT_ENCODER_READING(); // Initialize encoder reading

  // ---------------ESPNOW Initialization Starts Here -----------------------------
  // Set up watchdog timer
  esp_task_wdt_init(30, true); // 30 second timeout, reboot on timeout
  esp_task_wdt_add(NULL);      // Add current thread to watchdog

  ESP_NOW_SENDER_INIT();
  esp_task_wdt_reset();
  Serial.println(WiFi.macAddress());

  setupTimerInterrupt(INTERUPT_TIMER_INTERVAL_MS);
  INIT_INTERUPT_INTERRVALS(); // Initialize interrupt intervals
  
  blinkLED(Init_Blink);
  delay(1000);
  Serial.println("timestamp_ms,pwm_applied,motor_speed_rad_s,encoder_count,encoder_diff,");

}
  int pwm_steps[] = {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255};

void loop() {
  static unsigned long prevMillis = 0; // Previous time for debounce
  unsigned long currentMillis = millis(); // Current time

  esp_task_wdt_reset();
    // Try to reconnect peer if not connected
  if (!isPeerConnected && (millis() - lastReconnectAttempt > RECONNECT_INTERVAL)) {
    lastReconnectAttempt = millis();
    if (!ESP_NOW_ADD_PEER()) {
      blinkLED(heartbeat_Blink);
    }
  }

  if (Serial.available()) 
  {
    String command = Serial.readStringUntil('\n');
    
    command.trim(); // Remove any leading/trailing whitespace
    speed_vector = get_SpeedVector_from_string(command);
    inverse_kinematics(); // Calculate PWM values based on speed vector
    
  }

  if(timerFlag) {
    handleTimerEvent();
  }

  /* handle_serial_input();
  pidTuner.update();/When Tunning PID */ 

  SendData_ESP_NOW_PWM();
  

}

// ---------------------------Encoder Related Functions Starts Here ----------------------------------------------
//---------------------------------------------------------------------------------------------------------------

void print_pwm_values() {
    Serial.print(">FL:");
    Serial.println(frontLeftCount);
    Serial.print(">FR:");
    Serial.println(frontRightCount);
    Serial.print(">RL:");
    Serial.println(rearLeftCount);
    Serial.print(">RR:");
    Serial.println(rearRightCount); 
}

void configureEncoderPins() {
  pinMode(FRONT_LEFT_S1, INPUT_PULLUP);
  pinMode(FRONT_LEFT_S2, INPUT_PULLUP);
  pinMode(FRONT_RIGHT_S1, INPUT_PULLUP);
  pinMode(FRONT_RIGHT_S2, INPUT_PULLUP);
  pinMode(REAR_LEFT_S1, INPUT_PULLUP);
  pinMode(REAR_LEFT_S2, INPUT_PULLUP);
  pinMode(REAR_RIGHT_S1, INPUT_PULLUP);
  pinMode(REAR_RIGHT_S2, INPUT_PULLUP);
}

void readInitialStates() {
  frontLeftLastState = (digitalRead(FRONT_LEFT_S1) << 1) | digitalRead(FRONT_LEFT_S2);
  frontRightLastState = (digitalRead(FRONT_RIGHT_S1) << 1) | digitalRead(FRONT_RIGHT_S2);
  rearLeftLastState = (digitalRead(REAR_LEFT_S1) << 1) | digitalRead(REAR_LEFT_S2);
  rearRightLastState = (digitalRead(REAR_RIGHT_S1) << 1) | digitalRead(REAR_RIGHT_S2);
}

void attachEncoderInterrupts() {
  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_S1), updateFrontLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_S2), updateFrontLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_S1), updateFrontRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_S2), updateFrontRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REAR_LEFT_S1), updateRearLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REAR_LEFT_S2), updateRearLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_S1), updateRearRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_S2), updateRearRightEncoder, CHANGE);
}

void INIT_ENCODER_READING() {
  configureEncoderPins();
  readInitialStates();
  attachEncoderInterrupts();
}
// ---------------------------Encoder Related Functions Ends Here ----------------------------------------------
// -------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------


// ---------------------------ESPNOW RELATED FUNCTIONS STARTS HERE ---------------------------------------------
//--------------------------------------------------------------------------------------------------------------
void ESP_NOW_SENDER_INIT() {
  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Ensure clean state

  
  esp_err_t result = esp_now_init();
  if (result != ESP_OK) {
    Serial.print("ESP-NOW init failed: ");
    Serial.println(result);
    //blinkLED(Error_Blink);
    delay(1000);
    ESP.restart();
    return;
  }
  
  Serial.println("ESP-NOW initialized successfully");
  
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Add peer
  if (ESP_NOW_ADD_PEER()) {
    Serial.println("Peer added successfully");
    isPeerConnected = true;
  } else {
    Serial.println("Failed to add peer");
    isPeerConnected = false;
  }
}

bool ESP_NOW_ADD_PEER() {
  // Create peer info
  memcpy(peerInfo.peer_addr, ESP_LVL_0_MAC_Address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Remove peer if it exists (to handle reconnection)
  esp_now_del_peer(ESP_LVL_0_MAC_Address);
  
  // Add peer
  esp_err_t result = esp_now_add_peer(&peerInfo);
  return (result == ESP_OK);
}

void SendData_ESP_NOW_PWM() {
  static unsigned long lastSendTime = 0;
  const unsigned long SEND_INTERVAL = 50; // Send every 50ms (20Hz)
  
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime < SEND_INTERVAL) {
    return; // Don't send too frequently
  }
  
  esp_err_t result = esp_now_send(ESP_LVL_0_MAC_Address, (uint8_t *)&wheel_pwm, sizeof(PWM_data_t));
  
  if (result == ESP_OK) {
    lastSendTime = currentTime;
    if (debugmode) {
      Serial.println("Data sent successfully");
    }
  } else {
    isPeerConnected = false;
    if (debugmode) {
      Serial.print("Send failed: ");
      Serial.println(result);
    }
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
    
    static const float max_wheel_speed_rads = 22.0;

    // Mecanum wheel kinematics: vx=forward/back, vy=strafe left/right, vz=rotation
    float front_left_speed_rads = speed_vector.vx + speed_vector.vy + speed_vector.vz;
    float front_right_speed_rads = speed_vector.vx - speed_vector.vy - speed_vector.vz;
    float rear_left_speed_rads = speed_vector.vx - speed_vector.vy + speed_vector.vz;
    float rear_right_speed_rads = speed_vector.vx + speed_vector.vy - speed_vector.vz;

    if(abs(front_left_speed_rads) > max_wheel_speed_rads )  front_left_speed_rads = max_wheel_speed_rads * front_left_speed_rads / abs(front_left_speed_rads);
    if(abs(front_right_speed_rads) > max_wheel_speed_rads )  front_right_speed_rads = max_wheel_speed_rads * front_right_speed_rads / abs(front_right_speed_rads);
    if(abs(rear_left_speed_rads) > max_wheel_speed_rads )  rear_left_speed_rads = max_wheel_speed_rads * rear_left_speed_rads / abs(rear_left_speed_rads);
    if(abs(rear_right_speed_rads) > max_wheel_speed_rads )  rear_right_speed_rads = max_wheel_speed_rads * rear_right_speed_rads / abs(rear_right_speed_rads);

    wheel_pwm.frontLeftPWM = FrontLeftControl.PID_CONTROLLER(front_left_speed_rads);
    wheel_pwm.frontRightPWM = FrontLeftControl.PID_CONTROLLER(front_right_speed_rads);
    wheel_pwm.rearLeftPWM = FrontLeftControl.PID_CONTROLLER(rear_left_speed_rads);
    wheel_pwm.rearRightPWM = FrontLeftControl.PID_CONTROLLER(rear_right_speed_rads);
    
    /* // Convert to PWM values
    wheel_pwm.frontLeftPWM = (int)(fl * max_pwm);
    wheel_pwm.frontRightPWM = (int)(fr * max_pwm);
    wheel_pwm.rearLeftPWM = (int)(rl * max_pwm);
    wheel_pwm.rearRightPWM = (int)(rr * max_pwm);
    
    // Clamp to valid range
    wheel_pwm.frontLeftPWM = constrain(wheel_pwm.frontLeftPWM, -255, 255);
    wheel_pwm.frontRightPWM = constrain(wheel_pwm.frontRightPWM, -255, 255);
    wheel_pwm.rearLeftPWM = constrain(wheel_pwm.rearLeftPWM, -255, 255);
    wheel_pwm.rearRightPWM = constrain(wheel_pwm.rearRightPWM, -255, 255);*/
} 
// ---------------------------ESPNOW RELATED FUNCTIONS ENDS HERE ---------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------