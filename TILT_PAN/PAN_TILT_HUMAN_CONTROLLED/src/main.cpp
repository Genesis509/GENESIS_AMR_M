#include <ESP32Servo.h>

// Pin definitions
#define PAN_PIN  9
#define TILT_PIN 8
#define RX_PIN   D7
#define TX_PIN   D6

Servo panServo;
Servo tiltServo;

// Angle limits
const int ANGLE_MIN = 20;
const int ANGLE_MAX = 160;
const int DEFAULT_ANGLE = 90;

// Timeout for signal loss
unsigned long lastSignalTime = 0;
const unsigned long TIMEOUT_MS = 3000;
void parseAndMove(String data);
void setup() {
    Serial.begin(115200);  // Debug
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // RPI5 UART
    
    // Attach servos
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    
    // Initialize to center position
    panServo.write(DEFAULT_ANGLE);
    tiltServo.write(DEFAULT_ANGLE);
    
    Serial.println("Pan/Tilt receiver ready");
    delay(1000);
}

void loop() {
    if (Serial1.available()) {
        String data = Serial1.readStringUntil('\n');
        parseAndMove(data);
    }
    
    // Auto-center on signal loss
    if (millis() - lastSignalTime > TIMEOUT_MS) {
        panServo.write(DEFAULT_ANGLE);
        tiltServo.write(DEFAULT_ANGLE);
    }
}

void parseAndMove(String data) {
    // Expected format: "pan,tilt\n"
    int commaIdx = data.indexOf(',');
    
    if (commaIdx > 0) {
        int panAngle = data.substring(0, commaIdx).toInt();
        int tiltAngle = data.substring(commaIdx + 1).toInt();
        
        // Constrain to safe range
        panAngle = constrain(panAngle, ANGLE_MIN, ANGLE_MAX);
        tiltAngle = constrain(tiltAngle, ANGLE_MIN, ANGLE_MAX);
        
        // Move servos
        panServo.write(panAngle);
        tiltServo.write(tiltAngle);
        
        // Update signal time
        lastSignalTime = millis();
        
        // Debug output
        Serial.print("Pan: ");
        Serial.print(panAngle);
        Serial.print(" | Tilt: ");
        Serial.println(tiltAngle);
    }
}