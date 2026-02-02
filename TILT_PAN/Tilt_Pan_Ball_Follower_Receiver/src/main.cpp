#include <ESP32Servo.h>

#define PAN_PIN  9
#define TILT_PIN 8

Servo panServo;
Servo tiltServo;

float panAngle = 90;
float tiltAngle = 90;

// Fine-tuned parameters
const int DEADZONE = 30;      // Reduced for better precision
const float GAIN = 0.01;      // Adjust this for speed of following
const int ANGLE_MIN = 20;
const int ANGLE_MAX = 160;

unsigned long lastSignalTime = 0;

void ball_search() {
    static unsigned long lastMoveTime = 0;
    
    if(millis() - lastMoveTime > 100) {
        panAngle += 5; // Slow pan speed
        if (panAngle > ANGLE_MAX) {
            panAngle = ANGLE_MIN;
        }
        panServo.write((int)panAngle);
        lastMoveTime = millis();
    }
    
}

void setup() {
    Serial1.begin(115200, SERIAL_8N1, D7, D6); // Explicitly define RX, TX
    Serial.begin(115200);
    
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    
    panServo.write(90);
    tiltServo.write(180);
    delay(2000); // Allow servos to reach initial position
    tiltServo.write(90);
    delay(2000);
}

void loop() {
    if (Serial1.available())
    {
        String data = Serial1.readStringUntil('\n');
        
        int comma1 = data.indexOf(',');
        int comma2 = data.lastIndexOf(',');
        
        if (comma1 > 0 && comma2 > comma1) {
            int errorX = data.substring(0, comma1).toInt();
            int errorY = data.substring(comma1 + 1, comma2).toInt();
            int detected = data.substring(comma2 + 1).toInt();
            
            if (detected) {
                lastSignalTime = millis();
                
                // Proportional control logic
                if (abs(errorX) > DEADZONE) {
                    panAngle += errorX * GAIN; 
                }
                if (abs(errorY) > DEADZONE) {
                    tiltAngle -= errorY * GAIN; 
                }
                
                panAngle = constrain(panAngle, ANGLE_MIN, ANGLE_MAX);
                tiltAngle = constrain(tiltAngle, ANGLE_MIN, ANGLE_MAX);
                
                panServo.write((int)panAngle);
                tiltServo.write((int)tiltAngle);
            }
        
        Serial.print(">ErrorX:");
        Serial.println(errorX);

        Serial.print(">PanAngle:");
        Serial.println(panAngle);
        }
        
    }
    /* 
    else{
        //ball_search();
    } */
    
    if (millis() - lastSignalTime > 3000) {
        panAngle = 90; // Uncomment if you want auto-centering
        tiltAngle = 90;
    }
}