#include <ESP32Servo.h>  // Use ESP32Servo library

#define SERVO_PIN 18
Servo gateServo;  

void setup() {

  gateServo.attach(SERVO_PIN);  
  gateServo.write(0);  
  Serial.begin(115200);  
}

void loop() {
  
  
 
  for (int angle = 0; angle <= 90; angle += 10) {
        gateServo.write(angle);  // Open gate
        delay(100);  // Wait for the servo to move
  }
    delay(5000);  // Keep gate open for 5 seconds 

  for (int angle = 90; angle >= 0; angle -= 10) {
            gateServo.write(angle);  // Close gate
            delay(100);  // Wait for the servo to move
          }

  
  delay(500);  // Small delay for readability
}