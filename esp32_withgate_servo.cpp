#include <ESP32Servo.h>  // Use ESP32Servo library

#define IR_SENSOR_PIN 5   
#define SERVO_PIN 18      

Servo gateServo;  

void setup() {
  pinMode(IR_SENSOR_PIN, INPUT);  
  gateServo.attach(SERVO_PIN);  
  gateServo.write(0);  
  Serial.begin(115200);  
}

void loop() {
  int sensorValue = digitalRead(IR_SENSOR_PIN);  
  
  if (sensorValue == LOW) { // Added braces
    Serial.println("Vehicle detected! Opening gate...");
    gateServo.write(90);  
    delay(5000);  // Keep gate open for 5 seconds 
    Serial.println("Closing gate...");
    gateServo.write(0); 
  } else {
    Serial.println("No vehicle.");
  }
  
  delay(500);  // Small delay for readability
}

