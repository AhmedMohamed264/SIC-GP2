#include <Ultrasonic.h>

const int forwardTriggerPin = 15;
const int forwardEchoPin    = 13;  
Ultrasonic forwardUltra(forwardTriggerPin, forwardEchoPin);

const int forwardRightTriggerPin = 32;
const int forwardRightEchoPin    = 35;  
Ultrasonic forwardRightUltra(forwardRightTriggerPin, forwardRightEchoPin);

const int forwardLeftTriggerPin = 2;
const int forwardLeftEchoPin    = 4;  
Ultrasonic forwardLeftUltra(forwardLeftTriggerPin, forwardLeftEchoPin);

const int backwardRightTriggerPin = 18;
const int backwardRightEchoPin    = 5;  
Ultrasonic backwardRightUltra(backwardRightTriggerPin, backwardRightEchoPin);

const int backwardLeftTriggerPin = 16;
const int backwardLeftEchoPin    = 17;  
Ultrasonic backwardLeftUltra(backwardLeftTriggerPin, backwardLeftEchoPin);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Ultrasonic Sensor Initialized");
}

void loop() {
  long forwardDist = forwardUltra.read();
  long forwardRightDist = forwardRightUltra.read();
  long forwardLeftDist = forwardLeftUltra.read();
  long backwardRightDist = backwardRightUltra.read();
  long backwardLeftDist = backwardLeftUltra.read();
  
  // Print the distances
  printAllDistances(forwardDist, forwardRightDist, forwardLeftDist, backwardRightDist, backwardLeftDist);

  delay(200);  // Delay for readability
}

// Function to print the distances of all 5 sensors
void printAllDistances(long forward, long forwardRight, long forwardLeft, long backwardRight, long backwardLeft) {
  Serial.println("Sensor Readings:");

  printIndividualDistance("Forward", forward);
  printIndividualDistance("Forward Right", forwardRight);
  printIndividualDistance("Forward Left", forwardLeft);
  printIndividualDistance("Backward Right", backwardRight);
  printIndividualDistance("Backward Left", backwardLeft);

  Serial.println("-------------------------");
}

// Function to print individual distance with sensor label
void printIndividualDistance(const char* sensorLabel, long distance) {
  if (distance == 0) {
    Serial.print(sensorLabel);
    Serial.println(": Out of range");
  } else {
    Serial.print(sensorLabel);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}
