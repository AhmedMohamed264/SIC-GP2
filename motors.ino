#include <L298N.h>

// Motorr 1 pins
#define AIN1 27
#define AIN2 26
#define PWMA 12

// Motorr 2 pins
#define BIN1 33
#define BIN2 25
#define PWMB 14

// Initializing motors
L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

void setup() {
    Serial.begin(9600);
}

int locSpeed = 255;
void loop() {
    Serial.println("Moving Left ..");
    carMoveForward(locSpeed);
    delay(2000);
}

void carMoveForward(int speed) {
    motor1.setSpeed(speed);
    motor1.forward();
    motor2.setSpeed(speed);
    motor2.forward();
}

void carMoveBackward(int speed) {
    motor1.setSpeed(speed);
    motor1.backward();
    motor2.setSpeed(speed);
    motor2.backward();
}

void carMoveLeft(int speed) {
    motor1.setSpeed(0);
    motor1.stop();
    motor2.setSpeed(speed);
    motor2.forward();
}

void carMoveRight(int speed) {
    motor1.setSpeed(speed);
    motor1.forward();
    motor2.setSpeed(0);
    motor2.stop();
}