#include <Ultrasonic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <L298N.h>
#include <Wire.h>
#include <math.h>

#define AIN1 27
#define AIN2 26
#define PWMA 12

#define BIN1 33
#define BIN2 25
#define PWMB 14

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

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);
int locSpeed = 150;

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
float yaw = 0;
float totalYaw = 0;
unsigned long lastTime = 0;
const float alpha = 0.98;

typedef void (*TurnDirectionFunc)(int speed);
long distance = 0;

void setup(void) {
    Serial.begin(115200);
    mpuInit();
    lastTime = millis();
}

void slot_0() {
    delay(2000);
    carMoveForward(locSpeed);

    float read = 0, avg = 0;

    // Moving forward until the average distance is less than 75
    while (1) {
        read = 0;  // Reset read accumulator
        for (int i = 0; i < 10; i++) {
            long distance = forwardUltra.read();
            printDistance(distance);
            read += distance;
            delay(10);  // Small delay to avoid reading too quickly
        }
        avg = read / 10;  // Calculate average distance
        if (avg < 75) {
            break;  // Exit loop if average distance is less than 75
        }
    }

    // Turning 90 degrees to the right
    turnAngle(locSpeed, 90, carMoveRight); 
    delay(1000);
    
    // Moving forward while the distance is greater than 20 cm
    carMoveForward(locSpeed);
    while (forwardUltra.read() > 20) {
        printDistance(forwardUltra.read());
        delay(100);  // Avoid spamming sensor reads
    }    

    carStop();
    delay(3000);

    // Moving backward while the distance is less than 45 cm
    carMoveBackward(locSpeed);
    while (forwardUltra.read() < 45) {
        printDistance(forwardUltra.read());
        delay(100);
    }    
    
    // Turning 90 degrees to the right again
    turnAngle(locSpeed, 90, carMoveRight); 
    delay(1000);
    
    // Moving forward while the distance is greater than 30 cm
    carMoveForward(locSpeed);
    while (forwardUltra.read() > 30) {
        printDistance(forwardUltra.read());
        delay(100);
    } 
    carStop();

    while (1);
}

void loop() {
    slot_0();
}

void printDistance(long distance) {
    if (distance == 0) {
        Serial.println("Out of range");
    } else {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }
}

void turnAngle(int speed, int angle, TurnDirectionFunc turnDirection) {
    carStop();
    totalYaw = 0;
    float startYaw = normalizeYaw(mpuGetYAW());
    float currentYaw = 0;
    int flag = 1;
    turnDirection(speed);
    while (flag) {
        currentYaw = normalizeYaw(mpuGetYAW());
        totalYaw += calculateYawDifference(startYaw, currentYaw);
        startYaw = currentYaw;

        // Serial.print("Total Yaw Change: ");
        // Serial.println(totalYaw);

        if (fabs(totalYaw) >= angle) {
            flag = 0;
        }
    }
    carStop();
}

float normalizeYaw(float yaw) {
    if (yaw < 0) {
        return fmod((yaw + 360), 360);
    } 
    else {
        return fmod(yaw, 360);
    }
}

float calculateYawDifference(float startYaw, float currentYaw) {
    float diff = currentYaw - startYaw;
    
    if (diff > 180) {
        diff -= 360;
    } 
    else if (diff < -180) {
        diff += 360;
    }

    return diff;
}

void mpuInit() {
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

float mpuGetYAW() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    mpu.getEvent(&a, &g, &temp);

    yaw += (g.gyro.z * 180.0 / PI) * dt;
    return normalizeYaw(yaw);
}

void carMoveForward(int speed) {
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor1.forward();
    motor2.forward();
}

void carMoveBackward(int speed) {
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor1.backward();
    motor2.backward();
}

void carMoveRight(int speed) {
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor1.forward();
    motor2.backward();
}

void carMoveLeft(int speed) {
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor1.backward();
    motor2.forward();
}

void carStop() {
    motor1.stop();
    motor2.stop();
}