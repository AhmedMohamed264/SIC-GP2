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

#define CAR_STOP                0
#define CAR_MOVE_FORWARD        1
#define CAR_MOVE_BACKWARD       2
#define CAR_MOVE_RIGHT          3
#define CAR_MOVE_LEFT           4
int carState = CAR_STOP;

const int forwardTriggerPin = 15;
const int forwardEchoPin    = 13;  
Ultrasonic forwardUltra(forwardTriggerPin, forwardEchoPin);

const int forwardRightTriggerPin = 32;
const int forwardRightEchoPin    = 35;  
Ultrasonic forwardRightUltra(forwardRightTriggerPin, forwardRightEchoPin);

const int forwardLeftTriggerPin = 2;
const int forwardLeftEchoPin    = 4;  
Ultrasonic forwardLeftUltra(forwardLeftTriggerPin, forwardLeftEchoPin);

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);
int locSpeed = 100;

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

void loop() {
    updatedSlotPark(80, 20, 1, 255, true);
    delay(2000);
    // updatedSlotUnPark(55, 15, 1, 120, true);
}

void updatedSlotPark(float forwardThreshold, float wallThreshold, float margin, int adjustmentSpeed, bool turnLeft) {
    // Get average distances from all sensors
    float forwardAvg = getAverageDistance(forwardUltra, 10);
    float forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
    float forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
    float rightLeftDifference = forwardRightAvg - forwardLeftAvg;

    // Print distances for debugging
    printAllDistances(forwardAvg, forwardRightAvg, forwardLeftAvg);
    
    // delay(2000);
    // adjustCarPosition(forwardAvg, forwardRightAvg, forwardLeftAvg, rightLeftDifference, margin, adjustmentSpeed);
    carMoveForward(locSpeed);

    while (true) {
        // Get average distances from all sensors
        forwardAvg = getAverageDistance(forwardUltra, 10);

        // Print distances for debugging
        printDistance(forwardAvg);

        if (forwardAvg < forwardThreshold) {
            Serial.println("Car Reached...");

            // Turn 90 degrees based on the parameter
            if (turnLeft) {
                Serial.println("Car Turns 90 degrees to the left...");
                turnAngle(locSpeed, 90, carMoveLeft);
            } else {
                Serial.println("Car Turns 90 degrees to the right...");
                turnAngle(locSpeed, 90, carMoveRight);
            }
            delay(1000);

            // // Recheck distances after turning
            // forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
            // forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
            // rightLeftDifference = forwardRightAvg - forwardLeftAvg;
            // printAllDistances(forwardAvg, forwardRightAvg, forwardLeftAvg);

            // // Adjust car position again if needed
            // adjustCarPosition(forwardAvg, forwardRightAvg, forwardLeftAvg, rightLeftDifference, margin, adjustmentSpeed);

            // delay(2000);
            Serial.println("Car Moves toward the wall to park...");
            carMoveForward(locSpeed);

            // Check if the car is parked
            while (true) {
                forwardAvg = getAverageDistance(forwardUltra, 10);
                // forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
                // forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
                printDistance(forwardAvg);

                if (forwardAvg < wallThreshold) {
                    carStop();
                    Serial.println("Car Parked Successfully...");
                    while(1);  // Stop the program here
                }
            }
        }
    }
}

void updatedSlotUnPark(float forwardThreshold, float gateThreshold, float margin, int adjustmentSpeed, bool turnRight) {
    // Get average distances from all sensors
    float forwardAvg = getAverageDistance(forwardUltra, 10);
    float forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
    float forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
    float rightLeftDifference = forwardRightAvg - forwardLeftAvg;

    // Print distances for debugging
    printAllDistances(forwardAvg, forwardRightAvg, forwardLeftAvg);
    
    delay(2000);
    adjustCarPosition(forwardAvg, forwardRightAvg, forwardLeftAvg, rightLeftDifference, margin, adjustmentSpeed);
    carMoveBackward(locSpeed);

    while (true) {
        // Get average distances from all sensors
        forwardAvg = getAverageDistance(forwardUltra, 10);
        forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
        forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
        rightLeftDifference = forwardRightAvg - forwardLeftAvg;

        // Print distances for debugging
        printAllDistances(forwardAvg, forwardRightAvg, forwardLeftAvg);

        if (forwardAvg > forwardThreshold) {
            // Handle car adjustment if needed
            adjustCarPosition(forwardAvg, forwardRightAvg, forwardLeftAvg, rightLeftDifference, margin, adjustmentSpeed);

            // Turn 90 degrees based on the parameter
            if (turnRight) {
                Serial.println("Car Turns 90 degrees to the left...");
                turnAngle(locSpeed, 90, carMoveLeft);
            } else {
                Serial.println("Car Turns 90 degrees to the right...");
                turnAngle(locSpeed, 90, carMoveRight);
            }
            delay(1000);

            // Recheck distances after turning
            forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
            forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
            rightLeftDifference = forwardRightAvg - forwardLeftAvg;
            printAllDistances(forwardAvg, forwardRightAvg, forwardLeftAvg);

            // Adjust car position again if needed
            adjustCarPosition(forwardAvg, forwardRightAvg, forwardLeftAvg, rightLeftDifference, margin, adjustmentSpeed);

            delay(2000);
            Serial.println("Car Moves toward the Gate ...");
            carMoveForward(locSpeed);

            // Check if the car get to the Gate
            while (true) {
                forwardAvg = getAverageDistance(forwardUltra, 10);
                forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
                forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
                printAllDistances(forwardAvg, forwardRightAvg, forwardLeftAvg);

                if (forwardAvg < gateThreshold) {
                    Serial.println("Car Parked Successfully...");
                    carStop();
                    while (true);  // Stop the program here
                }
            }
        }
    }
}


void adjustCarPosition(float forwardAvg, float forwardRightAvg, float forwardLeftAvg, float rightLeftDifference, float margin, int adjustmentSpeed) {
    Serial.println("\nNow Car will make adjustments...");

    if (fabs(rightLeftDifference) > margin) {
        if (rightLeftDifference > 0) {
            Serial.println("Adjusting to the left");
            carMoveLeft(adjustmentSpeed);
        } else {
            Serial.println("Adjusting to the right");
            carMoveRight(adjustmentSpeed);
        }

        // Wait until the car is centered
        while (fabs((forwardRightAvg = getAverageDistance(forwardRightUltra, 10)) - (forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10))) > margin) {
            delay(100);
            printAllDistances(forwardAvg, forwardRightAvg, forwardLeftAvg);
            Serial.println("Still adjusting...");
        }
    }

    Serial.println("Car adjusted successfully.");
}

void printAllDistances(float forwardAvg, float rightAvg, float leftAvg) {
    Serial.print("Forward Avg: ");
    Serial.print(forwardAvg);

    Serial.print(" cm | Forward Left Avg: ");
    Serial.print(leftAvg);

    Serial.print(" cm | Forward Right Avg: ");
    Serial.println(rightAvg);
}


float getAverageDistance(Ultrasonic& sensor, int samples) {
    float total = 0;
    for (int i = 0; i < samples; i++) {
        total += sensor.read();
        delay(10);  // Small delay between readings
    }
    return total / samples;
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

void printDistance(long distance) {
    if (distance == 0) {
        Serial.println("Out of range");
    } else {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }
}

// Enhanced function to navigate forward and avoid crashes
void navigateForwardAvoidCrash(int speed, int threshold) {
    carMoveForward(speed);

    while (true) {
        long forwardDist = forwardUltra.read();
        long forwardRightDist = forwardRightUltra.read();
        long forwardLeftDist = forwardLeftUltra.read();

        // If the forward distance is less than the threshold, stop and decide action
        if (forwardDist < threshold) {
            carStop();
            delay(500);

            // Check both sides to decide the best direction
            if (forwardLeftDist > threshold && forwardRightDist > threshold) {
                // Choose the direction with the most clearance
                if (forwardLeftDist > forwardRightDist) {
                    turnAngle(speed, 90, carMoveLeft);
                } else {
                    turnAngle(speed, 90, carMoveRight);
                }
            } 
            else if (forwardLeftDist > threshold) {
                turnAngle(speed, 90, carMoveLeft);  // Turn left if left is clear
            } 
            else if (forwardRightDist > threshold) {
                turnAngle(speed, 90, carMoveRight);  // Turn right if right is clear
            } 
            else {
                // If both sides are blocked, move backward slightly
                carMoveBackward(speed);
                delay(1000);
                carStop();
                delay(500);
                
                // Re-check and try turning after reversing
                forwardLeftDist = forwardLeftUltra.read();
                forwardRightDist = forwardRightUltra.read();
                if (forwardLeftDist > forwardRightDist) {
                    turnAngle(speed, 90, carMoveLeft);
                } else {
                    turnAngle(speed, 90, carMoveRight);
                }
            }

            // Re-check forward distance after turning
            carMoveForward(speed);
            delay(500);  // Small forward movement after turning
            forwardDist = forwardUltra.read();

            // If still blocked after turning, reverse and retry the process
            if (forwardDist < threshold) {
                carStop();
                delay(500);
                carMoveBackward(speed);
                delay(1000);
                carStop();
            }
        }

        // Print current distances for debugging
        Serial.println("Distances:");
        Serial.print("Forward: "); Serial.println(forwardDist);
        Serial.print("Forward Left: "); Serial.println(forwardLeftDist);
        Serial.print("Forward Right: "); Serial.println(forwardRightDist);

        delay(100);  // Delay to prevent overloading sensor readings
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

        if (fabs(totalYaw) >= angle) {
            flag = 0;
        }
    }
    carStop();
}

float normalizeYaw(float yaw) {
    if (yaw < 0) {
        return fmod((yaw + 360), 360);
    } else {
        return fmod(yaw, 360);
    }
}

float calculateYawDifference(float startYaw, float currentYaw) {
    float diff = currentYaw - startYaw;
    
    if (diff > 180) {
        diff -= 360;
    } else if (diff < -180) {
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
    carState = CAR_MOVE_FORWARD;
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor1.forward();
    motor2.forward();
}

void carMoveBackward(int speed) {
    carState = CAR_MOVE_BACKWARD;
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor1.backward();
    motor2.backward();
}

void carMoveRight(int speed) {
    carState = CAR_MOVE_RIGHT;
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor1.forward();
    motor2.backward();
}

void carMoveLeft(int speed) {
    carState = CAR_MOVE_LEFT;
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor1.backward();
    motor2.forward();
}

void carStop() {
    carState = CAR_STOP;
    motor1.stop();
    motor2.stop();
}
