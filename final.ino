#include <Ultrasonic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <L298N.h>
#include <Wire.h>
#include <math.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "Sharq"; 
const char* password = "1234567890";

// MQTT broker credentials
const char* mqttServer = "c602d4e93f274a9e93bf8ba3395ef9ed.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "omar2003";
const char* mqttPassword = "12345678@Nu";
const char* mqttTopic = "parking";


/** root certificate ***/
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

WiFiClientSecure espClient;
PubSubClient client(espClient);

#define AIN1 27
#define AIN2 26
#define PWMA 12

#define BIN2 33
#define BIN1 25
#define PWMB 14

#define CAR_STOP                0
#define CAR_MOVE_FORWARD        1
#define CAR_MOVE_BACKWARD       2
#define CAR_MOVE_RIGHT          3
#define CAR_MOVE_LEFT           4
#define CAR_TURN_RIGHT          0
#define CAR_TURN_LEFT           1

#define UNPARKED                0xff
#define SLOT_ZERO               0
#define SLOT_ONE                1
#define SLOT_TWO                2
#define SLOT_THREE              3

int parkedSlot = UNPARKED;
int carState = CAR_STOP;

int locSpeed = 150;
int minSpeed = 100;

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

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

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
float yaw = 0;
float totalYaw = 0;
unsigned long lastTime = 0;
const float alpha = 0.98;

typedef void (*TurnDirectionFunc)(int speed);
long distance = 0;
unsigned long maxAdjustmentTimeMs = 2000;

void setup(void) {
    Serial.begin(115200);
    mpuInit();
    lastTime = millis();

    // Connecting to WiFi
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    // Wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Set up MQTT Server and Port
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);
    espClient.setCACert(root_ca);

    // Set the Root CA Certificate
    //BearSSL::CertStore certStore;

    // Connect to MQTT Broker
    connectToMQTT();
}

void unPark(int parkingSlot) {
    switch(parkingSlot) {
        case SLOT_ZERO :
            Serial.println("\nCar un Parking now from slot 0 ...");
            updatedSlotUnPark(30, 40, 1, 150, 255, CAR_TURN_RIGHT);
            carStop(); 
            break;

        case SLOT_ONE :
            Serial.println("\nCar un Parking now from slot 1 ...");
            updatedSlotUnPark(30, 19, 1, 120, 120, CAR_TURN_LEFT);
            carStop(); 
            break;

        case SLOT_TWO :
            Serial.println("\nCar un Parking now from slot 2 ...");
            updatedSlotUnPark(30, 40, 1, 150, 255, CAR_TURN_RIGHT);
            carStop(); 
            break;

        case SLOT_THREE :
            Serial.println("\nCar un Parking now from slot 3 ...");
            updatedSlotUnPark(30, 40, 1, 150, 255, CAR_TURN_LEFT);
            carStop(); 
            break;
    }

    parkingSlot = UNPARKED;
}

void parkingSlot_0() {
    updatedSlotPark(15, 10, 1, 150, 255, CAR_TURN_RIGHT);
    parkedSlot = SLOT_ZERO;
    Serial.println("Car Parked Successfully at slot 0 ...");
}

void parkingSlot_1() {
    updatedSlotPark(25, 16, 1, 130, 100, CAR_TURN_LEFT);
    parkedSlot = SLOT_ONE;
    Serial.println("Car Parked Successfully at slot 1 ...");
}

void parkingSlot_2() {
    updatedSlotPark(40, 15, 1, 150, 255, CAR_TURN_RIGHT);
    parkedSlot = SLOT_TWO;
    Serial.println("Car Parked Successfully at slot 2 ...");
}

void parkingSlot_3() {
    updatedSlotPark(50, 40, 1, 150, 255, CAR_TURN_LEFT);
    parkedSlot = SLOT_THREE;
    Serial.println("Car Parked Successfully at slot 3 ...");
}

void loop() {
    if (!client.connected()) {
        connectToMQTT();
    }
    client.loop();
}

void slotOneParkUnPark() {
    parkingSlot_1();
    delay(1000);
    updatedSlotUnPark(27, 19, 1, 120, 120, CAR_TURN_LEFT);
    delay(1000);
    while(1);
}

void slotThreeParkUnPark() {
    parkingSlot_3();
    delay(1000);
    updatedSlotUnPark(25, 30, 1, 150, 150, CAR_TURN_LEFT);
    delay(1000);
    while(1);
}

void connectToMQTT() {
  Serial.print("Connecting to MQTT Broker: ");
  Serial.println(mqttServer);

  // Attempt to connect
  while (!client.connected()) {
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT Broker successfully!");
      
      // Subscribe to the topic
      if (client.subscribe(mqttTopic)) {
        Serial.print("Subscribed to topic: ");
        Serial.println(mqttTopic);
      } else {
        Serial.println("Failed to subscribe to topic.");
      }
    } else {
      Serial.print("Failed to connect. State: ");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("\nMessage arrived in topic : ");
    Serial.println(topic);
    Serial.print("Message : ");
    
    // Convert the payload to a string to handle multi-character messages like "exit"
    String message = "";
    for (int i = 0; i < length; i++) 
    {
        message += (char)payload[i];
        Serial.print((char)payload[i]); // Print the message as it's received
    }

    // Switch case or conditional checks to handle the message
    if (message == "0") {
        Serial.println("\nAction for message '0'");
        parkingSlot_0();
    } 
    else if (message == "1") {
        Serial.println("\nAction for message '1'");
        parkingSlot_1();
    } 
    else if (message == "2") {
        Serial.println("\nAction for message '2'");
        parkingSlot_2();
    } 
    else if (message == "3") {
        Serial.println("\nAction for message '3'");
        parkingSlot_3();
    } 
    else if (message == "4") {
        unPark(parkedSlot);
    } 
    else if (message == "5") {
        carMoveForward(120);
        Serial.println("\nCar Leaves the Parking ...");
        delay(1000);
        carStop();
    } 
    else {
        Serial.println("Unknown message");
    }
}

void updatedSlotPark(float forwardThreshold, float wallThreshold, float margin, int workingSpeed, int adjustmentSpeed, int turningDirection) {
    float forwardAvg = getAverageDistance(forwardUltra, 10);
    float forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
    float forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
    float rightLeftDifference = forwardRightAvg - forwardLeftAvg;
    printAllDistances(forwardAvg, forwardRightAvg, forwardLeftAvg);
    
    delay(200);
    carMoveForward(workingSpeed);

    while (true) {
        forwardAvg = getAverageDistance(forwardUltra, 10);
        printDistance(forwardAvg);

        if (forwardAvg < forwardThreshold) {
            Serial.println("Car Reached...");

            if (CAR_TURN_LEFT == turningDirection) {
                Serial.println("Car Turns 90 degrees to the left...");
                turnAngle(workingSpeed, 90, carMoveLeft);
            }
            else if(CAR_TURN_RIGHT == turningDirection) {
                Serial.println("Car Turns 90 degrees to the right...");
                turnAngle(workingSpeed, 60, carMoveRight);
            }


            Serial.println("Car Moves toward the wall to park...");
            carMoveForward( workingSpeed );

            // Check if the car is parked
            while (true) {
                forwardAvg = getAverageDistance(forwardUltra, 10);
                printDistance(forwardAvg);

                if (forwardAvg < wallThreshold) {
                    carStop();
                    Serial.println("Car Parked Successfully...");
                    adjustCarPosition(1, 120);
                    return;
                }
            }
        }
    }
}

void updatedSlotUnPark(float backwardThreshold, float wallThreshold, float margin, int workingSpeed, int adjustmentSpeed, int turningDirection) {

    adjustCarPosition(margin, adjustmentSpeed);

    float backwardAvg = getAverageDistance(forwardUltra, 10);
    float backwardRightAvg = getAverageDistance(forwardRightUltra, 10);
    float backwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
    float rightLeftDifference = backwardRightAvg - backwardLeftAvg;
    float backAVG=0;
    printAllDistances(backwardAvg, backwardRightAvg, backwardLeftAvg);
    
    delay(200);
    carMoveBackward(workingSpeed);

    while (true) {
        backwardAvg = getAverageDistance(forwardUltra, 10);
        if (backwardAvg > backwardThreshold) {
            Serial.println("Car Reached...");

            // Turn 90 degrees based on the parameter
            if (CAR_TURN_LEFT == turningDirection) {
                Serial.println("Car Turns 90 degrees to the left...");
                turnAngle(workingSpeed, 90, carMoveLeft);
            }
            else if(CAR_TURN_RIGHT == turningDirection) {
                Serial.println("Car Turns 90 degrees to the right...");
                turnAngle(workingSpeed, 90, carMoveRight);
            }

            delay(500);
            Serial.println("Car Moves toward the Gate to Exit ...");
            carMoveForward(workingSpeed-20);

            // Check if the car is parked
            while (true) {
                float backwardRightAVG = getAverageDistance(backwardRightUltra, 10);
                float backwardLeftAVG = getAverageDistance(backwardLeftUltra, 10);
                float backAVG = (backwardLeftAVG+backwardRightAVG)/2;
                printDistance(backAVG);

                if (backAVG > wallThreshold) {
                    carStop();
                    Serial.println("Car Exit Successfully...");
                    return;
                }
            }
        }
    }
}

void adjustCarPosition(float margin, int maxSpeed) {
    Serial.println("\nNow Car will make adjustments...");

    float forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
    float forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
    float rightLeftDifference = forwardRightAvg - forwardLeftAvg;
    printAllDistances(rightLeftDifference, forwardRightAvg, forwardLeftAvg);

    // Sensor data validation
    if (forwardRightAvg < 0 || forwardLeftAvg < 0 || forwardRightAvg > 400 || forwardLeftAvg > 400) {
        Serial.println("Error: Sensor data out of range, stopping adjustment.");
        carStop();
        return;
    }

    // Proportional adjustment based on distance difference
    if (fabs(rightLeftDifference) > margin) {
        unsigned long startTime = millis();
        
        // Continue adjustment until car is centered or time limit is exceeded
        while (fabs(rightLeftDifference) > margin) {
            forwardRightAvg = getAverageDistance(forwardRightUltra, 10);
            forwardLeftAvg = getAverageDistance(forwardLeftUltra, 10);
            rightLeftDifference = forwardRightAvg - forwardLeftAvg;
            printAllDistances(rightLeftDifference, forwardRightAvg, forwardLeftAvg);

            // Adjust speed based on the magnitude of the difference
            // Ensuring speed is above minSpeed
            int adjustmentSpeed = map(fabs(rightLeftDifference), 0, 50, minSpeed, maxSpeed); 
            adjustmentSpeed = max(adjustmentSpeed, minSpeed);

            if (rightLeftDifference > 0) {
                Serial.println("Adjusting to the left");
                carMoveLeft(adjustmentSpeed);
            } else {
                Serial.println("Adjusting to the right");
                carMoveRight(adjustmentSpeed);
            }

            // Check for timeout
            if (millis() - startTime > maxAdjustmentTimeMs) {
                Serial.println("Adjustment timed out.");
                break;
            }

            // Adding a delay to prevent rapid switching
            delay(100);
        }
    }

    Serial.println("Car adjusted successfully.");
    carStop();
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
        delay(10);
    }
    return total / samples;
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
    motor2.backward();
    motor1.forward();
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
