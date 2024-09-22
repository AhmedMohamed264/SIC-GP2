#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
float yaw = 0; 

void setup(void) {
    Serial.begin(115200);
    mpuInit();
}

void loop() {
  Serial.print("\tYaw: ");
  Serial.println(mpuGetYAW());
  delay(500);
}

void mpuInit() {
    mpu.begin();
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

float mpuGetYAW() {
    mpu.getEvent(&a, &g, &temp);
    float roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float pitch = atan(-a.acceleration.x / sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    float dt = 0.5;
    yaw += (g.gyro.z * 180.0 / PI) * dt;
    return yaw;
}
