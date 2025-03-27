#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);  // IMU at I2C address 0x6A

float angleX = 0, angleY = 0;  // Estimated angles
unsigned long prevTime;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    prevTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;

    // Read accelerometer values (g)
    float accX = myIMU.readFloatAccelX();
    float accY = myIMU.readFloatAccelY();
    float accZ = myIMU.readFloatAccelZ();

    // Read gyroscope values (deg/sec)
    float gyroX = myIMU.readFloatGyroX();
    float gyroY = myIMU.readFloatGyroY();

    // Compute angle from accelerometer
    float accAngleX = atan2(accY, accZ) * (180.0 / M_PI); // Convert to degrees
    float accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * (180.0 / M_PI);

    // Integrate gyroscope to estimate angles
    angleX += gyroX * dt;
    angleY += gyroY * dt;

    // Apply complementary filter
    angleX = 0.98 * (angleX + gyroX * dt) + 0.02 * accAngleX;
    angleY = 0.98 * (angleY + gyroY * dt) + 0.02 * accAngleY;

    Serial.println(angleY, 2);

    delay(100); // Update at ~50Hz
}
