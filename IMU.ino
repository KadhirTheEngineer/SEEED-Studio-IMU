#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);  // IMU at I2C address 0x6A

float angleX = 0, angleY = 0;  // Estimated angles
unsigned long prevTime;

#define SLOUCH_THRESHOLD 10.0  // Angle deviation threshold (adjust as needed)
#define SLOUCH_TIME 3000       // Milliseconds of sustained slouch before detection
unsigned long slouchStartTime = 0;
bool isSlouching = false;
float baselineAngleY = 0;  // Calibrated upright angle

void setup() {
    Serial.begin(9600);
    while (!Serial);

    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    prevTime = millis();
    delay(1000);  // Give time for IMU to stabilize
    calibrateUpright();  // Set initial baseline for upright posture
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

    // Detect slouching
    bool slouchState = detectSlouch(angleY);

    Serial.print("AngleY: ");
    Serial.print(angleY, 2);
    Serial.print(" | Slouching: ");
    Serial.println(slouchState ? "YES" : "NO");

    delay(100); // Update at ~50Hz
}

// Function to set baseline angle when upright
void calibrateUpright() {
    baselineAngleY = angleY;
    Serial.println("Upright position calibrated!");
}

// Function to detect slouching based on deviation
bool detectSlouch(float angle) {
    if (abs(angle - baselineAngleY) > SLOUCH_THRESHOLD) {
        if (slouchStartTime == 0) slouchStartTime = millis();  // Start timing slouch
        if (millis() - slouchStartTime > SLOUCH_TIME) {
            return true;
        }
    } else {
        slouchStartTime = 0;  // Reset if posture improves
    }
    return false;
}
