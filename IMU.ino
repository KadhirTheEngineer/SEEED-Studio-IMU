#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);  // IMU at I2C address 0x6A

float angleY = 0;  // Estimated angle
unsigned long prevTime;

#define SLOUCH_THRESHOLD 10.0  // Angle deviation threshold (adjust as needed)
#define SLOUCH_TIME 3000       // Milliseconds of sustained slouch before detection
unsigned long slouchStartTime = 0;
float baselineAngleY = 0;  // Calibrated upright angle

// Kalman Filter Variables
float Q_angle = 0.01;  // Process noise variance for the accelerometer
float Q_bias = 0.003;  // Process noise variance for the gyro bias
float R_measure = 0.03;  // Measurement noise variance
float angleKalman = 0, bias = 0, rate = 0;
float P[2][2] = {{0, 0}, {0, 0}};  // Error covariance matrix

void setup() {
    Serial.begin(9600);
    while (!Serial);

    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    prevTime = millis();
    delay(2000);  // Allow the IMU to stabilize
    calibrateUpright();
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
    float gyroY = myIMU.readFloatGyroY();

    // Compute angle from accelerometer
    float accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * (180.0 / M_PI);

    // Use Kalman filter to estimate the angle
    angleY = kalmanFilter(accAngleY, gyroY, dt);

    // Detect slouching
    bool slouchState = detectSlouch(angleY);

    Serial.print("AngleY: ");
    Serial.print(angleY, 2);
    Serial.print(" | Slouching: ");
    Serial.println(slouchState ? "YES" : "NO");

    delay(50);  // Update faster (~20Hz) for better responsiveness
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

// Kalman Filter for Angle Estimation
float kalmanFilter(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angleKalman += dt * rate;

    // Update estimation error covariance
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Compute Kalman gain
    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};

    // Update angle and bias
    float y = newAngle - angleKalman;
    angleKalman += K[0] * y;
    bias += K[1] * y;

    // Update error covariance matrix
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angleKalman;
}
