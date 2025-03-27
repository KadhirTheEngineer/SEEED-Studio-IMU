#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>
#include <ArduinoBLE.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);  // IMU at I2C address 0x6A

float angleY = 0;  // Estimated angle
unsigned long prevTime;

#define BUTTON_PIN 4
#define SLOUCH_THRESHOLD 15.0  
#define SLOUCH_TIME 3000       
#define RECALIBRATION_TIME 5000  

unsigned long slouchStartTime = 0, recalibrationStartTime = 0;
float baselineAngleY = 90;  
bool slouching = false;
bool buttonPressed = false;

// Kalman Filter Variables
float Q_angle = 0.01, Q_bias = 0.003, R_measure = 0.03;
float angleKalman = 90, bias = 0, rate = 0; 
float P[2][2] = {{0, 0}, {0, 0}};

void setup() {
    Serial.begin(9600);
    while (!Serial);

    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    pinMode(BUTTON_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    prevTime = millis();
    delay(2000);
    calibrateUpright();
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; 
    prevTime = currentTime;

    float accX = myIMU.readFloatAccelX();
    float accY = myIMU.readFloatAccelY();
    float accZ = myIMU.readFloatAccelZ();
    float gyroY = myIMU.readFloatGyroY();

    float accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * (180.0 / M_PI) + 90.0;

    angleY = kalmanFilter(accAngleY, gyroY, dt);

    if (digitalRead(BUTTON_PIN) == HIGH && !buttonPressed) {
        calibrateUpright();
        buttonPressed = true; 
        digitalWrite(LED_BUILTIN, LOW);
    } 
    else if (digitalRead(BUTTON_PIN) == LOW) {
        buttonPressed = false; 
        digitalWrite(LED_BUILTIN, HIGH);        
    }

    bool newSlouchState = detectSlouch(angleY);

    if (!newSlouchState && slouching) {
        if (recalibrationStartTime == 0) {
            recalibrationStartTime = millis();
        } else if (millis() - recalibrationStartTime > RECALIBRATION_TIME) {
            calibrateUpright();
            recalibrationStartTime = 0;
        }
    } else {
        recalibrationStartTime = 0;
    }
    slouching = newSlouchState;

    Serial.print("AngleY: ");
    Serial.print(angleY, 2);
    Serial.print(" | Baseline: ");
    Serial.print(baselineAngleY, 2);
    Serial.print(" | Slouching: ");
    Serial.println(slouching ? "YES" : "NO");

    delay(50);  
}

void calibrateUpright() {
    baselineAngleY = angleY;
    Serial.println("Angle reset to zero!");
}

bool detectSlouch(float angle) {
    if (abs(angle - baselineAngleY) > SLOUCH_THRESHOLD) {
        if (slouchStartTime == 0) slouchStartTime = millis();
        if (millis() - slouchStartTime > SLOUCH_TIME) {
            return true;
        }
    } else {
        slouchStartTime = 0;
    }
    return false;
}

float kalmanFilter(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angleKalman += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};

    float y = newAngle - angleKalman;
    angleKalman += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angleKalman;
}
