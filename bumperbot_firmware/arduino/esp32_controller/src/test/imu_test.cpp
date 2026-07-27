#include <Arduino.h>
#include <Wire.h>

#include "imu/mpu6050_driver.hpp"
#include "wire_i2c_bus.hpp"

MPU6050<WireI2cBus> imu_sensor{WireI2cBus{Wire}, [](unsigned long ms) { delay(ms); }};

void setup() {
    Serial.begin(9600);
    Wire.begin();

    while (!Serial) {
        delay(10);
    }

    Serial.println("Initializing MPU6050...");

    if (!imu_sensor.connect()) {
        while (true) {
            Serial.println("[CRITICAL ERROR]: Failed to connect to MPU6050.");
            delay(1000);  // Halt execution if connection fails
        }
    }

    Serial.println("MPU6050 initialized.");
    Serial.println("Starting calibration (keep sensor flat)...");

    auto calOpt = imu_sensor.calibrate();
    if (!calOpt.has_value()) {
        while (true) {
            Serial.println("[CRITICAL ERROR]: Failed to calibrate MPU6050.");
            delay(1000);  // Halt execution if calibration fails
        }
    }

    const auto& cal = calOpt.value();
    Serial.printf(
        "Calibration offsets. Accel X: %.4f, Accel Y: %.4f, Accel Z: %.4f "
        "| Gyro X: %.4f, Gyro Y: %.4f, Gyro Z: %.4f\n",
        cal.accelX, cal.accelY, cal.accelZ, cal.gyroX, cal.gyroY, cal.gyroZ);
}

void loop() {
    if (imu_sensor.isConnected()) {
        const auto metricsOpt = imu_sensor.readCalibrated();
        if (metricsOpt.has_value()) {
            Serial.printf(
                "Accel X: %.2f, Accel Y: %.2f, Accel Z: %.2f "
                "| Gyro X: %.2f, Gyro Y: %.2f, Gyro Z: %.2f\n",
                metricsOpt->accelX, metricsOpt->accelY, metricsOpt->accelZ, metricsOpt->gyroX,
                metricsOpt->gyroY, metricsOpt->gyroZ);
        } else {
            Serial.println("[WARNING]: Failed to read calibrated data from MPU6050.");
        }
    }
    delay(10);
}