#include <Arduino.h>
#include <Wire.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "imu/mpu6050_driver.hpp"
#include "wire_i2c_bus.hpp"

namespace {

constexpr uint32_t kSerialBaudRate{115200U};
constexpr uint32_t kI2cClockHz{400'000U};

// MPU6050 INT is physically connected to Arduino Nano ESP32 A6.
constexpr uint8_t kImuInterruptPin{A6};

// MPU6050 is configured for 100 Hz DATA_RDY. A timeout substantially longer
// than one sample period makes a broken/missing interrupt observable.
constexpr uint32_t kInterruptTimeoutMs{100U};

// Reading remains interrupt-driven at the full 100 Hz sensor rate. Serial
// output is decimated to avoid making UART logging the bottleneck.
constexpr uint32_t kPrintEverySamples{10U};

MPU6050<WireI2cBus> imu_sensor{
    WireI2cBus{Wire},
    [](unsigned long ms) { delay(ms); },
};

TaskHandle_t imu_data_task{nullptr};
uint32_t sample_count{0U};
uint32_t missed_notification_count{0U};

[[noreturn]] void haltWithError(const char* message) {
    Serial.printf("[CRITICAL ERROR]: %s\n", message);
    for (;;) {
        delay(1000);
    }
}

void IRAM_ATTR handleImuDataReady() {
    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(
        imu_data_task,
        &higher_priority_task_woken);

    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void printSample(const mpu6050::IMUData& data) {
    Serial.printf(
        "Accel X: %.3f, Accel Y: %.3f, Accel Z: %.3f m/s^2 "
        "| Gyro X: %.4f, Gyro Y: %.4f, Gyro Z: %.4f rad/s\n",
        data.accelX,
        data.accelY,
        data.accelZ,
        data.gyroX,
        data.gyroY,
        data.gyroZ);
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);
    Wire.begin();
    Wire.setClock(kI2cClockHz);

    // Allow a terminal to attach, but do not make standalone operation depend
    // indefinitely on a USB serial connection.
    constexpr uint32_t kSerialWaitMs{1000U};
    const uint32_t wait_start_ms = millis();
    while (!Serial && (millis() - wait_start_ms < kSerialWaitMs)) {
        delay(10);
    }

    Serial.println("Initializing MPU6050...");

    if (!imu_sensor.connect()) {
        haltWithError("Failed to connect to MPU6050.");
    }

    Serial.println("MPU6050 initialized.");
    Serial.println("Starting calibration (keep sensor stationary and flat)...");

    const auto calibration = imu_sensor.calibrate();
    if (!calibration) {
        haltWithError("Failed to calibrate MPU6050.");
    }

    Serial.printf(
        "Calibration offsets | "
        "Accel: [%.4f, %.4f, %.4f] m/s^2 | "
        "Gyro: [%.4f, %.4f, %.4f] rad/s\n",
        calibration->accelX,
        calibration->accelY,
        calibration->accelZ,
        calibration->gyroX,
        calibration->gyroY,
        calibration->gyroZ);

    // setup() and loop() execute in the same Arduino FreeRTOS task. Capture its
    // handle before the GPIO interrupt can ever be enabled.
    imu_data_task = xTaskGetCurrentTaskHandle();
    configASSERT(imu_data_task != nullptr);

    pinMode(kImuInterruptPin, INPUT);

    const int interrupt_number = digitalPinToInterrupt(kImuInterruptPin);
    if (interrupt_number < 0) {
        haltWithError("A6 is not available as an interrupt-capable pin.");
    }

    // MPU INT is configured active-high/push-pull, therefore wake on RISING.
    attachInterrupt(interrupt_number, handleImuDataReady, RISING);

    // Enable DATA_RDY only after the receiving GPIO ISR is fully installed.
    // The MPU interrupt is latched until readCalibrated() performs its sensor
    // register read, so a short 50 us pulse cannot be missed.
    if (!imu_sensor.enableDataReadyInterrupt()) {
        detachInterrupt(interrupt_number);
        haltWithError("Failed to enable MPU6050 DATA_RDY interrupt.");
    }

    Serial.printf(
        "DATA_RDY interrupt enabled at %u Hz on A6. Waiting for samples...\n",
        static_cast<unsigned>(mpu6050::kSampleRateHz));
}

void loop() {
    // Block until DATA_RDY wakes this task. The timeout is diagnostic only and
    // never causes a sensor read; readCalibrated() is called only after an IRQ.
    const uint32_t notification_count = ulTaskNotifyTake(
        pdTRUE,
        pdMS_TO_TICKS(kInterruptTimeoutMs));

    if (notification_count == 0U) {
        Serial.println("[WARNING]: MPU6050 DATA_RDY interrupt timeout.");
        return;
    }

    // MPU6050 output registers are not a FIFO. If multiple IRQ notifications
    // accumulated before this task ran, only the newest register sample is
    // available. Read exactly once and record the coalesced notifications.
    if (notification_count > 1U) {
        missed_notification_count += notification_count - 1U;
    }

    const auto data = imu_sensor.readCalibrated();
    if (!data) {
        Serial.println("[WARNING]: Failed to read calibrated MPU6050 data.");
        return;
    }

    ++sample_count;

    if ((sample_count % kPrintEverySamples) == 0U) {
        printSample(*data);

        if (missed_notification_count != 0U) {
            Serial.printf(
                "[WARNING]: %lu DATA_RDY notification(s) coalesced since startup.\n",
                static_cast<unsigned long>(missed_notification_count));
        }
    }
}