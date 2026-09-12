#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <algorithm>
#include <optional>

#include "imu/mpu6050_driver.hpp"
#include "protocol/system_data.hpp"
#include "task_shared_data.hpp"
#include "utils.hpp"
#include "wire_i2c_bus.hpp"

namespace {

constexpr uint8_t kImuInterruptPin{A6};
constexpr uint32_t kI2cClockHz{200'000U};
constexpr uint16_t kCalibrationSampleIntervalMs{10U};
constexpr uint8_t kMaxRetries{3u};

TaskHandle_t g_sensor_read_task_handle{nullptr};

void IRAM_ATTR handleImuDataReady() noexcept {
    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(g_sensor_read_task_handle, &higher_priority_task_woken);

    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

ImuStateData toSystemImuState(const mpu6050::IMUData& imu) {
    return ImuStateData{
        .angular_velocity_x = imu.gyroX,
        .angular_velocity_y = imu.gyroY,
        .angular_velocity_z = imu.gyroZ,
        .linear_acceleration_x = imu.accelX,
        .linear_acceleration_y = imu.accelY,
        .linear_acceleration_z = imu.accelZ,
    };
}

void publishImuState(TaskSharedData& shared_data, const std::optional<mpu6050::IMUData>& imu_data) {
    ImuTaskState state{
        .data = {},
        .sample_time_ticks = xTaskGetTickCount(),
        .valid = imu_data.has_value(),
    };

    if (imu_data) {
        state.data = toSystemImuState(*imu_data);
    }

    xQueueOverwrite(shared_data.imu_state_queue, &state);
}

template <typename Imu>
bool configureImu(Imu& imu_sensor, TaskSharedData& shared_data, const ImuConfigData& request) {
    bool imu_ready = false;
    xQueueReset(shared_data.imu_state_queue);
    ImuConfigData response{.calibrate_period_ms = request.calibrate_period_ms, .result = false};

    do {
        if (request.calibrate_period_ms < kCalibrationSampleIntervalMs) {
            break;
        }

        if (imu_sensor.isConnected()) {
            // Reconfiguration must not race DATA_RDY reads with calibration reads.
            if (!utils::retry<kMaxRetries>(&Imu::disableInterrupts, imu_sensor)) {
                break;
            }
        } else {
            if (!utils::retry<kMaxRetries>(&Imu::connect, imu_sensor)) {
                break;
            }
        }

        const uint16_t sample_count =
            static_cast<uint16_t>(request.calibrate_period_ms / kCalibrationSampleIntervalMs);
        auto fn_delay = [](unsigned long ms) { vTaskDelay(pdMS_TO_TICKS(ms)); };
        if (!utils::retry<kMaxRetries>(&Imu::calibrate, imu_sensor, fn_delay, sample_count,
                                       kCalibrationSampleIntervalMs)) {
            break;
        }

        pinMode(kImuInterruptPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(kImuInterruptPin), handleImuDataReady, RISING);

        if (!utils::retry<kMaxRetries>(&Imu::enableDataReadyInterrupt, imu_sensor)) {
            detachInterrupt(digitalPinToInterrupt(kImuInterruptPin));
            break;
        }

        response.result = imu_ready = true;
    } while (false);

    xQueueOverwrite(shared_data.imu_config_response_queue, &response);
    return imu_ready;
}

}  // namespace

void sensorReadTask(void* pvParameters) {
    auto& shared_data = *static_cast<TaskSharedData*>(pvParameters);

    g_sensor_read_task_handle = xTaskGetCurrentTaskHandle();
    configASSERT(g_sensor_read_task_handle != nullptr);

    Wire.begin();
    Wire.setClock(kI2cClockHz);

    MPU6050<WireI2cBus> imu_sensor{WireI2cBus{Wire}};

    bool interrupt_attached{false};
    bool imu_ready{false};

    for (;;) {
        // Both ImuConfig and DATA_RDY use this counting notification only as a
        // wake-up primitive. ImuConfig payload itself lives in a queue.
        std::ignore = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Configuration always takes precedence. If a DATA_RDY event arrived
        // concurrently, it can be discarded because calibration invalidates
        // all pre-configuration samples anyway.
        ImuConfigData config{};
        if (xQueueReceive(shared_data.imu_config_queue, &config, 0) == pdPASS) {
            imu_ready = configureImu(imu_sensor, shared_data, config);
            continue;
        }

        if (!imu_ready) {
            continue;
        }

        // If multiple DATA_RDY notifications accumulated while this task was
        // delayed, read once. MPU6050 data registers contain the latest sample;
        // retry calibrated reads only if the previous read attempt failed.
        publishImuState(shared_data, utils::retry<kMaxRetries>(
                                         &decltype(imu_sensor)::readCalibrated, imu_sensor));
    }
}