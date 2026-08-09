/**
 * @file main.cpp
 *
 * Runtime ownership model:
 *   - DiffDriveControlTask (Core 1): wheel command/control and wheel-state queue.
 *   - SensorReadTask       (Core 0): exclusive Wire/I2C owner. It wakes from
 *     MPU6050 DATA_RDY, reads one latest sample and overwrites imu_state_queue.
 *   - SerialProcessTask    (Core 0): exclusive Serial TX owner. DiffDriveCommand
 *     reception reserves a response deadline; at that deadline the task snapshots
 *     latest IMU and wheel states and transmits SystemStateData.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "protocol/system_data.hpp"
#include "serial_message_processor.hpp"
#include "task_shared_data.hpp"

namespace {

// Core 0: prioritize response scheduling. SensorReadTask still runs immediately
// whenever SerialProcessTask is blocked in its 1 ms delay.
constexpr UBaseType_t kSerialProcessTaskPriority{2};
constexpr BaseType_t kSerialProcessTaskCpuCore{0};

constexpr UBaseType_t kSensorReadTaskPriority{1};
constexpr BaseType_t kSensorReadTaskCpuCore{0};

// Core 1 is dedicated to wheel control.
constexpr UBaseType_t kDiffDriveControlTaskPriority{2};
constexpr BaseType_t kDiffDriveControlTaskCpuCore{1};

}  // namespace

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(1);
    }

    static TaskSharedData task_shared_data{
        .diff_drive_config_queue = xQueueCreate(1, sizeof(DiffDriveConfigData)),
        .diff_drive_command_queue = xQueueCreate(1, sizeof(DiffDriveVelocityData)),
        .diff_drive_state_queue = xQueueCreate(1, sizeof(DiffDriveVelocityData)),
        .imu_config_queue = xQueueCreate(1, sizeof(ImuConfigData)),
        .imu_config_response_queue = xQueueCreate(1, sizeof(ImuConfigData)),
        .imu_state_queue = xQueueCreate(1, sizeof(ImuTaskState)),
        .system_state_response_queue = xQueueCreate(1, sizeof(SystemStateResponseRequest)),
        .diff_drive_control_task_handle = nullptr,
        .sensor_read_task_handle = nullptr,
    };

    configASSERT(task_shared_data.diff_drive_config_queue != nullptr);
    configASSERT(task_shared_data.diff_drive_command_queue != nullptr);
    configASSERT(task_shared_data.diff_drive_state_queue != nullptr);
    configASSERT(task_shared_data.imu_config_queue != nullptr);
    configASSERT(task_shared_data.imu_config_response_queue != nullptr);
    configASSERT(task_shared_data.imu_state_queue != nullptr);
    configASSERT(task_shared_data.system_state_response_queue != nullptr);

    configASSERT(xTaskCreatePinnedToCore(
                     diffDriveControlTask, "DiffDriveControlTask", 4096, &task_shared_data,
                     kDiffDriveControlTaskPriority, &task_shared_data.diff_drive_control_task_handle,
                     kDiffDriveControlTaskCpuCore) == pdPASS);

    configASSERT(xTaskCreatePinnedToCore(
                     sensorReadTask, "SensorReadTask", 4096, &task_shared_data,
                     kSensorReadTaskPriority, &task_shared_data.sensor_read_task_handle,
                     kSensorReadTaskCpuCore) == pdPASS);

    // Create SerialProcessTask last: its higher Core-0 priority can preempt setup,
    // so all queues and the SensorReadTask handle must already be valid.
    configASSERT(xTaskCreatePinnedToCore(serialProcessTask, "SerialProcessTask", 3072,
                                         &task_shared_data, kSerialProcessTaskPriority, nullptr,
                                         kSerialProcessTaskCpuCore) == pdPASS);

    vTaskDelete(nullptr);
}

void loop() {}
