#ifndef TASK_SHARED_DATA_HPP
#define TASK_SHARED_DATA_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstdint>

#include "protocol/system_data.hpp"

// Task function prototypes.
void diffDriveControlTask(void* pvParameters);
void serialProcessTask(void* pvParameters);
void sensorReadTask(void* pvParameters);

// Diff-drive control task event notifications.
constexpr uint8_t kDeactivateNotifyIndex{0U};

// Latest IMU sample shared from SensorReadTask to SerialProcessTask.
// Queue length is one and SensorReadTask overwrites it on each DATA_RDY event.
struct ImuTaskState {
    ImuStateData data{};
    TickType_t sample_time_ticks{0};
    bool valid{false};
};

// A DiffDriveCommand reserves exactly one SystemStateData response.
// Keeping this in a one-element queue makes "one outstanding command" an
// explicit protocol invariant without introducing shared atomics.
struct SystemStateResponseRequest {
    int64_t due_time_us{0};
};

struct TaskSharedData {
    QueueHandle_t diff_drive_config_queue;
    QueueHandle_t diff_drive_command_queue;
    QueueHandle_t diff_drive_state_queue;

    QueueHandle_t imu_config_queue;
    QueueHandle_t imu_config_response_queue;
    QueueHandle_t imu_state_queue;

    QueueHandle_t system_state_response_queue;

    TaskHandle_t diff_drive_control_task_handle;
    TaskHandle_t sensor_read_task_handle;
};

#endif  // TASK_SHARED_DATA_HPP