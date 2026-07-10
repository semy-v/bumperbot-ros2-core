#ifndef TASK_SHARED_DATA_HPP
#define TASK_SHARED_DATA_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <bit>
#include <cstdint>

// task function prototypes
void diffDriveControlTask(void* pvParameters);
void serialProcessTask(void* pvParameters);
void sensorReadTask(void* pvParameters);

// diff drive control task event notifications
constexpr uint8_t kDeactivateNotifyIndex{0u};

// sensor read task event notifications
enum class SensorTaskEventId : uint32_t { SensorRead = 0u, ImuConfig = 1u };

struct SensorTaskEvent {
    SensorTaskEventId id : 2;  // 2 bits for event ID
    uint32_t payload : 30;     // 30 bits for payload
};

static_assert(sizeof(SensorTaskEvent) == sizeof(uint32_t), "SensorTaskEvent size mismatch!");

// shared data structure for inter-task usage
struct TaskSharedData {
    QueueHandle_t diff_drive_config_queue;
    QueueHandle_t diff_drive_command_queue;
    QueueHandle_t diff_drive_state_queue;
    TaskHandle_t diff_drive_control_task_handle;
    TaskHandle_t sensor_read_task_handle;
};

#endif  // TASK_SHARED_DATA_HPP