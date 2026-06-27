#ifndef TASK_SHARED_DATA_HPP
#define TASK_SHARED_DATA_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

void diffDriveControlTask(void *pvParameters);
void serialProcessTask(void *pvParameters);

constexpr uint8_t kDeactivateNotifyIndex{0u};

struct TaskSharedData {
    QueueHandle_t config_message_queue;
    QueueHandle_t current_velocity_queue;
    QueueHandle_t target_velocity_message_queue;
    TaskHandle_t diff_drive_control_task_handle;
};

#endif // TASK_SHARED_DATA_HPP