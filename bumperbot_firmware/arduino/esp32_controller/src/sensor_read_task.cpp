#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "task_shared_data.hpp"
#include "serial_message_processor.hpp"
#include "protocol/diff_drive_data.hpp"

void sensorReadTask(void *pvParameters) {
    auto p_task_data = static_cast<TaskSharedData*>(pvParameters);
    uint32_t response_delay_ms;

    for (;;) {
        if (xTaskNotifyWait(0, ULONG_MAX, &response_delay_ms, portMAX_DELAY) == pdTRUE) {
            // Delay the task for the specified number of milliseconds
            // before sending the latest sensor data
            if (response_delay_ms > 0) {
                vTaskDelay(pdMS_TO_TICKS(response_delay_ms));
            }

            // send latest velocity data in response
            VelocityData vel_data{
                .right_wheel_velocity = 0.0,
                .left_wheel_velocity = 0.0,
            };
            xQueuePeek(p_task_data->current_velocity_queue, &vel_data, 0);
            vel_data.response_delay_ms = static_cast<uint8_t>(response_delay_ms);
            sendSerialMessage(vel_data);
        }
    }
}