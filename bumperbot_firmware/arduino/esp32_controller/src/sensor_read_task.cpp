
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "imu/mpu6050_driver.hpp"
#include "protocol/diff_drive_data.hpp"
#include "serial_message_processor.hpp"
#include "task_shared_data.hpp"
#include "wire_i2c_bus.hpp"

void task_delay_func(unsigned long ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void sensorReadTask(void* pvParameters) {
    Wire.begin();

    auto p_task_data = static_cast<TaskSharedData*>(pvParameters);
    MPU6050<WireI2cBus> imu_sensor{WireI2cBus{Wire}, task_delay_func};
    uint32_t notify_value{};

    for (;;) {
        if (xTaskNotifyWait(0, ULONG_MAX, &notify_value, portMAX_DELAY) == pdTRUE) {
            const SensorTaskEvent event = std::bit_cast<SensorTaskEvent>(notify_value);

            switch (event.id) {
                case SensorTaskEventId::ImuConfig: {
                    ImuConfigData imu_config_data{
                        .calibrate_period_ms = static_cast<uint16_t>(event.payload),
                        .result = false};
                    if (imu_sensor.connect()) {
                        constexpr uint16_t kSampleIntervalMs{10};
                        const uint16_t sample_count{
                            static_cast<uint16_t>(imu_config_data.calibrate_period_ms / kSampleIntervalMs)};
                        imu_config_data.result =
                            imu_sensor.calibrate(sample_count, kSampleIntervalMs).has_value();
                    }
                    sendSerialMessage(imu_config_data);
                } break;
                case SensorTaskEventId::SensorRead: {
                    // Delay the task for the specified number of milliseconds
                    // in order to send the latest sensor data
                    const uint8_t response_delay_ms =
                        static_cast<uint8_t>(event.payload);
                    vTaskDelay(pdMS_TO_TICKS(response_delay_ms));

                    // send latest velocity data in response
                    DiffDriveStateData state{
                        .velocity = {
                            .right_wheel_velocity = 0.0f,
                            .left_wheel_velocity = 0.0f
                        }};
                    xQueuePeek(p_task_data->diff_drive_state_queue, &state.velocity, 0);
                    sendSerialMessage(state);
                } break;
                default:
                    // Handle unknown event
                    break;
            }
        }
    }
}