#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "diff_drive/diff_drive_constants.hpp"
#include "diff_drive/wheel_controller.hpp"
#include "task_shared_data.hpp"

namespace {

// ISRs for wheel encoder callbacks
WheelController* p_right_wheel;
WheelController* p_left_wheel;

// Task control variables
uint32_t dt_ms;
TickType_t main_loop_ticks;
TickType_t last_valid_msg_time_ticks;

// Wheels configure, activate, deactivete callbacks
void activateWheels() {
    p_right_wheel->setActive(true);
    p_left_wheel->setActive(true);
    last_valid_msg_time_ticks = xTaskGetTickCount();
};

void deactivateWheels() {
    p_right_wheel->setActive(false);
    p_left_wheel->setActive(false);
};

void configureWheels(const DiffDriveConfigData& config_data) {
    dt_ms = 1000 / config_data.control_rate_hz;
    main_loop_ticks = pdMS_TO_TICKS(dt_ms);

    p_right_wheel->configure(dt_ms, config_data.right_wheel);
    p_left_wheel->configure(dt_ms, config_data.left_wheel);

    // Safety measure: drop torque when PID tunings change
    deactivateWheels();
};

}  // namespace

void diffDriveControlTask(void* pvParameters) {
    auto p_task_data = static_cast<TaskSharedData*>(pvParameters);

    WheelController right_wheel{
        kRightMotorDirectionPin, kRightMotorSpeedCommandPin, kRightMotorSpeedStatePin,
        kRightMotorPulsePerRevolution, true /*invert_logic*/
    };

    WheelController left_wheel{
        kLeftMotorDirectionPin, kLeftMotorSpeedCommandPin, kLeftMotorSpeedStatePin,
        kLeftMotorPulsePerRevolution, false /*invert_logic*/
    };

    p_right_wheel = &right_wheel;
    p_left_wheel = &left_wheel;

    // Initial wheels configuration loop
    for (;;) {
        DiffDriveConfigData initial_config;
        if (xQueueReceive(p_task_data->diff_drive_config_queue, &initial_config, portMAX_DELAY) ==
            pdPASS) {
            configureWheels(initial_config);
            break;
        }
    }

    // Configure PINs
    right_wheel.begin();
    left_wheel.begin();

    // Time variables setup
    auto last_wake_time = xTaskGetTickCount();
    last_valid_msg_time_ticks = last_wake_time;

    // Main real-time PID control task loop
    for (;;) {
        {
            DiffDriveConfigData config;
            if (xQueueReceive(p_task_data->diff_drive_config_queue, &config, 0) == pdPASS) {
                configureWheels(config);
            }
        }

        DiffDriveVelocityData velocity_data;
        if (xQueueReceive(p_task_data->diff_drive_command_queue, &velocity_data, 0) == pdPASS) {
            right_wheel.setTargetVelocity(velocity_data.right_wheel_velocity);
            left_wheel.setTargetVelocity(velocity_data.left_wheel_velocity);
            activateWheels();
        }

        // Deactivate wheels upon request
        if (ulTaskNotifyTakeIndexed(kDeactivateNotifyIndex, pdTRUE, 0) > 0) {
            deactivateWheels();
        }

        // Emergency stop if no valid velocity message received within timeout
        if ((xTaskGetTickCount() - last_valid_msg_time_ticks) * portTICK_PERIOD_MS >=
            kEmergencyStopTimeoutMs) {
            deactivateWheels();
        }

        // PID control update for each wheel
        right_wheel.update(dt_ms);
        left_wheel.update(dt_ms);

        // Publish current wheel velocity states for other tasks to consume
        velocity_data.right_wheel_velocity = right_wheel.getCurrentVelocity();
        velocity_data.left_wheel_velocity = left_wheel.getCurrentVelocity();
        xQueueOverwrite(p_task_data->diff_drive_state_queue, &velocity_data);

        vTaskDelayUntil(&last_wake_time, main_loop_ticks);
    }
}