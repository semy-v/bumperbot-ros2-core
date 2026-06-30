#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "task_shared_data.hpp"
#include "quadrature_encoder.hpp"
#include "l298n_motor.hpp"
#include "wheel_controller.hpp"

namespace {

// right wheel control pins
constexpr uint8_t kPinL298EnA{9};
constexpr uint8_t kPinL298In1{12};
constexpr uint8_t kPinL298In2{10};
constexpr uint8_t kPinRwEncoderPhaseA{3};
constexpr uint8_t kPinRwEncoderPhaseB{5};

// left wheel control pins
constexpr uint8_t kPinL298EnB{11};
constexpr uint8_t kPinL298In3{7};
constexpr uint8_t kPinL298In4{8};
constexpr uint8_t kPinLwEncoderPhaseA{2};
constexpr uint8_t kPinLwEncoderPhaseB{4};

// unconfigurable constants
constexpr unsigned long kEmergencyStopTimeoutMs{2000};
constexpr double kPulsePerRevolution{1280.0};

// configurable constants (overriden by Config message)
constexpr double kDefaultPidControlRate{25.0}; // Hz

constexpr WheelConfig kDefaultRightMotorConfig{
  .kp = 15.5,
  .ki = 39.0,
  .kd = 0.0,
  .pwm_deadband = 17
};

constexpr WheelConfig kDefaultLeftMotorConfig{
  .kp = 14.2,
  .ki = 43.0,
  .kd = 0.0,
  .pwm_deadband = 18
};

// ISRs for wheel encoder callbacks
WheelController* p_right_wheel{nullptr};
WheelController* p_left_wheel{nullptr};

void ARDUINO_ISR_ATTR rightWheelEncoderCallback() {
  p_right_wheel->encoder().update();
}

void ARDUINO_ISR_ATTR leftWheelEncoderCallback() {
  p_left_wheel->encoder().update();
}

// Task control variables
double dt_sec{};
TickType_t main_loop_ticks{};
TickType_t last_valid_msg_time_ticks{};

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

void configureWheels(const ConfigData& config_data) {
    p_right_wheel->configure(config_data.pid_rate, config_data.r_wheel);
    p_left_wheel->configure(config_data.pid_rate, config_data.l_wheel);

    dt_sec = 1.0 / config_data.pid_rate;
    main_loop_ticks = pdMS_TO_TICKS(static_cast<uint32_t>(dt_sec * 1000.0));

    // Safety measure: drop torque when PID tunings change
    deactivateWheels();
};

} // namespace


void diffDriveControlTask(void *pvParameters) {
    auto p_task_data = static_cast<TaskSharedData*>(pvParameters);

    WheelController right_wheel{
        L298NMotor{kPinL298EnA, kPinL298In1, kPinL298In2} /*motor*/,
        QuadratureEncoder{kPinRwEncoderPhaseA, kPinRwEncoderPhaseB} /*encoder*/,
        kDefaultPidControlRate /*pid_control_rate*/,
        kDefaultRightMotorConfig /*wheel_config*/,
        kPulsePerRevolution /*ticks_per_rev*/,
        false /*invert_logic*/
    };

    WheelController left_wheel{
        L298NMotor{kPinL298EnB, kPinL298In3, kPinL298In4} /*motor*/,
        QuadratureEncoder{kPinLwEncoderPhaseA, kPinLwEncoderPhaseB} /*encoder*/,
        kDefaultPidControlRate /*pid_control_rate*/,
        kDefaultLeftMotorConfig /*wheel_config*/,
        kPulsePerRevolution /*ticks_per_rev*/,
        true /*invert_logic*/
    };

    p_right_wheel = &right_wheel;
    p_left_wheel = &left_wheel;

    // Initial wheels configuration loop
    for(;;) {
        ConfigData initial_config;
        if (xQueueReceive(p_task_data->config_message_queue, &initial_config, portMAX_DELAY) == pdPASS) {
            configureWheels(initial_config);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Bind ISRs and hardware
    right_wheel.begin();
    left_wheel.begin();
    attachInterrupt(kPinRwEncoderPhaseA, rightWheelEncoderCallback, CHANGE);
    attachInterrupt(kPinLwEncoderPhaseA, leftWheelEncoderCallback, CHANGE);

    // Time variables setup
    auto last_wake_time = xTaskGetTickCount();
    last_valid_msg_time_ticks = last_wake_time;

    // Main real-time PID control task loop
    for (;;) {
        ConfigData pending_config;
        if (xQueueReceive(p_task_data->config_message_queue, &pending_config, 0) == pdPASS) {
            configureWheels(pending_config);
        }

        VelocityData velocity_data;
        if (xQueueReceive(p_task_data->target_velocity_message_queue, &velocity_data, 0) == pdPASS) {
            right_wheel.setTargetVelocity(velocity_data.right_wheel_velocity);
            left_wheel.setTargetVelocity(velocity_data.left_wheel_velocity);
            activateWheels();
        }

        // deactivate wheels if requested by the serial input task
        if (ulTaskNotifyTakeIndexed(kDeactivateNotifyIndex, pdTRUE, 0) > 0) {
            deactivateWheels();
        }

        // Emergency stop if no valid velocity message received within timeout
        if ((xTaskGetTickCount() - last_valid_msg_time_ticks) * portTICK_PERIOD_MS >= kEmergencyStopTimeoutMs) {
            deactivateWheels();
        }

        // PID control update for each wheel
        right_wheel.update(dt_sec);
        left_wheel.update(dt_sec);

        // Publish current wheel velocities for other tasks to consume
        velocity_data.right_wheel_velocity = right_wheel.getCurrentVelocity();
        velocity_data.left_wheel_velocity = left_wheel.getCurrentVelocity();
        xQueueOverwrite(p_task_data->current_velocity_queue, &velocity_data);

        vTaskDelayUntil(&last_wake_time, main_loop_ticks);
    }
}