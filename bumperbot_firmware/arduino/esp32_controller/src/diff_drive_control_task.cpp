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
    
    // Bind ISRs and hardware
    p_right_wheel = &right_wheel;
    p_left_wheel = &left_wheel;
    right_wheel.begin();
    left_wheel.begin();

    attachInterrupt(kPinRwEncoderPhaseA, rightWheelEncoderCallback, CHANGE);
    attachInterrupt(kPinLwEncoderPhaseA, leftWheelEncoderCallback, CHANGE);

    // Apply default PID control loop configuration
    double dt_sec = 1.0 / kDefaultPidControlRate;
    auto loop_frequency_ticks = pdMS_TO_TICKS(static_cast<uint32_t>(dt_sec * 1000.0));

    auto last_wake_time = xTaskGetTickCount();
    auto last_valid_msg_time_ticks = last_wake_time;

    auto activate_wheels = [&right_wheel, &left_wheel, &last_valid_msg_time_ticks] {
        right_wheel.setActive(true);
        left_wheel.setActive(true);
        last_valid_msg_time_ticks = xTaskGetTickCount();
    };

    auto deactivate_wheels = [&right_wheel, &left_wheel] {
        right_wheel.setActive(false);
        left_wheel.setActive(false);
    };

    // Main REAL-TIME task loop
    for (;;) {
        // Process Queued messages from other tasks
        ConfigData pending_config;
        if (xQueueReceive(p_task_data->config_message_queue, &pending_config, 0) == pdPASS) {
            right_wheel.configure(pending_config.pid_rate, pending_config.r_wheel);
            left_wheel.configure(pending_config.pid_rate, pending_config.l_wheel);

            dt_sec = 1.0 / pending_config.pid_rate;
            loop_frequency_ticks = pdMS_TO_TICKS(static_cast<uint32_t>(dt_sec * 1000.0));

            // Safety measure: drop torque when PID tunings change
            deactivate_wheels();
        }

        VelocityData velocity_data;
        if (xQueueReceive(p_task_data->target_velocity_message_queue, &velocity_data, 0) == pdPASS) {
            right_wheel.setTargetVelocity(velocity_data.right_wheel_velocity);
            left_wheel.setTargetVelocity(velocity_data.left_wheel_velocity);
            activate_wheels();
        }

        // deactivate wheels if requested by the serial input task
        if (ulTaskNotifyTakeIndexed(kDeactivateNotifyIndex, pdTRUE, 0) > 0) {
            deactivate_wheels();
        }

        // Emergency stop if no valid velocity message received within timeout
        if ((xTaskGetTickCount() - last_valid_msg_time_ticks) * portTICK_PERIOD_MS >= kEmergencyStopTimeoutMs) {
            deactivate_wheels();
        }

        // PID control update for each wheel
        right_wheel.update(dt_sec);
        left_wheel.update(dt_sec);

        // Publish current wheel velocities for other tasks to consume
        velocity_data.right_wheel_velocity = right_wheel.getCurrentVelocity();
        velocity_data.left_wheel_velocity = left_wheel.getCurrentVelocity();
        xQueueOverwrite(p_task_data->current_velocity_queue, &velocity_data);

        vTaskDelayUntil(&last_wake_time, loop_frequency_ticks);
    }
}