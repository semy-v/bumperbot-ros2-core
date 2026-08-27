#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <cstdint>
#include <optional>

#include "diff_drive/bldc2430_encoder.hpp"
#include "diff_drive/bldc2430_motor.hpp"
#include "diff_drive/bldc2430_pulse_counter.hpp"
#include "diff_drive/diff_drive_constants.hpp"
#include "wheel_rotation_sequence_runner.hpp"

namespace {

constexpr uint32_t kSerialBaudRate{115200U};

// Velocity estimation is refreshed periodically.  Full-revolution boundaries
// are event-driven by the dedicated BLDC2430PulseCounter and are not limited by
// this update period.
constexpr uint32_t kVelocityUpdatePeriodMs{10U};

// Each full-revolution counter consumes the same FG and direction GPIOs as the
// velocity encoder but owns a separate PCNT hardware unit.  Because only FG
// falling edges are counted, these limits must be the empirically measured
// FALLING-EDGE count per output-wheel revolution (306 for the current BLDC2430
// 35:1 installation), not the previous rising+falling transition count of 612.
BLDC2430PulseCounter left_full_revolution_counter{
    kLeftMotorDirectionPin,
    kLeftMotorSpeedStatePin,
    kLeftMotorPulsePerRevolution,
    -kLeftMotorPulsePerRevolution,
    false,
};

BLDC2430PulseCounter right_full_revolution_counter{
    kRightMotorDirectionPin,
    kRightMotorSpeedStatePin,
    kRightMotorPulsePerRevolution,
    -kRightMotorPulsePerRevolution,
    true,
};

// Velocity encoders intentionally expose no custom PCNT limits.  Internally
// they always use +1/-1 so every falling edge contributes one full FG period to
// the reciprocal-period velocity estimator.
BLDC2430Encoder left_velocity_encoder{
    kLeftMotorDirectionPin,
    kLeftMotorSpeedStatePin,
    false,
};

BLDC2430Encoder right_velocity_encoder{
    kRightMotorDirectionPin,
    kRightMotorSpeedStatePin,
    true,
};

BLDC2430Motor left_motor{kLeftMotorDirectionPin, kLeftMotorSpeedCommandPin, false};
BLDC2430Motor right_motor{kRightMotorDirectionPin, kRightMotorSpeedCommandPin, true};

constexpr std::array kWheelRotationSequence{
    WheelRotationStep{50, 3},    WheelRotationStep{100, 4}, WheelRotationStep{255, 15},
    WheelRotationStep{50, 3},    WheelRotationStep{-50, 3}, WheelRotationStep{-100, 4},
    WheelRotationStep{-255, 15}, WheelRotationStep{-50, 3}, WheelRotationStep{50, 3},
};

using RotationRunner = WheelRotationSequenceRunner<kWheelRotationSequence.size()>;

RotationRunner left_runner{
    left_full_revolution_counter,
    left_velocity_encoder,
    left_motor,
    kVelocityUpdatePeriodMs,
    static_cast<float>(kLeftMotorPulsePerRevolution),
    kWheelRotationSequence,
};

RotationRunner right_runner{
    right_full_revolution_counter,
    right_velocity_encoder,
    right_motor,
    kVelocityUpdatePeriodMs,
    static_cast<float>(kRightMotorPulsePerRevolution),
    kWheelRotationSequence,
};

bool completion_reported{false};

void printCompletedStep(const char* wheel_name,
                        const std::optional<RotationRunner::MeasuredPwmVelocity>& result) {
    if (!result) {
        return;
    }

    Serial.printf("%-5s wheel | PWM: %4d | velocity: %8.3f rad/s", wheel_name, result->pwm_speed,
                  result->velocity);

    if (result->excess_revolution_events != 0U) {
        Serial.printf(" | WARNING: %lu excess revolution event(s)",
                      static_cast<unsigned long>(result->excess_revolution_events));
    }

    Serial.println();
}

void processRunnerEventsImmediately() {
    // Process both event counters before velocity estimation or Serial output.
    // A pending full-revolution event therefore receives the next PWM command
    // with the minimum possible task-context latency.
    left_runner.processPendingRevolutionEvents();
    right_runner.processPendingRevolutionEvents();
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);

    constexpr uint32_t kSerialWaitTimeoutMs{1000U};
    const uint32_t wait_start_ms = millis();
    while (!Serial && millis() - wait_start_ms < kSerialWaitTimeoutMs) {
        delay(10);
    }

    Serial.println();
    Serial.println("Bumperbot differential-drive rotation diagnostic");
    Serial.printf(
        "Steps: %u | velocity update period: %lu ms | revolution response: PCNT ISR wake-up\n",
        static_cast<unsigned>(kWheelRotationSequence.size()),
        static_cast<unsigned long>(kVelocityUpdatePeriodMs));
    Serial.printf("Falling-edge counts/revolution: left=%d right=%d\n",
                  static_cast<int>(kLeftMotorPulsePerRevolution),
                  static_cast<int>(kRightMotorPulsePerRevolution));

    const TaskHandle_t diagnostic_task = xTaskGetCurrentTaskHandle();
    configASSERT(diagnostic_task != nullptr);

    pinMode(kMotorsPowerEnablePin, OUTPUT);
    digitalWrite(kMotorsPowerEnablePin, HIGH);

    // begin() stores the task handle before enabling each full-revolution PCNT
    // interrupt, so the ISR never notifies an uninitialized task handle.
    left_runner.begin(diagnostic_task);
    right_runner.begin(diagnostic_task);

    // Remove any stale notification left by setup/reset activity.  Wheel-specific
    // revolution counts are maintained independently inside the two runners.
    (void)ulTaskNotifyTake(pdTRUE, 0U);

    left_runner.run();
    right_runner.run();

    Serial.println("Rotation sequence started.");
}

void loop() {
    // Wake immediately when either full-revolution counter reaches its limit;
    // otherwise wake periodically to refresh velocity and diagnostic output.
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kVelocityUpdatePeriodMs));

    processRunnerEventsImmediately();

    left_runner.updateVelocity();
    right_runner.updateVelocity();

    printCompletedStep("Left", left_runner.takeLastStepResult());
    printCompletedStep("Right", right_runner.takeLastStepResult());

    if (!completion_reported && left_runner.isFinished() && right_runner.isFinished()) {
        completion_reported = true;
        Serial.println("Rotation sequence completed successfully; both motors are stopped.");
    }
}