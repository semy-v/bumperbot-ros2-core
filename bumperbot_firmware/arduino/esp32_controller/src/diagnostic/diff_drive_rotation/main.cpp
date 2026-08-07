#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <cstdint>
#include <optional>

#include "diff_drive/bl2418_encoder.hpp"
#include "diff_drive/bl2418_motor.hpp"
#include "diff_drive/diff_drive_constants.hpp"
#include "wheel_rotation_sequence_runner.hpp"

namespace {

constexpr uint32_t kSerialBaudRate{115200U};

// Velocity estimation still runs periodically. Revolution-boundary response is
// event-driven and is not limited by this period.
constexpr uint32_t kVelocityUpdatePeriodMs{10U};

BL2418Encoder left_revolution_encoder{kLeftMotorDirectionPin, kLeftMotorSpeedStatePin,
                                      kLeftMotorPulsePerRevolution, -kLeftMotorPulsePerRevolution,
                                      false};

BL2418Encoder right_revolution_encoder{kRightMotorDirectionPin, kRightMotorSpeedStatePin,
                                       kRightMotorPulsePerRevolution,
                                       -kRightMotorPulsePerRevolution, true};

BL2418Encoder left_velocity_encoder{kLeftMotorDirectionPin, kLeftMotorSpeedStatePin, false};

BL2418Encoder right_velocity_encoder{kRightMotorDirectionPin, kRightMotorSpeedStatePin, true};

BL2418Motor left_motor{kLeftMotorDirectionPin, kLeftMotorSpeedCommandPin, false};

BL2418Motor right_motor{kRightMotorDirectionPin, kRightMotorSpeedCommandPin, true};

constexpr std::array kWheelRotationSequence{
    WheelRotationStep{50, 3},   WheelRotationStep{100, 4}, WheelRotationStep{255, 5},
    WheelRotationStep{50, 3},   WheelRotationStep{-50, 3}, WheelRotationStep{-100, 4},
    WheelRotationStep{-255, 5}, WheelRotationStep{-50, 3}, WheelRotationStep{50, 3}};

using RotationRunner = WheelRotationSequenceRunner<kWheelRotationSequence.size()>;

RotationRunner left_runner{
    left_revolution_encoder, left_velocity_encoder,        left_motor,
    kVelocityUpdatePeriodMs, kLeftMotorPulsePerRevolution, kWheelRotationSequence};

RotationRunner right_runner{
    right_revolution_encoder, right_velocity_encoder,        right_motor,
    kVelocityUpdatePeriodMs,  kRightMotorPulsePerRevolution, kWheelRotationSequence};

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
    // A wheel with a pending full-revolution event receives its next PWM command
    // with minimal task-context latency.
    left_runner.processPendingRevolutionEvents();
    right_runner.processPendingRevolutionEvents();
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);

    // Wait briefly for a development serial terminal, but continue when the
    // diagnostic runs standalone from battery power.
    constexpr uint32_t kSerialWaitTimeoutMs{1000U};
    const uint32_t wait_start_ms = millis();
    while (!Serial && millis() - wait_start_ms < kSerialWaitTimeoutMs) {
        delay(10);
    }

    Serial.println();
    Serial.println("Bumperbot differential-drive rotation diagnostic");
    Serial.printf("Steps: %u | velocity update period: %lu ms | revolution response: ISR wake-up\n",
                  static_cast<unsigned>(kWheelRotationSequence.size()),
                  static_cast<unsigned long>(kVelocityUpdatePeriodMs));

    const TaskHandle_t diagnostic_task = xTaskGetCurrentTaskHandle();
    configASSERT(diagnostic_task != nullptr);

    // Enable power for both motors before starting the rotation sequence.
    pinMode(kMotorsPowerEnablePin, OUTPUT);
    digitalWrite(kMotorsPowerEnablePin, HIGH);

    // Initialize both runners before either motor starts. begin() stores the
    // task handle before enabling each PCNT ISR.
    left_runner.begin(diagnostic_task);
    right_runner.begin(diagnostic_task);

    // Remove any stale notification left by setup/reset activity before starting
    // both sequences. Wheel-specific event counts are maintained independently.
    (void)ulTaskNotifyTake(pdTRUE, 0U);

    left_runner.run();
    right_runner.run();

    Serial.println("Rotation sequence started.");
}

void loop() {
    // Sleep until either encoder completes a revolution. If no event occurs,
    // wake periodically to refresh velocity estimates and diagnostic output.
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kVelocityUpdatePeriodMs));

    // This is intentionally the first work after wake-up.
    processRunnerEventsImmediately();

    // Velocity estimation and logging are lower-priority diagnostic work and
    // happen only after any required motor command transition has been issued.
    left_runner.updateVelocity();
    right_runner.updateVelocity();

    printCompletedStep("Left", left_runner.takeLastStepResult());
    printCompletedStep("Right", right_runner.takeLastStepResult());

    if (!completion_reported && left_runner.isFinished() && right_runner.isFinished()) {
        completion_reported = true;
        Serial.println("Rotation sequence completed successfully; both motors are stopped.");
    }
}