#include <Arduino.h>
#include <array>

#include "diff_drive/bl2418_encoder.hpp"
#include "diff_drive/bl2418_motor.hpp"
#include "diff_drive/diff_drive_constants.hpp"
#include "wheel_rotation_sequence_runner.hpp"

/*
 * Wheel rotation validation test.
 *
 * This test verifies the complete closed-loop behavior of both differential
 * drive wheel assemblies:
 *
 *   BL2418 motor controller
 *       |
 *       +--> PWM speed command
 *       |
 *       +--> CW/CCW direction control
 *       |
 *       +--> FG speed feedback signal
 *                |
 *                +--> ESP32 PCNT hardware pulse counter
 *
 * Each wheel executes an identical predefined rotation sequence. The sequence
 * advances only after the encoder reports the expected number of wheel
 * revolutions through FG pulse counting.
 *
 * The test validates:
 *   - PWM speed control at different operating points.
 *   - Forward and reverse wheel rotation.
 *   - Direction inversion for mirrored left/right motor installation.
 *   - FG pulse counting accuracy.
 *   - Correct pulses-per-revolution calibration.
 *   - Repeatable wheel rotation based on encoder feedback.
 */

/*
 * Encoder instances.
 *
 * Each encoder is configured with the calibrated number of FG edge counts per
 * wheel revolution. The right encoder inverts the PCNT counting direction to
 * compensate for the mirrored motor installation, so positive counts always
 * represent forward wheel rotation.
 */
BL2418Encoder left_encoder{kLeftMotorDirectionPin, kLeftMotorSpeedStatePin,
                           kLeftMotorPulsePerRevolution, -kLeftMotorPulsePerRevolution, false};

BL2418Encoder right_encoder{kRightMotorDirectionPin, kRightMotorSpeedStatePin,
                            kRightMotorPulsePerRevolution, -kRightMotorPulsePerRevolution, true};

BL2418Encoder left_edge_encoder{kLeftMotorDirectionPin, kLeftMotorSpeedStatePin, false};

BL2418Encoder right_edge_encoder{kRightMotorDirectionPin, kRightMotorSpeedStatePin, true};

/*
 * Motor instances.
 *
 * The right wheel motor has its direction inverted because the motor is
 * physically mirrored relative to the left wheel. The BL2418Motor abstraction
 * hides this hardware difference so that positive speed always represents
 * forward wheel rotation.
 */
BL2418Motor left_motor{kLeftMotorDirectionPin, kLeftMotorSpeedCommandPin, false};

BL2418Motor right_motor{kRightMotorDirectionPin, kRightMotorSpeedCommandPin, true};

/*
 * Wheel rotation validation sequence.
 *
 * Each step defines:
 *
 *   { PWM command, number of wheel revolutions }
 *
 * Positive PWM values:
 *   - Rotate the wheel in the configured forward direction.
 *
 * Negative PWM values:
 *   - Rotate the wheel in the configured reverse direction.
 *
 * The sequence intentionally exercises:
 *
 *   1. Low-speed forward rotation.
 *   2. Medium-speed forward rotation.
 *   3. Maximum-speed forward rotation.
 *   4. Direction transition from forward to reverse.
 *   5. Reverse rotation at different speeds.
 *   6. Direction transition back to forward rotation.
 *
 * The purpose is to verify that direction changes and speed changes do not
 * affect encoder pulse counting or wheel revolution tracking.
 */
// constexpr std::array kWheelRotationSequence{
//     WheelRotationStep{50, 3},  WheelRotationStep{100, 4},  WheelRotationStep{255, 5},

//     WheelRotationStep{50, 3},

//     WheelRotationStep{-50, 3}, WheelRotationStep{-100, 4}, WheelRotationStep{-255, 5},

//     WheelRotationStep{-50, 3},

//     WheelRotationStep{50, 3}};

constexpr std::array kFeedForwardRotationSequence{
    WheelRotationStep{-255, 5}, WheelRotationStep{-250, 5}, WheelRotationStep{-240, 5},
    WheelRotationStep{-230, 5}, WheelRotationStep{-220, 5}, WheelRotationStep{-210, 5},
    WheelRotationStep{-200, 5}, WheelRotationStep{-190, 3}, WheelRotationStep{-180, 3},
    WheelRotationStep{-170, 3}, WheelRotationStep{-160, 3}, WheelRotationStep{-150, 3},
    WheelRotationStep{-140, 3}, WheelRotationStep{-130, 3}, WheelRotationStep{-120, 3},
    WheelRotationStep{-110, 3}, WheelRotationStep{-100, 3}, WheelRotationStep{-90, 3},
    WheelRotationStep{-80, 2},  WheelRotationStep{-70, 2},  WheelRotationStep{-60, 2},
    WheelRotationStep{-50, 2},  WheelRotationStep{-40, 2},  WheelRotationStep{-30, 2},
    WheelRotationStep{30, 2},   WheelRotationStep{40, 2},   WheelRotationStep{50, 2},
    WheelRotationStep{60, 2},   WheelRotationStep{70, 2},   WheelRotationStep{80, 2},
    WheelRotationStep{90, 3},   WheelRotationStep{100, 3},  WheelRotationStep{110, 3},
    WheelRotationStep{120, 3},  WheelRotationStep{130, 3},  WheelRotationStep{140, 3},
    WheelRotationStep{150, 3},  WheelRotationStep{160, 3},  WheelRotationStep{170, 3},
    WheelRotationStep{180, 3},  WheelRotationStep{190, 3},  WheelRotationStep{200, 5},
    WheelRotationStep{210, 5},  WheelRotationStep{220, 5},  WheelRotationStep{230, 5},
    WheelRotationStep{240, 5},  WheelRotationStep{250, 5},  WheelRotationStep{255, 5}};

/*
 * Independent validation runners for each wheel.
 *
 * Each runner owns the execution state for one wheel:
 *   - current sequence step,
 *   - completed revolutions,
 *   - associated motor command,
 *   - associated encoder feedback.
 *
 * Both runners execute the same sequence simultaneously, allowing comparison
 * of left and right wheel behavior under identical commands.
 */

static constexpr uint32_t kLoopPeriodMs{10};

WheelRotationSequenceRunner left_wheel_rotation_runner{left_encoder,
                                                       left_edge_encoder,
                                                       left_motor,
                                                       kLoopPeriodMs,
                                                       kLeftMotorPulsePerRevolution,
                                                       kFeedForwardRotationSequence};

WheelRotationSequenceRunner right_wheel_rotation_runner{right_encoder,
                                                        right_edge_encoder,
                                                        right_motor,
                                                        kLoopPeriodMs,
                                                        kRightMotorPulsePerRevolution,
                                                        kFeedForwardRotationSequence};

void setup() {
    Serial.begin(9600);

    // Wait for a serial terminal during development while allowing the system
    // to continue booting when running standalone from battery power.
    constexpr int kSerialWaitIterations{100};

    for (int count{}; !Serial && count < kSerialWaitIterations; count++) {
        delay(10);
    }

    Serial.println("Setup complete, starting wheel rotation sequence");

    right_wheel_rotation_runner.run();
    left_wheel_rotation_runner.run();
}

void loop() {
    left_wheel_rotation_runner.measureVelocity();
    right_wheel_rotation_runner.measureVelocity();
    const auto left_step_velocity = left_wheel_rotation_runner.getLastStepVelocity();
    const auto right_step_velocity = right_wheel_rotation_runner.getLastStepVelocity();
    if (left_step_velocity) {
        const auto [pwm_speed, velocity] = *left_step_velocity;
        Serial.printf("Left wheel step : { pwm: %d |  velocity: %.2f}", pwm_speed, velocity);
        left_wheel_rotation_runner.resetLastStepVelocity();
    }
    if (right_step_velocity) {
        const auto [pwm_speed, velocity] = *right_step_velocity;
        Serial.printf("Right wheel step : { pwm: %d |  velocity: %.2f}", pwm_speed, velocity);
        right_wheel_rotation_runner.resetLastStepVelocity();
    }
    if (left_step_velocity || right_step_velocity) {
        Serial.println();
    }
    delay(kLoopPeriodMs);
}