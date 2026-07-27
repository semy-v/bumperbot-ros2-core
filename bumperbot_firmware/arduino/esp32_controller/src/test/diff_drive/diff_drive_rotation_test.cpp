#include <Arduino.h>
#include <array>

#include "bl2418_encoder.hpp"
#include "bl2418_motor.hpp"
#include "diff_drive_constants.hpp"
#include "encoder_pcnt_test_helper.hpp"
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
 * Encoder instances for each wheel.
 *
 * The PCNT high/low limits are configured to the measured FG pulse count for
 * one complete wheel revolution. Positive limits generate events for forward
 * rotation, while negative limits generate events for reverse rotation.
 *
 * The pulse-per-revolution values may differ between motors due to mechanical
 * tolerances and are calibrated independently.
 */
BL2418Encoder left_encoder{kLeftMotorDirectionPin, kLeftMotorSpeedStatePin,
                           kLeftMotorPulsePerRevolution, -kLeftMotorPulsePerRevolution};

BL2418Encoder right_encoder{kRightMotorDirectionPin, kRightMotorSpeedStatePin,
                            kRightMotorPulsePerRevolution, -kRightMotorPulsePerRevolution};

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
constexpr std::array kWheelRotationSequence{
    WheelRotationStep{50, 3},  WheelRotationStep{100, 4},  WheelRotationStep{255, 5},

    WheelRotationStep{50, 3},

    WheelRotationStep{-50, 3}, WheelRotationStep{-100, 4}, WheelRotationStep{-255, 5},

    WheelRotationStep{-50, 3},

    WheelRotationStep{50, 3}};

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
WheelRotationSequenceRunner left_wheel_rotation_runner{left_encoder, left_motor,
                                                       kWheelRotationSequence};

WheelRotationSequenceRunner right_wheel_rotation_runner{right_encoder, right_motor,
                                                        kWheelRotationSequence};

void setup() {
    Serial.begin(9600);

    // Wait for a serial terminal during development while allowing the system
    // to continue booting when running standalone from battery power.
    constexpr int kSerialWaitIterations{100};

    for (int count{}; !Serial && count < kSerialWaitIterations; count++) {
        delay(10);
    }

    Serial.println("Initializing motors...");
    left_motor.begin();
    right_motor.begin();

    Serial.println("Initializing encoders...");
    left_encoder.begin();
    right_encoder.begin();

    /*
     * Start both wheel validation sequences.
     *
     * Encoder pulse counting is performed by the ESP32 PCNT hardware, so the
     * CPU is not required to process every FG pulse. The runners advance to
     * the next sequence step when the expected wheel revolution count is
     * reached.
     */
    Serial.println("Setup complete, starting wheel rotation sequence");

    right_wheel_rotation_runner.run();
    left_wheel_rotation_runner.run();
}

void loop() {
    // Wheel rotation progress is event-driven by PCNT encoder events.
    // No periodic motor control logic is required in this validation test.
    delay(10);
}