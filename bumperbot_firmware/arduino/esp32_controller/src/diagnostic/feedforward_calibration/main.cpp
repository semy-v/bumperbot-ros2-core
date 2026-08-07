#include <Arduino.h>

#include "diff_drive/bl2418_encoder.hpp"
#include "diff_drive/bl2418_motor.hpp"
#include "diff_drive/diff_drive_constants.hpp"
#include "feedforward_calibration_runner.hpp"

namespace {

constexpr uint32_t kSerialBaudRate{115200U};

// -----------------------------------------------------------------------------
// Calibration update rate
// -----------------------------------------------------------------------------

constexpr uint32_t kCalibrationUpdateRateHz{100};
constexpr uint32_t kCalibrationUpdatePeriodMs{
    1000U / kCalibrationUpdateRateHz
};

static_assert(
    1000U % kCalibrationUpdateRateHz == 0U,
    "Calibration update rate must produce an integral millisecond period.");

// -----------------------------------------------------------------------------
// Calibration configuration
// -----------------------------------------------------------------------------

constexpr FeedForwardCalibrationRunner::Config kCalibrationConfig{
    // PWM sweep.
    .pwm_start = 30,
    .pwm_end = 250,
    .pwm_step = 20,

    // Directions.
    .calibrate_forward = true,
    .calibrate_reverse = true,

    // State timing.
    .settle_time_ms = 400,
    .steady_time_ms = 300,
    .max_steady_wait_ms = 4000,
    .brake_time_ms = 250,
    .stop_time_ms = 500,

    // Steady-state detector.
    .velocity_stddev_limit = 0.40F,
    .velocity_mean_deviation_limit = 0.15F,

    // Sample capture.
    .capture_time_ms = 2000,
    .capture_sample_count = 50,
    .max_capture_wait_ms = 2000,

    // Regression.
    .minimum_regression_velocity = 1.0F,
    .static_friction_pwm = 25.0F,
    .use_fixed_static_friction = false
};

// -----------------------------------------------------------------------------
// Wheel hardware
// -----------------------------------------------------------------------------

/*
 * Only one encoder instance per wheel is needed by the calibration runner.
 *
 * Each encoder uses PCNT high/low limits of +1/-1 internally so every FG edge
 * updates the reciprocal-period velocity measurement.
 */
BL2418Encoder left_encoder{
    kLeftMotorDirectionPin,
    kLeftMotorSpeedStatePin,
    false  // Left wheel logic is not inverted.
};

BL2418Encoder right_encoder{
    kRightMotorDirectionPin,
    kRightMotorSpeedStatePin,
    true  // Right wheel is mounted as a mirror image.
};

BL2418Motor left_motor{
    kLeftMotorDirectionPin,
    kLeftMotorSpeedCommandPin,
    false
};

BL2418Motor right_motor{
    kRightMotorDirectionPin,
    kRightMotorSpeedCommandPin,
    true
};

// -----------------------------------------------------------------------------
// Independent wheel calibration runners
// -----------------------------------------------------------------------------

FeedForwardCalibrationRunner left_calibration_runner{
    "LEFT",
    left_motor,
    left_encoder,
    kCalibrationUpdatePeriodMs,
    static_cast<float>(kLeftMotorPulsePerRevolution),
    kCalibrationConfig
};

FeedForwardCalibrationRunner right_calibration_runner{
    "RIGHT",
    right_motor,
    right_encoder,
    kCalibrationUpdatePeriodMs,
    static_cast<float>(kRightMotorPulsePerRevolution),
    kCalibrationConfig
};

// -----------------------------------------------------------------------------
// Test state
// -----------------------------------------------------------------------------

bool calibration_started{false};
bool results_printed{false};

uint32_t last_update_ms{0};

// -----------------------------------------------------------------------------
// Serial helpers
// -----------------------------------------------------------------------------

void printTestInstructions() {
    Serial.println();
    Serial.println("========================================================");
    Serial.println("Differential-drive feed-forward calibration");
    Serial.println("========================================================");
    Serial.println("The robot will move under open-loop PWM control.");
    Serial.println("Both wheels are calibrated simultaneously.");
    Serial.println();
    Serial.println("Before starting:");
    Serial.println("  1. Place the assembled robot on its normal floor.");
    Serial.println("  2. Make sure the travel path is clear.");
    Serial.println("  3. Be ready to disconnect motor power.");
    Serial.println();
    Serial.println("Serial commands:");
    Serial.println("  s - start calibration");
    Serial.println("  x - stop calibration immediately");
    Serial.println("========================================================");
}

void printWheelResult(
    const char* wheel_name,
    const std::optional<FeedForwardCalibrationRunner::CalibrationResult>& result) {
    Serial.println();
    Serial.printf("%s wheel result:\n", wheel_name);

    if (!result) {
        Serial.println("  Calibration failed or produced insufficient samples.");
        return;
    }

    Serial.printf(
        "  Ks             : %.4f PWM\n",
        result->ks);

    Serial.printf(
        "  Kv             : %.4f PWM/(rad/s)\n",
        result->kv);

    Serial.printf(
        "  R^2            : %.5f\n",
        result->r_squared);

    Serial.printf(
        "  RMSE           : %.4f PWM\n",
        result->rmse_pwm);

    Serial.printf(
        "  Samples used   : %u\n",
        static_cast<unsigned>(result->sample_count));

    Serial.println();
    Serial.println("  Feed-forward equation:");

    Serial.printf(
        "    pwm_ff = %.4f * sign(target_velocity)"
        " + %.4f * target_velocity\n",
        result->ks,
        result->kv);
}

void printCombinedResults() {
    Serial.println();
    Serial.println("========================================================");
    Serial.println("Combined feed-forward calibration results");
    Serial.println("========================================================");

    printWheelResult(
        "Left",
        left_calibration_runner.result());

    printWheelResult(
        "Right",
        right_calibration_runner.result());

    Serial.println();
    Serial.println("Suggested wheel configuration values:");

    const auto left_result = left_calibration_runner.result();
    if (left_result) {
        Serial.printf(
            "  left_feedforward_ks: %.4f\n",
            left_result->ks);
        Serial.printf(
            "  left_feedforward_kv: %.4f\n",
            left_result->kv);
    }

    const auto right_result = right_calibration_runner.result();
    if (right_result) {
        Serial.printf(
            "  right_feedforward_ks: %.4f\n",
            right_result->ks);
        Serial.printf(
            "  right_feedforward_kv: %.4f\n",
            right_result->kv);
    }

    Serial.println("========================================================");
}

// -----------------------------------------------------------------------------
// Calibration control
// -----------------------------------------------------------------------------

void startCalibration() {
    if (calibration_started) {
        Serial.println("Calibration is already running.");
        return;
    }

    results_printed = false;
    calibration_started = true;

    // Initialize the fixed-rate update schedule immediately before starting.
    last_update_ms = millis();

    Serial.println();
    Serial.println("Starting left and right feed-forward calibration...");

    // Both runners must begin on the same control iteration so that the robot
    // receives matching left/right PWM commands and travels approximately
    // straight during each calibration point.
    left_calibration_runner.start();
    right_calibration_runner.start();
}

void stopCalibration() {
    left_calibration_runner.stop();
    right_calibration_runner.stop();

    calibration_started = false;

    Serial.println();
    Serial.println("Calibration stopped. Both motor commands are zero.");
}

void processSerialCommand() {
    while (Serial.available() > 0) {
        const char command =
            static_cast<char>(Serial.read());

        switch (command) {
            case 's':
            case 'S':
                startCalibration();
                break;

            case 'x':
            case 'X':
                stopCalibration();
                break;

            case '\r':
            case '\n':
            case ' ':
                break;

            default:
                Serial.printf(
                    "Unknown command '%c'. Use 's' to start or 'x' to stop.\n",
                    command);
                break;
        }
    }
}

void updateCalibration() {
    if (!calibration_started) {
        return;
    }

    const uint32_t now_ms = millis();

    if (now_ms - last_update_ms <
        kCalibrationUpdatePeriodMs) {
        return;
    }

    last_update_ms += kCalibrationUpdatePeriodMs;

    left_calibration_runner.update(
        kCalibrationUpdatePeriodMs);

    right_calibration_runner.update(
        kCalibrationUpdatePeriodMs);

    const bool left_waiting =
        left_calibration_runner.waitingForPeer();

    const bool right_waiting =
        right_calibration_runner.waitingForPeer();

    /*
     * Neither wheel advances until both have:
     *
     * 1. reached steady state or timed out,
     * 2. captured its sample,
     * 3. completed braking,
     * 4. completed its stationary pause.
     */
    if (left_waiting && right_waiting) {
        Serial.println();
        Serial.println(
            "[FF:SYNC] Both wheels ready; advancing calibration point.");

        left_calibration_runner.advanceSynchronizedStep();
        right_calibration_runner.advanceSynchronizedStep();
    }

    const bool left_finished =
        left_calibration_runner.finished();

    const bool right_finished =
        right_calibration_runner.finished();

    /*
     * With synchronized advancement and identical sweep configuration, both
     * runners finish during the same coordinator iteration.
     */
    if (left_finished && right_finished) {
        calibration_started = false;

        Serial.println();
        Serial.println(
            "[FF:SYNC] Both wheel calibrations finished.");

        printCombinedResults();
    }
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);

    // Wait briefly for a development terminal, but continue booting when the
    // robot operates without a connected host.
    constexpr uint32_t kSerialWaitTimeoutMs{2000};
    const uint32_t serial_wait_start_ms = millis();

    while (!Serial &&
           millis() - serial_wait_start_ms < kSerialWaitTimeoutMs) {
        delay(10);
    }

    Serial.println();
    Serial.println("Initializing feed-forward calibration hardware...");

    // Enable power for both motors before starting the rotation sequence.
    pinMode(kMotorsPowerEnablePin, OUTPUT);
    digitalWrite(kMotorsPowerEnablePin, HIGH);

    left_calibration_runner.begin();
    right_calibration_runner.begin();

    Serial.println("Hardware initialization complete.");

    printTestInstructions();
}

void loop() {
    processSerialCommand();
    updateCalibration();

    // Yield CPU time while preserving the 100 Hz calibration schedule.
    delay(1);
}