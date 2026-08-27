#include <Arduino.h>

#include "diff_drive/bldc2430_encoder.hpp"
#include "diff_drive/bldc2430_motor.hpp"
#include "diff_drive/diff_drive_constants.hpp"
#include "feedforward_calibration_runner.hpp"
#include "wireless_console.hpp"

namespace {

constexpr uint32_t kSerialBaudRate{115200U};
constexpr uint32_t kCalibrationUpdateRateHz{100};
constexpr uint32_t kCalibrationUpdatePeriodMs{1000U / kCalibrationUpdateRateHz};

static_assert(1000U % kCalibrationUpdateRateHz == 0U,
              "Calibration update rate must produce an integral millisecond period.");

constexpr uint32_t kMotorPowerOffResetMs{300};
constexpr uint32_t kMotorPowerOnSettleMs{150};

// -----------------------------------------------------------------------------
// Directional moving-state feed-forward calibration
// -----------------------------------------------------------------------------

constexpr FeedForwardCalibrationRunner::Config kCalibrationConfig{
    // Start below the expected ~2 rad/s operating point so the calibration can
    // capture (or explicitly reject) the direct-from-rest low-speed region.
    // 40..200 by 10 produces 17 magnitudes / 34 signed points.
    .velocity_pwm_start = 40,
    .velocity_pwm_end = 200,
    .velocity_pwm_step = 10,

    .calibrate_forward = true,
    .calibrate_reverse = true,

    // No high-PWM preconditioning is used with the BL2430.  The actual test PWM
    // is applied directly from rest.  This makes the diagnostic consistent with
    // the production model, which intentionally has no startup/breakaway term.
    .settle_time_ms = 1000,

    // Capture the full response and judge stability from this entire interval
    // rather than waiting for a short window that happens to look quiet.
    .capture_time_ms = 2000,
    .capture_sample_count = 100,
    .max_capture_wait_ms = 2500,

    .brake_time_ms = 250,
    .stop_time_ms = 500,

    // Relative limits scale with wheel speed and therefore do not become
    // artificially stricter at high PWM as the previous absolute sigma/deviation
    // thresholds did.
    .max_capture_relative_stddev = 0.10F,
    .max_capture_relative_mean_drift = 0.05F,

    // The target production floor is ~2 rad/s.  Keeping points >=1 rad/s allows
    // the directional Ks intercepts to be constrained by real low-speed data.
    .minimum_regression_velocity = 1.0F,
    .minimum_regression_points_per_direction = 5,
};

#if __has_include("wifi_credentials.hpp")
#include "wifi_credentials.hpp"
constexpr const char* configured_ssid = wifi_credentials::kSsid;
constexpr const char* configured_password = wifi_credentials::kPassword;
#else
#pragma message("wifi_credentials.hpp not found. Using empty/fallback credentials.")
constexpr const char* configured_ssid = "";
constexpr const char* configured_password = "";
#endif

WirelessConsole console{WirelessConsole::Config{
    .ssid = configured_ssid,
    .password = configured_password,
    .port = 23,
    .log_buffer_size = 8192,
    .task_priority = 1,
    .cpu_core = 0,
}};

// -----------------------------------------------------------------------------
// Shared motor-supply power cycler
// -----------------------------------------------------------------------------

class MotorPowerCycler {
 public:
    void begin() {
        pinMode(kMotorsPowerEnablePin, OUTPUT);
        digitalWrite(kMotorsPowerEnablePin, HIGH);
        state_ = State::Idle;
        completion_pending_ = false;
    }

    [[nodiscard]] bool canRequest() const { return state_ == State::Idle && !completion_pending_; }

    void request() {
        if (!canRequest()) {
            return;
        }

        digitalWrite(kMotorsPowerEnablePin, LOW);
        state_ = State::PowerOffWait;
        state_start_ms_ = millis();
        console.println("[FF:POWER] motor supply OFF for driver reset");
    }

    void update() {
        const uint32_t now_ms = millis();

        switch (state_) {
            case State::Idle:
                break;

            case State::PowerOffWait:
                if (now_ms - state_start_ms_ >= kMotorPowerOffResetMs) {
                    digitalWrite(kMotorsPowerEnablePin, HIGH);
                    state_ = State::PowerOnWait;
                    state_start_ms_ = now_ms;
                    console.println("[FF:POWER] motor supply ON; waiting for driver settle");
                }
                break;

            case State::PowerOnWait:
                if (now_ms - state_start_ms_ >= kMotorPowerOnSettleMs) {
                    state_ = State::Idle;
                    completion_pending_ = true;
                    console.println("[FF:POWER] motor power cycle complete");
                }
                break;
        }
    }

    [[nodiscard]] bool consumeCompletion() {
        if (!completion_pending_) {
            return false;
        }

        completion_pending_ = false;
        return true;
    }

    void cancelAndEnable() {
        digitalWrite(kMotorsPowerEnablePin, HIGH);
        state_ = State::Idle;
        completion_pending_ = false;
    }

 private:
    enum class State : uint8_t {
        Idle,
        PowerOffWait,
        PowerOnWait,
    };

    State state_{State::Idle};
    uint32_t state_start_ms_{0};
    bool completion_pending_{false};
};

// -----------------------------------------------------------------------------
// Wheel hardware
// -----------------------------------------------------------------------------

BLDC2430Encoder left_encoder{
    kLeftMotorDirectionPin,
    kLeftMotorSpeedStatePin,
    false,
};

BLDC2430Encoder right_encoder{
    kRightMotorDirectionPin,
    kRightMotorSpeedStatePin,
    true,
};

BLDC2430Motor left_motor{
    kLeftMotorDirectionPin,
    kLeftMotorSpeedCommandPin,
    false,
};

BLDC2430Motor right_motor{
    kRightMotorDirectionPin,
    kRightMotorSpeedCommandPin,
    true,
};

MotorPowerCycler motor_power_cycler;

FeedForwardCalibrationRunner left_runner{
    "LEFT",
    left_motor,
    left_encoder,
    kCalibrationUpdatePeriodMs,
    static_cast<float>(kLeftMotorPulsePerRevolution),
    console,
    kCalibrationConfig,
};

FeedForwardCalibrationRunner right_runner{
    "RIGHT",
    right_motor,
    right_encoder,
    kCalibrationUpdatePeriodMs,
    static_cast<float>(kRightMotorPulsePerRevolution),
    console,
    kCalibrationConfig,
};

// -----------------------------------------------------------------------------
// Application state
// -----------------------------------------------------------------------------

enum class CalibrationStage : uint8_t {
    Idle,
    InitialMotorReset,
    Running,
};

CalibrationStage calibration_stage{CalibrationStage::Idle};
bool calibration_started{false};
uint32_t last_update_ms{0};

void printTestInstructions() {
    console.println();
    console.println("========================================================");
    console.println("BL2430 directional feed-forward calibration");
    console.println("========================================================");
    console.println("Production model per wheel:");
    console.println("  forward: +[Ks_forward + Kv*|v|]");
    console.println("  reverse: -[Ks_reverse + Kv*|v|]");
    console.println("  No startup/breakaway PWM and no calibration preconditioning.");
    console.println();
    console.println("Calibration behavior:");
    console.println("  - both wheels are sampled simultaneously");
    console.println("  - +PWM then -PWM at every magnitude");
    console.println("  - every test PWM is applied directly from rest");
    console.println("  - fixed settling time, then a full 2-second capture");
    console.println("  - capture acceptance uses relative sigma and first/second-half mean drift");
    console.println("  - there is no short-window steady-state wait/timeout gate");
    console.println("  - motor power is cycled between every signed sample");
    console.println();
    console.printf("Configured falling-edge ticks/rev: LEFT=%d RIGHT=%d\n",
                   static_cast<int>(kLeftMotorPulsePerRevolution),
                   static_cast<int>(kRightMotorPulsePerRevolution));
    console.println("These values MUST be falling-edge counts, not old rising+falling counts.");
    console.println();
    console.println("Before starting:");
    console.println("  1. Put the assembled robot on its normal floor for production calibration.");
    console.println("  2. Keep a clear travel area in both directions.");
    console.println("  3. Verify falling-edge pulses/revolution for both installed BL2430 motors.");
    console.println();
    console.println("Commands:");
    console.println("  s - start calibration");
    console.println("  x - stop immediately");
    console.println("  h - print this help");
    console.println("========================================================");
}

void abortCalibration(const char* reason) {
    left_runner.stop();
    right_runner.stop();
    left_motor.setPwmSpeed(0);
    right_motor.setPwmSpeed(0);
    motor_power_cycler.cancelAndEnable();

    calibration_started = false;
    calibration_stage = CalibrationStage::Idle;

    console.println();
    console.printf("[FF] Calibration aborted: %s\n", reason);
    console.println("[FF] Both motor PWM commands are zero.");
}

void printFinalResults() {
    const auto left = left_runner.result();
    const auto right = right_runner.result();

    console.println();
    console.println("========================================================");
    console.println("Final directional feed-forward calibration results");
    console.println("========================================================");

    if (left) {
        console.printf("LEFT : Ks_fwd=%.4f, Ks_rev=%.4f, Kv=%.4f, R^2=%.5f, RMSE=%.4f PWM\n",
                       left->ks_forward, left->ks_reverse, left->kv, left->r_squared,
                       left->rmse_pwm);
        console.printf("       predicted |PWM| at 2 rad/s: fwd=%.2f rev=%.2f\n",
                       left->ks_forward + 2.0F * left->kv, left->ks_reverse + 2.0F * left->kv);
    } else {
        console.println("LEFT : FAILED");
    }

    if (right) {
        console.printf("RIGHT: Ks_fwd=%.4f, Ks_rev=%.4f, Kv=%.4f, R^2=%.5f, RMSE=%.4f PWM\n",
                       right->ks_forward, right->ks_reverse, right->kv, right->r_squared,
                       right->rmse_pwm);
        console.printf("       predicted |PWM| at 2 rad/s: fwd=%.2f rev=%.2f\n",
                       right->ks_forward + 2.0F * right->kv, right->ks_reverse + 2.0F * right->kv);
    } else {
        console.println("RIGHT: FAILED");
    }

    if (left && right) {
        console.println();
        console.println("Suggested production feed-forward parameters:");
        console.printf("  left_feedforward_ks_forward: %.4f\n", left->ks_forward);
        console.printf("  left_feedforward_ks_reverse: %.4f\n", left->ks_reverse);
        console.printf("  left_feedforward_kv: %.4f\n", left->kv);
        console.printf("  right_feedforward_ks_forward: %.4f\n", right->ks_forward);
        console.printf("  right_feedforward_ks_reverse: %.4f\n", right->ks_reverse);
        console.printf("  right_feedforward_kv: %.4f\n", right->kv);
    }

    console.println("========================================================");
}

void startCalibration() {
    if (calibration_started) {
        console.println("Calibration is already running.");
        return;
    }

    left_runner.stop();
    right_runner.stop();
    left_motor.setPwmSpeed(0);
    right_motor.setPwmSpeed(0);
    motor_power_cycler.cancelAndEnable();

    calibration_started = true;
    calibration_stage = CalibrationStage::InitialMotorReset;
    last_update_ms = millis();

    console.println();
    console.println("[FF] Starting calibration with initial motor-driver reset...");
    motor_power_cycler.request();
}

void processCommand() {
    char command{};
    while (console.readCommand(command)) {
        switch (command) {
            case 's':
            case 'S':
                startCalibration();
                break;

            case 'x':
            case 'X':
                if (calibration_started) {
                    abortCalibration("stopped by user");
                }
                break;

            case 'h':
            case 'H':
                printTestInstructions();
                break;

            default:
                break;
        }
    }
}

void updateInitialMotorReset() {
    if (!motor_power_cycler.consumeCompletion()) {
        return;
    }

    const bool left_started = left_runner.startCalibration();
    const bool right_started = right_runner.startCalibration();

    if (!left_started || !right_started) {
        abortCalibration("failed to start directional calibration");
        return;
    }

    calibration_stage = CalibrationStage::Running;
}

void serviceSamplePowerCycle() {
    if (!left_runner.waitingForPeer() || !right_runner.waitingForPeer()) {
        return;
    }

    // Consume completion before requesting another cycle so a just-completed
    // power-cycle event cannot be overwritten.
    if (motor_power_cycler.consumeCompletion()) {
        console.println();
        console.println("[FF:SYNC] motor reset complete; advancing both runners.");
        left_runner.advanceSynchronizedStep();
        right_runner.advanceSynchronizedStep();
        return;
    }

    if (motor_power_cycler.canRequest()) {
        console.println();
        console.println(
            "[FF:SYNC] both samples complete; power-cycling drivers before next point.");
        motor_power_cycler.request();
    }
}

void updateRunningStage() {
    left_runner.update(kCalibrationUpdatePeriodMs);
    right_runner.update(kCalibrationUpdatePeriodMs);

    if (left_runner.failed() || right_runner.failed()) {
        abortCalibration("directional regression failed");
        return;
    }

    serviceSamplePowerCycle();

    if (left_runner.failed() || right_runner.failed()) {
        abortCalibration("directional regression failed");
        return;
    }

    if (!left_runner.finished() || !right_runner.finished()) {
        return;
    }

    calibration_started = false;
    calibration_stage = CalibrationStage::Idle;
    left_motor.setPwmSpeed(0);
    right_motor.setPwmSpeed(0);

    console.println();
    console.println("[FF] Calibration finished successfully.");
    printFinalResults();
}

void updateCalibration() {
    if (!calibration_started) {
        return;
    }

    const uint32_t now_ms = millis();
    if (now_ms - last_update_ms < kCalibrationUpdatePeriodMs) {
        return;
    }

    // Keep the calibration state machine on the same nominal 100 Hz cadence as
    // the production velocity estimator/controller.
    last_update_ms += kCalibrationUpdatePeriodMs;

    switch (calibration_stage) {
        case CalibrationStage::Idle:
            break;

        case CalibrationStage::InitialMotorReset:
            updateInitialMotorReset();
            break;

        case CalibrationStage::Running:
            updateRunningStage();
            break;
    }
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);

    if (!console.begin()) {
        Serial.println("[CRITICAL] Failed to initialize wireless console.");
        while (true) {
            delay(1000);
        }
    }

    console.println();
    console.println("Initializing BL2430 directional feed-forward calibration hardware...");

    motor_power_cycler.begin();
    left_runner.begin();
    right_runner.begin();

    console.println("Hardware initialization complete.");
    printTestInstructions();
}

void loop() {
    processCommand();
    motor_power_cycler.update();
    updateCalibration();
    delay(1);
}
