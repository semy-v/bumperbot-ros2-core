#ifndef FEEDFORWARD_CALIBRATION_RUNNER_HPP
#define FEEDFORWARD_CALIBRATION_RUNNER_HPP

#include <Arduino.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "diff_drive/bl2418_encoder.hpp"
#include "diff_drive/bl2418_motor.hpp"
#include "diff_drive/wheel_velocity_estimator.hpp"

/**
 * @brief Directional steady-state feed-forward calibration for one wheel.
 *
 * Production model:
 *
 *   forward: |PWM| = Ks_forward + Kv * |velocity|
 *   reverse: |PWM| = Ks_reverse + Kv * |velocity|
 *
 * There is deliberately no startup/breakaway production parameter.  To make
 * low-PWM steady-state samples measurable even when a wheel cannot start from
 * rest at that PWM, the diagnostic uses a calibration-only preconditioning
 * pulse before each sample:
 *
 *   1. Apply sign(test_PWM) * max(|test_PWM|, precondition_pwm).
 *   2. Keep it for precondition_time_ms so the wheel is already rotating.
 *   3. Drop to the actual test PWM.
 *   4. Wait for steady state and capture the sustainable velocity.
 *
 * The preconditioning PWM is never returned as a calibration result and is not
 * used by the production WheelController.  This keeps the calibrated format
 * motor-independent while still allowing the same diagnostic to characterize
 * motors with different breakaway behavior.
 */
class FeedForwardCalibrationRunner {
 public:
    struct Config {
        int velocity_pwm_start{50};
        int velocity_pwm_end{200};
        int velocity_pwm_step{10};

        bool calibrate_forward{true};
        bool calibrate_reverse{true};

        // Diagnostic-only preconditioning.  This is NOT a production parameter.
        int precondition_pwm{130};
        uint32_t precondition_time_ms{400};

        uint32_t settle_time_ms{500};
        uint32_t steady_time_ms{400};
        uint32_t max_steady_wait_ms{5000};
        uint32_t brake_time_ms{250};
        uint32_t stop_time_ms{500};

        float velocity_stddev_limit{0.45F};
        float velocity_mean_deviation_limit{0.20F};

        uint32_t capture_time_ms{2000};
        size_t capture_sample_count{50};
        uint32_t max_capture_wait_ms{2500};

        // The target production floor is 2 rad/s.  Samples down to 1 rad/s are
        // retained so the fitted directional intercepts are constrained by real
        // low-speed data instead of being extrapolated only from high speed.
        float minimum_regression_velocity{1.0F};
        float max_regression_stddev{0.8F};

        size_t minimum_regression_points_per_direction{5};
    };

    struct Sample {
        int pwm{0};
        float velocity{0.0F};
        float stddev{0.0F};
    };

    struct CalibrationResult {
        float ks_forward{0.0F};
        float ks_reverse{0.0F};
        float kv{0.0F};

        float r_squared{0.0F};
        float rmse_pwm{0.0F};

        size_t forward_sample_count{0};
        size_t reverse_sample_count{0};
    };

    static constexpr size_t kMaxSamples{32};

    FeedForwardCalibrationRunner(const char* runner_name,
                                 BL2418Motor& motor,
                                 BL2418Encoder& encoder,
                                 uint32_t update_period_ms,
                                 float ticks_per_rev,
                                 Print& logger,
                                 const Config& config);

    void begin();
    void stop();

    [[nodiscard]] bool startCalibration();
    void update(uint32_t dt_ms);

    // Called after both wheel runners have completed the same sample and the
    // common motor-supply reset has completed.
    void advanceSynchronizedStep();

    [[nodiscard]] bool waitingForPeer() const;
    [[nodiscard]] bool finished() const;
    [[nodiscard]] bool failed() const;
    [[nodiscard]] std::optional<CalibrationResult> result() const { return result_; }

 private:
    enum class State : uint8_t {
        Idle,
        ApplyPrecondition,
        Precondition,
        ApplyTestPWM,
        Settling,
        WaitForSteadyState,
        CaptureSample,
        Brake,
        Stop,
        WaitForPeer,
        Finished,
        Failed,
    };

    struct DirectionAccumulator {
        size_t count{0};
        double sum_x{0.0};
        double sum_y{0.0};
    };

    [[nodiscard]] bool configIsValid() const;

    void enterState(State next);
    void updateApplyPrecondition();
    void updatePrecondition();
    void updateApplyTestPWM();
    void updateSettling();
    void updateSteadyState();
    void updateCaptureSample();
    void updateBrake();
    void updateStop();

    [[nodiscard]] bool steadyStateReached();
    [[nodiscard]] bool selectNextSweepPoint();
    [[nodiscard]] int signedPreconditionPwm() const;

    float measureVelocity();
    void resetVelocityWindow();
    void computeWindowStatistics();

    void resetCaptureAccumulator();
    void accumulateCaptureSample(float velocity);
    [[nodiscard]] bool captureRequirementsSatisfied() const;
    void finishSampleCapture();

    [[nodiscard]] const char* sampleRejectionReason(const Sample& sample) const;
    [[nodiscard]] bool sampleIsUsable(const Sample& sample) const;

    void resetRegression();
    void computeRegression();
    void printResult() const;
    void printPrefix() const;

    const char* runner_name_;
    BL2418Motor& motor_;
    BL2418Encoder& encoder_;
    WheelVelocityEstimator estimator_;
    Print& logger_;
    Config config_;

    State state_{State::Idle};
    uint32_t state_time_ms_{0};
    int current_pwm_{0};

    static constexpr size_t kVelocityWindowSize{25};
    std::array<float, kVelocityWindowSize> velocity_window_{};
    size_t velocity_window_count_{0};
    size_t velocity_window_index_{0};
    float velocity_mean_{0.0F};
    float velocity_stddev_{0.0F};
    float newest_velocity_{0.0F};
    float newest_mean_deviation_{0.0F};

    size_t capture_sample_count_{0};
    double capture_velocity_sum_{0.0};
    double capture_velocity_squared_sum_{0.0};

    std::array<Sample, kMaxSamples> samples_{};
    size_t sample_count_{0};
    std::optional<CalibrationResult> result_{};
};

#endif  // FEEDFORWARD_CALIBRATION_RUNNER_HPP