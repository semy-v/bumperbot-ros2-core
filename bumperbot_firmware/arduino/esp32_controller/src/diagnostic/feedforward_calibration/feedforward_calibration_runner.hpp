#ifndef FEEDFORWARD_CALIBRATION_RUNNER_HPP
#define FEEDFORWARD_CALIBRATION_RUNNER_HPP

#include <Arduino.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "diff_drive/bldc2430_encoder.hpp"
#include "diff_drive/bldc2430_motor.hpp"
#include "diff_drive/wheel_velocity_estimator.hpp"

/**
 * @brief Directional moving-state feed-forward calibration for one wheel.
 *
 * Production model:
 *
 *   forward: |PWM| = Ks_forward + Kv * |velocity|
 *   reverse: |PWM| = Ks_reverse + Kv * |velocity|
 *
 * The diagnostic deliberately does not use a startup/breakaway PWM and does
 * not precondition a wheel with a higher PWM command.  Every calibration point
 * is therefore applied directly from the stopped state, matching the intended
 * production behavior of a controller that contains only direction-specific
 * Ks and one shared Kv.
 *
 * Per signed PWM point the runner performs:
 *
 *   1. Apply the actual test PWM directly from rest.
 *   2. Wait a fixed settling interval while keeping the production velocity
 *      estimator updated.
 *   3. Capture velocity for a fixed observation interval.
 *   4. Validate the whole capture using:
 *        - minimum mean speed,
 *        - direction agreement,
 *        - relative standard deviation sigma / |mean|,
 *        - relative drift between the first and second capture halves.
 *   5. Store only accepted points for the directional regression.
 *
 * This intentionally avoids the previous "wait until one short window happens
 * to look quiet" behavior.  Periodic motor/gearbox ripple or residual FG timing
 * jitter is allowed as long as the full capture has bounded relative variation
 * and its average speed is no longer drifting materially.
 *
 * @note ticks_per_rev must match the production reciprocal-period encoder
 *       events.  With the revised BL2430 encoder configuration that means the
 *       empirically measured number of *falling FG edges* per output-wheel
 *       revolution, not the previous rising+falling transition count.
 */
class FeedForwardCalibrationRunner {
 public:
    struct Config {
        int velocity_pwm_start{40};
        int velocity_pwm_end{200};
        int velocity_pwm_step{10};

        // Direction-specific Ks requires measurements in both directions.
        bool calibrate_forward{true};
        bool calibrate_reverse{true};

        // Time at the actual test PWM before capture starts.  No higher-PWM
        // preconditioning command is applied.
        uint32_t settle_time_ms{1000};

        uint32_t capture_time_ms{2000};

        // Minimum number of finite 100 Hz velocity samples required.  The
        // duration is still the primary capture criterion; this protects
        // against unexpectedly missing/non-finite measurements.
        size_t capture_sample_count{100};
        uint32_t max_capture_wait_ms{2500};

        uint32_t brake_time_ms{250};
        uint32_t stop_time_ms{500};

        // A candidate is accepted only if the full-capture standard deviation
        // is no more than this fraction of |mean velocity|.
        float max_capture_relative_stddev{0.10F};

        // Compare the mean velocity of the first and second halves of the
        // capture.  This rejects a wheel that is still accelerating/decelerating
        // while tolerating periodic ripple about a stable mean.
        float max_capture_relative_mean_drift{0.05F};

        // Keep useful low-speed data around the desired ~2 rad/s production
        // operating floor while rejecting stopped/stalled samples.
        float minimum_regression_velocity{1.0F};

        size_t minimum_regression_points_per_direction{5};
    };

    struct Sample {
        int pwm{0};
        float velocity{0.0F};
        float stddev{0.0F};
        float relative_stddev{0.0F};
        float first_half_mean{0.0F};
        float second_half_mean{0.0F};
        float relative_mean_drift{0.0F};
        size_t capture_count{0};
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

    // 40..200 by 10 gives 17 magnitudes / 34 signed samples.  Keep a little
    // spare capacity for future sweep changes without dynamic allocation.
    static constexpr size_t kMaxSamples{40};

    FeedForwardCalibrationRunner(const char* runner_name,
                                 BLDC2430Motor& motor,
                                 BLDC2430Encoder& encoder,
                                 uint32_t update_period_ms,
                                 float ticks_per_rev,
                                 Print& logger,
                                 const Config& config);

    void begin();
    void stop();

    [[nodiscard]] bool startCalibration();
    void update(uint32_t dt_ms);

    // Called after both wheel runners have completed the same signed PWM point
    // and the shared motor-supply power cycle has completed.
    void advanceSynchronizedStep();

    [[nodiscard]] bool waitingForPeer() const;
    [[nodiscard]] bool finished() const;
    [[nodiscard]] bool failed() const;
    [[nodiscard]] std::optional<CalibrationResult> result() const { return result_; }

 private:
    enum class State : uint8_t {
        Idle,
        ApplyTestPWM,
        Settling,
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
    void updateApplyTestPWM();
    void updateSettling();
    void updateCaptureSample();
    void updateBrake();
    void updateStop();

    [[nodiscard]] bool selectNextSweepPoint();

    float measureVelocity();

    void resetCaptureAccumulator();
    void accumulateCaptureSample(float velocity, uint32_t capture_elapsed_ms);
    [[nodiscard]] bool captureRequirementsSatisfied() const;
    void finishSampleCapture();

    [[nodiscard]] const char* sampleRejectionReason(const Sample& sample) const;
    [[nodiscard]] bool sampleIsUsable(const Sample& sample) const;

    void resetRegression();
    void computeRegression();
    void printResult() const;
    void printPrefix() const;

    const char* runner_name_;
    BLDC2430Motor& motor_;
    BLDC2430Encoder& encoder_;
    WheelVelocityEstimator estimator_;
    Print& logger_;
    Config config_;

    State state_{State::Idle};
    uint32_t state_time_ms_{0};
    int current_pwm_{0};

    size_t capture_sample_count_{0};
    double capture_velocity_sum_{0.0};
    double capture_velocity_squared_sum_{0.0};

    size_t capture_first_half_count_{0};
    double capture_first_half_sum_{0.0};

    size_t capture_second_half_count_{0};
    double capture_second_half_sum_{0.0};

    float capture_min_velocity_{0.0F};
    float capture_max_velocity_{0.0F};

    std::array<Sample, kMaxSamples> samples_{};
    size_t sample_count_{0};
    std::optional<CalibrationResult> result_{};
};

#endif  // FEEDFORWARD_CALIBRATION_RUNNER_HPP
