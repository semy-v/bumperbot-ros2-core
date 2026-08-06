#ifndef FEEDFORWARD_CALIBRATION_RUNNER_HPP
#define FEEDFORWARD_CALIBRATION_RUNNER_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "diff_drive/bl2418_encoder.hpp"
#include "diff_drive/bl2418_motor.hpp"
#include "diff_drive/wheel_velocity_estimator.hpp"

class FeedForwardCalibrationRunner {
 public:
    struct Config {
        //----------------------------------------
        // PWM sweep
        //----------------------------------------

        int pwm_start{30};
        int pwm_end{250};
        int pwm_step{20};

        //----------------------------------------
        // Directions
        //----------------------------------------

        bool calibrate_forward{true};
        bool calibrate_reverse{true};

        //----------------------------------------
        // State timing
        //----------------------------------------

        uint32_t settle_time_ms{400};
        uint32_t steady_time_ms{300};
        uint32_t max_steady_wait_ms{4000};

        uint32_t brake_time_ms{250};
        uint32_t stop_time_ms{500};

        //----------------------------------------
        // Steady-state detector
        //----------------------------------------

        // Maximum permitted standard deviation in the velocity window.
        float velocity_stddev_limit{0.25F};

        // Maximum difference between the newest sample and window mean.
        float velocity_mean_deviation_limit{0.15F};

        //----------------------------------------
        // Sample capture
        //----------------------------------------

        /*
         * Set either requirement to zero to disable it:
         *
         * capture_time_ms > 0, capture_sample_count == 0:
         *   Capture for a fixed duration.
         *
         * capture_time_ms == 0, capture_sample_count > 0:
         *   Capture a fixed number of samples.
         *
         * Both nonzero:
         *   Both requirements must be satisfied.
         */
        uint32_t capture_time_ms{1000};
        size_t capture_sample_count{25};

        // Prevent a missing encoder signal from blocking calibration forever.
        uint32_t max_capture_wait_ms{2000};

        //----------------------------------------
        // Regression
        //----------------------------------------

        float minimum_regression_velocity{1.0F};

        float static_friction_pwm{25.0F};
        bool use_fixed_static_friction{true};
    };

    struct Sample {
        int pwm{0};
        float velocity{0.0F};
        float stddev{0.0F};
    };

    static constexpr size_t kMaxSamples{32};

    struct CalibrationResult {
        float kv{0.0F};
        float ks{0.0F};

        float r_squared{0.0F};
        float rmse_pwm{0.0F};

        size_t sample_count{0};
    };

    enum class State : uint8_t {
        Idle,
        ApplyPWM,
        Settling,
        WaitForSteadyState,
        CaptureSample,
        Brake,
        Stop,
        WaitForPeer,
        Finished
    };

    FeedForwardCalibrationRunner(const char* runner_name,
                                 BL2418Motor& motor,
                                 BL2418Encoder& encoder,
                                 uint32_t update_period_ms,
                                 float ticks_per_rev);

    FeedForwardCalibrationRunner(const char* runner_name,
                                 BL2418Motor& motor,
                                 BL2418Encoder& encoder,
                                 uint32_t update_period_ms,
                                 float ticks_per_rev,
                                 const Config& config);

    void begin();
    void start();
    void stop();

    void update(uint32_t dt_ms);

    /**
     * @brief Advances the runner after every runner has reached WaitForPeer.
     *
     * This must be called by the external calibration coordinator only when
     * both wheel runners report waitingForPeer() == true.
     */
    void advanceSynchronizedStep();

    bool running() const { return running_; }

    bool finished() const { return state_ == State::Finished; }

    bool waitingForPeer() const { return state_ == State::WaitForPeer; }

    State state() const { return state_; }

    std::optional<CalibrationResult> result() const { return result_; }

 private:
    //----------------------------------------
    // State machine
    //----------------------------------------

    void enterState(State next);

    void updateApplyPWM();
    void updateSettling();
    void updateSteadyState();
    void updateCaptureSample();
    void updateBrake();
    void updateStop();

    //----------------------------------------
    // Sweep
    //----------------------------------------

    bool selectNextSweepPoint();

    //----------------------------------------
    // Velocity and steady-state detection
    //----------------------------------------

    float measureVelocity();
    bool steadyStateReached();
    void computeWindowStatistics();
    void resetVelocityWindow();

    //----------------------------------------
    // Sample capture
    //----------------------------------------

    void resetCaptureAccumulator();
    void accumulateCaptureSample(float velocity);
    bool captureRequirementsSatisfied() const;
    void finishSampleCapture();

    //----------------------------------------
    // Regression
    //----------------------------------------

    void resetRegression();
    void computeRegression();
    bool sampleIsUsable(const Sample& sample) const;

    //----------------------------------------
    // Reporting
    //----------------------------------------

    void printResult() const;
    void printPrefix() const;

    //----------------------------------------
    // Identity
    //----------------------------------------

    const char* runner_name_;

    //----------------------------------------
    // Hardware
    //----------------------------------------

    BL2418Motor& motor_;
    BL2418Encoder& encoder_;
    WheelVelocityEstimator estimator_;

    //----------------------------------------
    // Configuration
    //----------------------------------------

    Config config_;

    //----------------------------------------
    // Calibration samples
    //----------------------------------------

    std::array<Sample, kMaxSamples> samples_{};
    size_t sample_count_{0};

    //----------------------------------------
    // Steady-state velocity window
    //----------------------------------------

    static constexpr size_t kVelocityWindowSize{25};

    std::array<float, kVelocityWindowSize> velocity_window_{};

    size_t velocity_window_count_{0};
    size_t velocity_window_index_{0};

    float velocity_mean_{0.0F};
    float velocity_stddev_{0.0F};

    //----------------------------------------
    // Capture accumulator
    //----------------------------------------

    size_t capture_sample_count_{0};

    double capture_velocity_sum_{0.0};
    double capture_velocity_squared_sum_{0.0};

    float newest_velocity_{0.0F};
    float newest_mean_deviation_{0.0F};

    //----------------------------------------
    // Sweep state
    //----------------------------------------

    int current_pwm_{0};
    bool reverse_{false};

    //----------------------------------------
    // Timing and state
    //----------------------------------------

    uint32_t state_time_ms_{0};

    State state_{State::Idle};
    bool running_{false};

    //----------------------------------------
    // Final result
    //----------------------------------------

    std::optional<CalibrationResult> result_;
};

#endif  // FEEDFORWARD_CALIBRATION_RUNNER_HPP