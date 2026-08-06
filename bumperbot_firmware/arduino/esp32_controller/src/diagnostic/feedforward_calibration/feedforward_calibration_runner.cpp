#include "feedforward_calibration_runner.hpp"

#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <limits>

FeedForwardCalibrationRunner::FeedForwardCalibrationRunner(const char* runner_name,
                                                           BL2418Motor& motor,
                                                           BL2418Encoder& encoder,
                                                           uint32_t update_period_ms,
                                                           float ticks_per_rev)
    : FeedForwardCalibrationRunner{runner_name,      motor,         encoder,
                                   update_period_ms, ticks_per_rev, Config{}} {}

FeedForwardCalibrationRunner::FeedForwardCalibrationRunner(const char* runner_name,
                                                           BL2418Motor& motor,
                                                           BL2418Encoder& encoder,
                                                           uint32_t update_period_ms,
                                                           float ticks_per_rev,
                                                           const Config& config)
    : runner_name_{runner_name},
      motor_{motor},
      encoder_{encoder},
      estimator_{ticks_per_rev},
      config_{config} {
    estimator_.configure(update_period_ms);
}

void FeedForwardCalibrationRunner::begin() {
    encoder_.begin();
    motor_.begin();

    estimator_.reset();
    resetRegression();

    motor_.setPwmSpeed(0);

    running_ = false;
    state_ = State::Idle;

    printPrefix();
    Serial.println("initialized");
}

void FeedForwardCalibrationRunner::start() {
    if (config_.pwm_start <= 0 || config_.pwm_end < config_.pwm_start || config_.pwm_step <= 0 ||
        (!config_.calibrate_forward && !config_.calibrate_reverse) ||
        (config_.capture_time_ms == 0 && config_.capture_sample_count == 0)) {
        printPrefix();
        Serial.println("cannot start: invalid calibration configuration");
        return;
    }

    estimator_.reset();
    resetRegression();

    current_pwm_ = config_.pwm_start;

    reverse_ = config_.calibrate_reverse;

    running_ = true;

    printPrefix();
    Serial.printf("starting sweep: PWM %d..%d, step %d, direction=%s\n", config_.pwm_start,
                  config_.pwm_end, config_.pwm_step, reverse_ ? "reverse" : "forward");

    enterState(State::ApplyPWM);
}

void FeedForwardCalibrationRunner::stop() {
    motor_.setPwmSpeed(0);

    running_ = false;
    enterState(State::Idle);

    printPrefix();
    Serial.println("stopped");
}

void FeedForwardCalibrationRunner::update(uint32_t dt_ms) {
    if (!running_) {
        return;
    }

    state_time_ms_ += dt_ms;

    switch (state_) {
        case State::Idle:
            break;

        case State::ApplyPWM:
            updateApplyPWM();
            break;

        case State::Settling:
            updateSettling();
            break;

        case State::WaitForSteadyState:
            updateSteadyState();
            break;

        case State::CaptureSample:
            updateCaptureSample();
            break;

        case State::Brake:
            updateBrake();
            break;

        case State::Stop:
            updateStop();
            break;

        case State::WaitForPeer:
            // Intentionally idle until the external coordinator releases both
            // runners using advanceSynchronizedStep().
            break;

        case State::Finished:
            break;
    }
}

void FeedForwardCalibrationRunner::enterState(State next) {
    state_ = next;
    state_time_ms_ = 0;

    switch (state_) {
        case State::Idle:
            break;

        case State::ApplyPWM:
            resetVelocityWindow();
            break;

        case State::Settling:
            resetVelocityWindow();
            break;

        case State::WaitForSteadyState:
            resetVelocityWindow();

            printPrefix();
            Serial.printf("waiting for steady state at PWM %d\n", current_pwm_);
            break;

        case State::CaptureSample:
            resetCaptureAccumulator();

            printPrefix();
            Serial.printf("capturing PWM %d: duration=%lu ms, samples=%u\n", current_pwm_,
                          static_cast<unsigned long>(config_.capture_time_ms),
                          static_cast<unsigned>(config_.capture_sample_count));
            break;

        case State::Brake:
            motor_.setPwmSpeed(0);

            printPrefix();
            Serial.println("sample complete; braking");
            break;

        case State::Stop:
            motor_.setPwmSpeed(0);
            estimator_.reset();

            printPrefix();
            Serial.println("stationary pause");
            break;

        case State::WaitForPeer:
            motor_.setPwmSpeed(0);

            printPrefix();
            Serial.printf("waiting for peer after PWM %d\n", current_pwm_);
            break;

        case State::Finished:
            motor_.setPwmSpeed(0);
            running_ = false;

            computeRegression();
            printResult();
            break;
    }
}

void FeedForwardCalibrationRunner::updateApplyPWM() {
    estimator_.reset();
    resetVelocityWindow();

    motor_.setPwmSpeed(current_pwm_);

    printPrefix();
    Serial.printf("applying %s PWM %d\n", current_pwm_ < 0 ? "reverse" : "forward", current_pwm_);

    enterState(State::Settling);
}

void FeedForwardCalibrationRunner::updateSettling() {
    // Keep the reciprocal estimator current during acceleration. The velocity
    // window is reset when WaitForSteadyState begins.
    measureVelocity();

    if (state_time_ms_ >= config_.settle_time_ms) {
        enterState(State::WaitForSteadyState);
    }
}

void FeedForwardCalibrationRunner::updateSteadyState() {
    if (steadyStateReached()) {
        enterState(State::CaptureSample);
        return;
    }

    if (state_time_ms_ >= config_.max_steady_wait_ms) {
        computeWindowStatistics();

        printPrefix();
        Serial.printf(
            "steady-state timeout at PWM %d: "
            "mean=%8.3f, newest=%8.3f, "
            "sigma=%.4f/%.4f, "
            "mean_delta=%.4f/%.4f\n",
            current_pwm_, velocity_mean_, newest_velocity_, velocity_stddev_,
            config_.velocity_stddev_limit, newest_mean_deviation_,
            config_.velocity_mean_deviation_limit);

        enterState(State::Brake);
    }
}

void FeedForwardCalibrationRunner::updateCaptureSample() {
    const float velocity = measureVelocity();

    if (std::isfinite(velocity) && std::fabs(velocity) >= config_.minimum_regression_velocity &&
        std::signbit(velocity) == std::signbit(static_cast<float>(current_pwm_))) {
        accumulateCaptureSample(velocity);
    }

    if (captureRequirementsSatisfied()) {
        finishSampleCapture();
        enterState(State::Brake);
        return;
    }

    if (state_time_ms_ >= config_.max_capture_wait_ms) {
        printPrefix();

        if (capture_sample_count_ > 0) {
            Serial.printf("capture timeout at PWM %d; accepting %u samples\n", current_pwm_,
                          static_cast<unsigned>(capture_sample_count_));

            finishSampleCapture();
        } else {
            Serial.printf("capture timeout at PWM %d; no valid samples\n", current_pwm_);
        }

        enterState(State::Brake);
    }
}

void FeedForwardCalibrationRunner::updateBrake() {
    if (state_time_ms_ >= config_.brake_time_ms) {
        enterState(State::Stop);
    }
}

void FeedForwardCalibrationRunner::updateStop() {
    if (state_time_ms_ >= config_.stop_time_ms) {
        enterState(State::WaitForPeer);
    }
}

void FeedForwardCalibrationRunner::advanceSynchronizedStep() {
    if (state_ != State::WaitForPeer) {
        return;
    }

    if (selectNextSweepPoint()) {
        enterState(State::ApplyPWM);
    } else {
        enterState(State::Finished);
    }
}

bool FeedForwardCalibrationRunner::selectNextSweepPoint() {
    if (reverse_ && current_pwm_ > 0) {
        current_pwm_ = -current_pwm_;
        return true;
    }

    current_pwm_ = std::abs(current_pwm_) + config_.pwm_step;

    if (current_pwm_ <= config_.pwm_end) {
        return true;
    }

    return false;
}

float FeedForwardCalibrationRunner::measureVelocity() {
    const EncoderEdgeData edge = encoder_.getEdgeData();
    const float velocity = estimator_.update(edge);

    if (!std::isfinite(velocity) || std::fabs(velocity) < 0.05F) {
        return velocity;
    }

    velocity_window_[velocity_window_index_] = velocity;

    velocity_window_index_ = (velocity_window_index_ + 1U) % kVelocityWindowSize;

    if (velocity_window_count_ < kVelocityWindowSize) {
        ++velocity_window_count_;
    }

    return velocity;
}

void FeedForwardCalibrationRunner::resetVelocityWindow() {
    velocity_window_.fill(0.0F);

    velocity_window_count_ = 0;
    velocity_window_index_ = 0;

    velocity_mean_ = 0.0F;
    velocity_stddev_ = 0.0F;
}

void FeedForwardCalibrationRunner::computeWindowStatistics() {
    if (velocity_window_count_ == 0) {
        velocity_mean_ = 0.0F;
        velocity_stddev_ = 0.0F;
        return;
    }

    double sum = 0.0;

    for (size_t index = 0; index < velocity_window_count_; ++index) {
        sum += velocity_window_[index];
    }

    velocity_mean_ = static_cast<float>(sum / static_cast<double>(velocity_window_count_));

    double squared_error_sum = 0.0;

    for (size_t index = 0; index < velocity_window_count_; ++index) {
        const double error = static_cast<double>(velocity_window_[index]) - velocity_mean_;

        squared_error_sum += error * error;
    }

    const double variance = squared_error_sum / static_cast<double>(velocity_window_count_);

    velocity_stddev_ = static_cast<float>(std::sqrt(variance));
}

bool FeedForwardCalibrationRunner::steadyStateReached() {
    newest_velocity_ = measureVelocity();

    if (velocity_window_count_ < kVelocityWindowSize) {
        return false;
    }

    if (state_time_ms_ < config_.steady_time_ms) {
        return false;
    }

    computeWindowStatistics();

    newest_mean_deviation_ = std::fabs(newest_velocity_ - velocity_mean_);

    const bool variation_is_small = velocity_stddev_ <= config_.velocity_stddev_limit;

    const bool newest_matches_mean =
        newest_mean_deviation_ <= config_.velocity_mean_deviation_limit;

    return variation_is_small && newest_matches_mean;
}

void FeedForwardCalibrationRunner::resetCaptureAccumulator() {
    capture_sample_count_ = 0;
    capture_velocity_sum_ = 0.0;
    capture_velocity_squared_sum_ = 0.0;
}

void FeedForwardCalibrationRunner::accumulateCaptureSample(float velocity) {
    capture_velocity_sum_ += velocity;

    capture_velocity_squared_sum_ += static_cast<double>(velocity) * static_cast<double>(velocity);

    ++capture_sample_count_;
}

bool FeedForwardCalibrationRunner::captureRequirementsSatisfied() const {
    const bool duration_satisfied =
        config_.capture_time_ms == 0 || state_time_ms_ >= config_.capture_time_ms;

    const bool count_satisfied =
        config_.capture_sample_count == 0 || capture_sample_count_ >= config_.capture_sample_count;

    return duration_satisfied && count_satisfied && capture_sample_count_ > 0;
}

void FeedForwardCalibrationRunner::finishSampleCapture() {
    if (capture_sample_count_ == 0) {
        return;
    }

    if (sample_count_ >= samples_.size()) {
        printPrefix();
        Serial.println("sample buffer full; point discarded");
        return;
    }

    const double count = static_cast<double>(capture_sample_count_);

    const double mean = capture_velocity_sum_ / count;

    const double mean_square = capture_velocity_squared_sum_ / count;

    // Guard against a tiny negative result caused by floating-point rounding.
    const double variance = std::max(0.0, mean_square - mean * mean);

    Sample& sample = samples_[sample_count_++];

    sample.pwm = current_pwm_;
    sample.velocity = static_cast<float>(mean);
    sample.stddev = static_cast<float>(std::sqrt(variance));

    printPrefix();
    Serial.printf(
        "sample %02u: PWM=%4d, velocity=%8.3f rad/s, "
        "sigma=%6.4f, capture_count=%u, capture_time=%lu ms\n",
        static_cast<unsigned>(sample_count_), sample.pwm, sample.velocity, sample.stddev,
        static_cast<unsigned>(capture_sample_count_), static_cast<unsigned long>(state_time_ms_));
}

void FeedForwardCalibrationRunner::resetRegression() {
    samples_.fill(Sample{});
    sample_count_ = 0;

    resetVelocityWindow();
    resetCaptureAccumulator();

    result_.reset();
}

bool FeedForwardCalibrationRunner::sampleIsUsable(const Sample& sample) const {
    if (!std::isfinite(sample.velocity) || !std::isfinite(sample.stddev)) {
        return false;
    }

    if (sample.pwm == 0 || std::fabs(sample.velocity) < config_.minimum_regression_velocity) {
        return false;
    }

    if (std::signbit(sample.velocity) != std::signbit(static_cast<float>(sample.pwm))) {
        return false;
    }

    return true;
}

void FeedForwardCalibrationRunner::computeRegression() {
    size_t used_sample_count = 0;

    if (config_.use_fixed_static_friction) {
        // With independently measured Ks, fit only:
        //
        //   |PWM| - Ks = Kv * |velocity|
        //
        // The least-squares line is constrained through the origin.
        double sum_velocity_squared = 0.0;
        double sum_velocity_pwm = 0.0;

        for (size_t index = 0; index < sample_count_; ++index) {
            const Sample& sample = samples_[index];

            if (!sampleIsUsable(sample)) {
                continue;
            }

            const double velocity = std::fabs(sample.velocity);
            const double effective_pwm = std::max(
                0.0, static_cast<double>(std::abs(sample.pwm)) - config_.static_friction_pwm);

            sum_velocity_squared += velocity * velocity;
            sum_velocity_pwm += velocity * effective_pwm;

            ++used_sample_count;
        }

        if (used_sample_count < 2 ||
            sum_velocity_squared <= std::numeric_limits<double>::epsilon()) {
            Serial.println("Feed-forward regression failed: insufficient usable samples.");
            result_.reset();
            return;
        }

        const float kv = static_cast<float>(sum_velocity_pwm / sum_velocity_squared);

        result_ = CalibrationResult{.kv = kv,
                                    .ks = config_.static_friction_pwm,
                                    .r_squared = 0.0F,
                                    .rmse_pwm = 0.0F,
                                    .sample_count = used_sample_count};
    } else {
        // Fit:
        //
        //   y = Ks + Kv*x
        //
        // where:
        //   x = |velocity|
        //   y = |PWM|
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_x_squared = 0.0;
        double sum_xy = 0.0;

        for (size_t index = 0; index < sample_count_; ++index) {
            const Sample& sample = samples_[index];

            if (!sampleIsUsable(sample)) {
                continue;
            }

            const double x = std::fabs(sample.velocity);
            const double y = std::abs(sample.pwm);

            sum_x += x;
            sum_y += y;
            sum_x_squared += x * x;
            sum_xy += x * y;

            ++used_sample_count;
        }

        if (used_sample_count < 2) {
            Serial.println("Feed-forward regression failed: insufficient usable samples.");
            result_.reset();
            return;
        }

        const double sample_count = static_cast<double>(used_sample_count);

        const double denominator = sample_count * sum_x_squared - sum_x * sum_x;

        if (std::fabs(denominator) <= std::numeric_limits<double>::epsilon()) {
            Serial.println(
                "Feed-forward regression failed: velocity samples have "
                "insufficient spread.");
            result_.reset();
            return;
        }

        const double kv = (sample_count * sum_xy - sum_x * sum_y) / denominator;

        const double ks = (sum_y - kv * sum_x) / sample_count;

        if (!std::isfinite(kv) || !std::isfinite(ks) || kv <= 0.0) {
            Serial.println("Feed-forward regression failed: invalid fitted coefficients.");
            result_.reset();
            return;
        }

        result_ = CalibrationResult{.kv = static_cast<float>(kv),
                                    .ks = static_cast<float>(std::max(0.0, ks)),
                                    .r_squared = 0.0F,
                                    .rmse_pwm = 0.0F,
                                    .sample_count = used_sample_count};
    }

    // Compute fit-quality metrics for either regression mode.
    double measured_pwm_sum = 0.0;

    for (size_t index = 0; index < sample_count_; ++index) {
        if (sampleIsUsable(samples_[index])) {
            measured_pwm_sum += std::abs(samples_[index].pwm);
        }
    }

    const double mean_measured_pwm = measured_pwm_sum / static_cast<double>(used_sample_count);

    double residual_sum_squared = 0.0;
    double total_sum_squared = 0.0;

    for (size_t index = 0; index < sample_count_; ++index) {
        const Sample& sample = samples_[index];

        if (!sampleIsUsable(sample)) {
            continue;
        }

        const double measured_pwm = std::abs(sample.pwm);
        const double velocity = std::fabs(sample.velocity);

        const double predicted_pwm =
            static_cast<double>(result_->ks) + static_cast<double>(result_->kv) * velocity;

        const double residual = measured_pwm - predicted_pwm;
        residual_sum_squared += residual * residual;

        const double deviation = measured_pwm - mean_measured_pwm;
        total_sum_squared += deviation * deviation;
    }

    result_->rmse_pwm = static_cast<float>(
        std::sqrt(residual_sum_squared / static_cast<double>(used_sample_count)));

    if (total_sum_squared > std::numeric_limits<double>::epsilon()) {
        result_->r_squared = static_cast<float>(1.0 - residual_sum_squared / total_sum_squared);
    } else {
        result_->r_squared = 0.0F;
    }
}

void FeedForwardCalibrationRunner::printPrefix() const {
    Serial.printf("[FF:%s] ", runner_name_ != nullptr ? runner_name_ : "UNKNOWN");
}

void FeedForwardCalibrationRunner::printResult() const {
    Serial.println();
    printPrefix();
    Serial.println("========================================");

    printPrefix();
    Serial.println("feed-forward calibration complete");

    printPrefix();
    Serial.println("model: PWM = Ks*sign(velocity) + Kv*velocity");

    printPrefix();
    Serial.printf("captured samples: %u\n", static_cast<unsigned>(sample_count_));

    if (!result_) {
        printPrefix();
        Serial.println("FAILED: no valid calibration result");

        printPrefix();
        Serial.println("check steady-state limits and captured samples");

        printPrefix();
        Serial.println("========================================");
        return;
    }

    printPrefix();
    Serial.printf("regression samples: %u\n", static_cast<unsigned>(result_->sample_count));

    printPrefix();
    Serial.printf("Ks = %.4f PWM\n", result_->ks);

    printPrefix();
    Serial.printf("Kv = %.4f PWM/(rad/s)\n", result_->kv);

    printPrefix();
    Serial.printf("R^2 = %.5f\n", result_->r_squared);

    printPrefix();
    Serial.printf("RMSE = %.4f PWM\n", result_->rmse_pwm);

    printPrefix();
    Serial.println("feed-forward equation:");

    printPrefix();
    Serial.printf(
        "pwm_ff = %.4f * sign(target_velocity)"
        " + %.4f * target_velocity\n",
        result_->ks, result_->kv);

    printPrefix();
    Serial.println("configuration:");

    printPrefix();
    Serial.printf("feedforward_ks: %.4f\n", result_->ks);

    printPrefix();
    Serial.printf("feedforward_kv: %.4f\n", result_->kv);

    if (result_->r_squared < 0.95F) {
        printPrefix();
        Serial.println("WARNING: R^2 < 0.95; linear model quality is limited");
    }

    printPrefix();
    Serial.println("========================================");
}