#include "feedforward_calibration_runner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

FeedForwardCalibrationRunner::FeedForwardCalibrationRunner(const char* runner_name,
                                                           BLDC2430Motor& motor,
                                                           BLDC2430Encoder& encoder,
                                                           uint32_t update_period_ms,
                                                           float ticks_per_rev,
                                                           Print& logger,
                                                           const Config& config)
    : runner_name_{runner_name},
      motor_{motor},
      encoder_{encoder},
      estimator_{ticks_per_rev},
      logger_{logger},
      config_{config} {
    estimator_.configure(update_period_ms);
}

void FeedForwardCalibrationRunner::begin() {
    encoder_.begin();
    motor_.begin();
    motor_.setPwmSpeed(0);

    estimator_.reset();
    resetRegression();
    state_ = State::Idle;

    printPrefix();
    logger_.println("initialized");
}

void FeedForwardCalibrationRunner::stop() {
    motor_.setPwmSpeed(0);
    estimator_.reset();
    resetCaptureAccumulator();

    if (state_ != State::Finished && state_ != State::Failed) {
        state_ = State::Idle;
    }
}

bool FeedForwardCalibrationRunner::configIsValid() const {
    if (config_.velocity_pwm_start <= 0 || config_.velocity_pwm_end < config_.velocity_pwm_start ||
        config_.velocity_pwm_end > 255 || config_.velocity_pwm_step <= 0) {
        return false;
    }

    // The production model has direction-specific Ks, therefore both
    // directions must be measured by this diagnostic.
    if (!config_.calibrate_forward || !config_.calibrate_reverse) {
        return false;
    }

    if (config_.settle_time_ms == 0 || config_.capture_time_ms == 0 ||
        config_.capture_sample_count == 0 ||
        config_.max_capture_wait_ms < config_.capture_time_ms) {
        return false;
    }

    if (!(config_.max_capture_relative_stddev > 0.0F) ||
        !(config_.max_capture_relative_mean_drift > 0.0F) ||
        !(config_.minimum_regression_velocity > 0.0F) ||
        config_.minimum_regression_points_per_direction < 2) {
        return false;
    }

    const size_t magnitude_count =
        1U + static_cast<size_t>((config_.velocity_pwm_end - config_.velocity_pwm_start) /
                                 config_.velocity_pwm_step);

    return magnitude_count * 2U <= kMaxSamples;
}

bool FeedForwardCalibrationRunner::startCalibration() {
    if ((state_ != State::Idle && state_ != State::Finished && state_ != State::Failed) ||
        !configIsValid()) {
        printPrefix();
        logger_.println("cannot start directional feed-forward calibration");
        return false;
    }

    motor_.setPwmSpeed(0);
    estimator_.reset();
    resetRegression();

    current_pwm_ = config_.velocity_pwm_start;
    enterState(State::ApplyTestPWM);

    printPrefix();
    logger_.printf(
        "calibration start: |PWM|=%d..%d step=%d; direct-from-rest test; "
        "settle=%lu ms; capture=%lu ms; model={Ks_forward,Ks_reverse,Kv}\n",
        config_.velocity_pwm_start, config_.velocity_pwm_end, config_.velocity_pwm_step,
        static_cast<unsigned long>(config_.settle_time_ms),
        static_cast<unsigned long>(config_.capture_time_ms));

    return true;
}

bool FeedForwardCalibrationRunner::waitingForPeer() const {
    return state_ == State::WaitForPeer;
}

bool FeedForwardCalibrationRunner::finished() const {
    return state_ == State::Finished;
}

bool FeedForwardCalibrationRunner::failed() const {
    return state_ == State::Failed;
}

void FeedForwardCalibrationRunner::enterState(State next) {
    state_ = next;
    state_time_ms_ = 0;

    switch (next) {
        case State::Idle:
        case State::ApplyTestPWM:
        case State::Settling:
        case State::Finished:
        case State::Failed:
            break;

        case State::CaptureSample:
            resetCaptureAccumulator();
            printPrefix();
            logger_.printf(
                "capturing test PWM %d for %lu ms; acceptance: rel_sigma<=%.1f%%, "
                "half_mean_drift<=%.1f%%, min|v|=%.2f rad/s\n",
                current_pwm_, static_cast<unsigned long>(config_.capture_time_ms),
                100.0F * config_.max_capture_relative_stddev,
                100.0F * config_.max_capture_relative_mean_drift,
                config_.minimum_regression_velocity);
            break;

        case State::Brake:
            motor_.setPwmSpeed(0);
            printPrefix();
            logger_.println("point complete; PWM command zero");
            break;

        case State::Stop:
            motor_.setPwmSpeed(0);
            estimator_.reset();
            printPrefix();
            logger_.println("stationary pause");
            break;

        case State::WaitForPeer:
            motor_.setPwmSpeed(0);
            printPrefix();
            logger_.printf("waiting for peer after test PWM %d\n", current_pwm_);
            break;
    }
}

void FeedForwardCalibrationRunner::update(uint32_t dt_ms) {
    if (state_ == State::Idle || state_ == State::WaitForPeer || state_ == State::Finished ||
        state_ == State::Failed) {
        return;
    }

    state_time_ms_ += dt_ms;

    switch (state_) {
        case State::Idle:
        case State::WaitForPeer:
        case State::Finished:
        case State::Failed:
            break;

        case State::ApplyTestPWM:
            updateApplyTestPWM();
            break;

        case State::Settling:
            updateSettling();
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
    }
}

void FeedForwardCalibrationRunner::updateApplyTestPWM() {
    /*
     * Start every test with fresh estimator history.
     *
     * The encoder itself may still contain the last falling-edge timestamp from
     * the previous/coasting wheel motion.  Mark that edge as consumed and
     * discard the first new edge so the first reciprocal period used by the
     * estimator is a full, fresh FG period generated at this test command.
     */
    const EncoderEdgeData edge = encoder_.getEdgeData();
    estimator_.reset(edge.edge_time_us, true);

    motor_.setPwmSpeed(current_pwm_);

    printPrefix();
    logger_.printf("applying TEST %s PWM %d directly from rest\n",
                   current_pwm_ < 0 ? "reverse" : "forward", current_pwm_);

    enterState(State::Settling);
}

void FeedForwardCalibrationRunner::updateSettling() {
    // Keep the same production estimator warm while the motor accelerates.
    (void)measureVelocity();

    if (state_time_ms_ >= config_.settle_time_ms) {
        enterState(State::CaptureSample);
    }
}

float FeedForwardCalibrationRunner::measureVelocity() {
    return estimator_.update(encoder_.getEdgeData());
}

void FeedForwardCalibrationRunner::resetCaptureAccumulator() {
    capture_sample_count_ = 0;
    capture_velocity_sum_ = 0.0;
    capture_velocity_squared_sum_ = 0.0;

    capture_first_half_count_ = 0;
    capture_first_half_sum_ = 0.0;

    capture_second_half_count_ = 0;
    capture_second_half_sum_ = 0.0;

    capture_min_velocity_ = std::numeric_limits<float>::infinity();
    capture_max_velocity_ = -std::numeric_limits<float>::infinity();
}

void FeedForwardCalibrationRunner::accumulateCaptureSample(float velocity,
                                                           uint32_t capture_elapsed_ms) {
    capture_velocity_sum_ += velocity;
    capture_velocity_squared_sum_ += static_cast<double>(velocity) * static_cast<double>(velocity);
    ++capture_sample_count_;

    capture_min_velocity_ = std::min(capture_min_velocity_, velocity);
    capture_max_velocity_ = std::max(capture_max_velocity_, velocity);

    const uint32_t half_time_ms = config_.capture_time_ms / 2U;

    if (capture_elapsed_ms <= half_time_ms) {
        capture_first_half_sum_ += velocity;
        ++capture_first_half_count_;
    } else {
        capture_second_half_sum_ += velocity;
        ++capture_second_half_count_;
    }
}

bool FeedForwardCalibrationRunner::captureRequirementsSatisfied() const {
    return state_time_ms_ >= config_.capture_time_ms &&
           capture_sample_count_ >= config_.capture_sample_count;
}

void FeedForwardCalibrationRunner::updateCaptureSample() {
    const float velocity = measureVelocity();

    // Keep the capture statistically honest.  Do not discard zero, low-speed,
    // or wrong-direction values here: those conditions are part of the behavior
    // being calibrated and are rejected only after the full observation window.
    if (std::isfinite(velocity)) {
        accumulateCaptureSample(velocity, state_time_ms_);
    }

    if (captureRequirementsSatisfied()) {
        finishSampleCapture();
        enterState(State::Brake);
        return;
    }

    if (state_time_ms_ >= config_.max_capture_wait_ms) {
        printPrefix();
        logger_.printf("capture timeout at PWM %d after %lu ms; finite samples=%u/%u\n",
                       current_pwm_, static_cast<unsigned long>(state_time_ms_),
                       static_cast<unsigned>(capture_sample_count_),
                       static_cast<unsigned>(config_.capture_sample_count));

        if (capture_sample_count_ > 0) {
            // Finalize for diagnostic logging.  The candidate will still be
            // rejected when it does not satisfy the minimum capture count or
            // the stability requirements.
            finishSampleCapture();
        }

        enterState(State::Brake);
    }
}

const char* FeedForwardCalibrationRunner::sampleRejectionReason(const Sample& sample) const {
    if (!std::isfinite(sample.velocity) || !std::isfinite(sample.stddev) ||
        !std::isfinite(sample.relative_stddev) || !std::isfinite(sample.first_half_mean) ||
        !std::isfinite(sample.second_half_mean) || !std::isfinite(sample.relative_mean_drift)) {
        return "non-finite velocity statistics";
    }

    if (sample.pwm == 0) {
        return "zero PWM";
    }

    if (sample.capture_count < config_.capture_sample_count) {
        return "insufficient capture samples";
    }

    if (std::fabs(sample.velocity) < config_.minimum_regression_velocity) {
        return "mean velocity below regression minimum";
    }

    if (std::signbit(sample.velocity) != std::signbit(static_cast<float>(sample.pwm))) {
        return "mean velocity direction does not match PWM direction";
    }

    if (sample.relative_stddev > config_.max_capture_relative_stddev) {
        return "capture relative velocity standard deviation too high";
    }

    if (sample.relative_mean_drift > config_.max_capture_relative_mean_drift) {
        return "capture mean still drifting";
    }

    return nullptr;
}

bool FeedForwardCalibrationRunner::sampleIsUsable(const Sample& sample) const {
    return sampleRejectionReason(sample) == nullptr;
}

void FeedForwardCalibrationRunner::finishSampleCapture() {
    if (capture_sample_count_ == 0) {
        printPrefix();
        logger_.printf("sample rejected: PWM=%d, reason=no finite capture samples\n", current_pwm_);
        return;
    }

    const double count = static_cast<double>(capture_sample_count_);
    const double mean = capture_velocity_sum_ / count;
    const double mean_square = capture_velocity_squared_sum_ / count;
    const double variance = std::max(0.0, mean_square - mean * mean);
    const double stddev = std::sqrt(variance);

    const double first_half_mean =
        capture_first_half_count_ > 0
            ? capture_first_half_sum_ / static_cast<double>(capture_first_half_count_)
            : std::numeric_limits<double>::quiet_NaN();

    const double second_half_mean =
        capture_second_half_count_ > 0
            ? capture_second_half_sum_ / static_cast<double>(capture_second_half_count_)
            : std::numeric_limits<double>::quiet_NaN();

    const double abs_mean = std::fabs(mean);
    const double relative_stddev = abs_mean > std::numeric_limits<double>::epsilon()
                                       ? stddev / abs_mean
                                       : std::numeric_limits<double>::infinity();

    const double relative_mean_drift =
        abs_mean > std::numeric_limits<double>::epsilon() && std::isfinite(first_half_mean) &&
                std::isfinite(second_half_mean)
            ? std::fabs(second_half_mean - first_half_mean) / abs_mean
            : std::numeric_limits<double>::infinity();

    const Sample candidate{
        .pwm = current_pwm_,
        .velocity = static_cast<float>(mean),
        .stddev = static_cast<float>(stddev),
        .relative_stddev = static_cast<float>(relative_stddev),
        .first_half_mean = static_cast<float>(first_half_mean),
        .second_half_mean = static_cast<float>(second_half_mean),
        .relative_mean_drift = static_cast<float>(relative_mean_drift),
        .capture_count = capture_sample_count_,
    };

    printPrefix();
    logger_.printf(
        "capture: test_PWM=%4d, mean=%8.3f rad/s, sigma=%6.4f (%.2f%%), "
        "first=%8.3f, second=%8.3f, drift=%.2f%%, min=%8.3f, max=%8.3f, count=%u\n",
        candidate.pwm, candidate.velocity, candidate.stddev, 100.0F * candidate.relative_stddev,
        candidate.first_half_mean, candidate.second_half_mean,
        100.0F * candidate.relative_mean_drift, capture_min_velocity_, capture_max_velocity_,
        static_cast<unsigned>(candidate.capture_count));

    if (const char* reason = sampleRejectionReason(candidate); reason != nullptr) {
        printPrefix();
        logger_.printf("sample rejected: PWM=%d, reason=%s\n", candidate.pwm, reason);
        return;
    }

    if (sample_count_ >= samples_.size()) {
        printPrefix();
        logger_.println("sample buffer full; point discarded");
        return;
    }

    samples_[sample_count_++] = candidate;
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
        enterState(State::ApplyTestPWM);
        return;
    }

    computeRegression();

    if (result_) {
        state_ = State::Finished;
        printResult();
    } else {
        state_ = State::Failed;
    }
}

bool FeedForwardCalibrationRunner::selectNextSweepPoint() {
    const int magnitude = std::abs(current_pwm_);

    // Sample +PWM then -PWM at every magnitude.  The measurements are kept
    // separate in regression so each direction gets its own Ks while Kv is
    // fitted jointly.
    if (current_pwm_ > 0) {
        current_pwm_ = -magnitude;
        return true;
    }

    const int next = magnitude + config_.velocity_pwm_step;
    if (next > config_.velocity_pwm_end) {
        return false;
    }

    current_pwm_ = next;
    return true;
}

void FeedForwardCalibrationRunner::resetRegression() {
    samples_.fill(Sample{});
    sample_count_ = 0;
    resetCaptureAccumulator();
    result_.reset();
}

void FeedForwardCalibrationRunner::computeRegression() {
    result_.reset();

    DirectionAccumulator forward{};
    DirectionAccumulator reverse{};

    for (size_t i = 0; i < sample_count_; ++i) {
        const Sample& sample = samples_[i];
        if (!sampleIsUsable(sample)) {
            continue;
        }

        const double x = std::fabs(static_cast<double>(sample.velocity));
        const double y = static_cast<double>(std::abs(sample.pwm));

        DirectionAccumulator& direction = sample.pwm > 0 ? forward : reverse;

        ++direction.count;
        direction.sum_x += x;
        direction.sum_y += y;

        printPrefix();
        logger_.printf(
            "regression sample: %s |PWM|=%d |v|=%.3f sigma=%.4f rel_sigma=%.2f%% drift=%.2f%%\n",
            sample.pwm > 0 ? "forward" : "reverse", std::abs(sample.pwm), x, sample.stddev,
            100.0F * sample.relative_stddev, 100.0F * sample.relative_mean_drift);
    }

    if (forward.count < config_.minimum_regression_points_per_direction ||
        reverse.count < config_.minimum_regression_points_per_direction) {
        printPrefix();
        logger_.printf(
            "regression failed: forward points=%u reverse points=%u; minimum per direction=%u\n",
            static_cast<unsigned>(forward.count), static_cast<unsigned>(reverse.count),
            static_cast<unsigned>(config_.minimum_regression_points_per_direction));
        return;
    }

    const double forward_mean_x = forward.sum_x / static_cast<double>(forward.count);
    const double forward_mean_y = forward.sum_y / static_cast<double>(forward.count);
    const double reverse_mean_x = reverse.sum_x / static_cast<double>(reverse.count);
    const double reverse_mean_y = reverse.sum_y / static_cast<double>(reverse.count);

    // Shared-slope/group-intercept least squares:
    //   y_f = Ks_f + Kv*x_f
    //   y_r = Ks_r + Kv*x_r
    double centered_xy = 0.0;
    double centered_x2 = 0.0;

    for (size_t i = 0; i < sample_count_; ++i) {
        const Sample& sample = samples_[i];
        if (!sampleIsUsable(sample)) {
            continue;
        }

        const bool is_forward = sample.pwm > 0;
        const double x = std::fabs(static_cast<double>(sample.velocity));
        const double y = static_cast<double>(std::abs(sample.pwm));

        const double mean_x = is_forward ? forward_mean_x : reverse_mean_x;
        const double mean_y = is_forward ? forward_mean_y : reverse_mean_y;

        const double dx = x - mean_x;
        const double dy = y - mean_y;

        centered_xy += dx * dy;
        centered_x2 += dx * dx;
    }

    if (centered_x2 <= std::numeric_limits<double>::epsilon()) {
        printPrefix();
        logger_.println("regression failed: insufficient velocity spread");
        return;
    }

    const double kv = centered_xy / centered_x2;
    const double ks_forward = forward_mean_y - kv * forward_mean_x;
    const double ks_reverse = reverse_mean_y - kv * reverse_mean_x;

    if (!std::isfinite(ks_forward) || !std::isfinite(ks_reverse) || !std::isfinite(kv) ||
        ks_forward < 0.0 || ks_reverse < 0.0 || kv <= 0.0) {
        printPrefix();
        logger_.println("regression failed: invalid directional Ks/Kv");
        return;
    }

    const size_t total_count = forward.count + reverse.count;
    const double overall_mean_y =
        (forward.sum_y + reverse.sum_y) / static_cast<double>(total_count);

    double residual_ss = 0.0;
    double total_ss = 0.0;

    for (size_t i = 0; i < sample_count_; ++i) {
        const Sample& sample = samples_[i];
        if (!sampleIsUsable(sample)) {
            continue;
        }

        const double x = std::fabs(static_cast<double>(sample.velocity));
        const double y = static_cast<double>(std::abs(sample.pwm));
        const double ks = sample.pwm > 0 ? ks_forward : ks_reverse;
        const double predicted = ks + kv * x;
        const double residual = y - predicted;

        residual_ss += residual * residual;

        const double deviation = y - overall_mean_y;
        total_ss += deviation * deviation;
    }

    result_ = CalibrationResult{
        .ks_forward = static_cast<float>(ks_forward),
        .ks_reverse = static_cast<float>(ks_reverse),
        .kv = static_cast<float>(kv),
        .r_squared = total_ss > std::numeric_limits<double>::epsilon()
                         ? static_cast<float>(1.0 - residual_ss / total_ss)
                         : 0.0F,
        .rmse_pwm = static_cast<float>(std::sqrt(residual_ss / static_cast<double>(total_count))),
        .forward_sample_count = forward.count,
        .reverse_sample_count = reverse.count,
    };
}

void FeedForwardCalibrationRunner::printPrefix() const {
    logger_.printf("[FF:%s] ", runner_name_ != nullptr ? runner_name_ : "UNKNOWN");
}

void FeedForwardCalibrationRunner::printResult() const {
    logger_.println();
    printPrefix();
    logger_.println("========================================");
    printPrefix();
    logger_.println("directional moving-state calibration complete");
    printPrefix();
    logger_.println("model: one Kv with direction-specific Ks; no startup PWM; no preconditioning");

    if (!result_) {
        printPrefix();
        logger_.println("FAILED: no valid calibration result");
        return;
    }

    printPrefix();
    logger_.printf("Ks_forward = %.4f PWM\n", result_->ks_forward);
    printPrefix();
    logger_.printf("Ks_reverse = %.4f PWM\n", result_->ks_reverse);
    printPrefix();
    logger_.printf("Kv = %.4f PWM/(rad/s)\n", result_->kv);
    printPrefix();
    logger_.printf("R^2 = %.5f\n", result_->r_squared);
    printPrefix();
    logger_.printf("RMSE = %.4f PWM\n", result_->rmse_pwm);
    printPrefix();
    logger_.printf("regression points: forward=%u reverse=%u\n",
                   static_cast<unsigned>(result_->forward_sample_count),
                   static_cast<unsigned>(result_->reverse_sample_count));

    printPrefix();
    logger_.printf("forward: pwm_ff = %.4f + %.4f * |velocity|\n", result_->ks_forward,
                   result_->kv);
    printPrefix();
    logger_.printf("reverse: pwm_ff = -(%.4f + %.4f * |velocity|)\n", result_->ks_reverse,
                   result_->kv);

    printPrefix();
    logger_.printf("predicted |PWM| at 2 rad/s: forward=%.2f reverse=%.2f\n",
                   result_->ks_forward + 2.0F * result_->kv,
                   result_->ks_reverse + 2.0F * result_->kv);

    if (result_->r_squared < 0.98F) {
        printPrefix();
        logger_.println("WARNING: R^2 < 0.98; inspect directional samples");
    }

    if (result_->rmse_pwm > 5.0F) {
        printPrefix();
        logger_.println("WARNING: RMSE > 5 PWM; inspect low-speed/load stability");
    }

    printPrefix();
    logger_.println("========================================");
}
