#include "feedforward_calibration_runner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

FeedForwardCalibrationRunner::FeedForwardCalibrationRunner(const char* runner_name,
                                                           BL2418Motor& motor,
                                                           BL2418Encoder& encoder,
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
    resetVelocityWindow();
    resetRegression();
    state_ = State::Idle;

    printPrefix();
    logger_.println("initialized");
}

void FeedForwardCalibrationRunner::stop() {
    motor_.setPwmSpeed(0);
    estimator_.reset();
    resetVelocityWindow();

    if (state_ != State::Finished && state_ != State::Failed) {
        state_ = State::Idle;
    }
}

bool FeedForwardCalibrationRunner::configIsValid() const {
    if (config_.velocity_pwm_start <= 0 || config_.velocity_pwm_end < config_.velocity_pwm_start ||
        config_.velocity_pwm_end > 255 || config_.velocity_pwm_step <= 0) {
        return false;
    }

    // Direction-specific Ks requires measurements in both directions.
    if (!config_.calibrate_forward || !config_.calibrate_reverse) {
        return false;
    }

    if (config_.precondition_pwm <= 0 || config_.precondition_pwm > 255 ||
        config_.precondition_time_ms == 0) {
        return false;
    }

    if (config_.capture_time_ms == 0 && config_.capture_sample_count == 0) {
        return false;
    }

    if (config_.capture_time_ms != 0 && config_.max_capture_wait_ms < config_.capture_time_ms) {
        return false;
    }

    if (config_.minimum_regression_velocity <= 0.0F || config_.max_regression_stddev <= 0.0F ||
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
    resetVelocityWindow();
    resetRegression();

    current_pwm_ = config_.velocity_pwm_start;
    enterState(State::ApplyPrecondition);

    printPrefix();
    logger_.printf(
        "calibration start: |PWM|=%d..%d step=%d; "
        "calibration-only precondition=%d PWM for %lu ms; "
        "model={Ks_forward,Ks_reverse,Kv}\n",
        config_.velocity_pwm_start, config_.velocity_pwm_end, config_.velocity_pwm_step,
        config_.precondition_pwm, static_cast<unsigned long>(config_.precondition_time_ms));

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
        case State::Finished:
        case State::Failed:
            break;

        case State::ApplyPrecondition:
            estimator_.reset();
            resetVelocityWindow();
            break;

        case State::Precondition:
            resetVelocityWindow();
            break;

        case State::ApplyTestPWM:
            break;

        case State::Settling:
            resetVelocityWindow();
            break;

        case State::WaitForSteadyState:
            resetVelocityWindow();
            printPrefix();
            logger_.printf("waiting for steady state at test PWM %d\n", current_pwm_);
            break;

        case State::CaptureSample:
            resetCaptureAccumulator();
            printPrefix();
            logger_.printf("capturing test PWM %d: duration=%lu ms, samples=%u\n", current_pwm_,
                           static_cast<unsigned long>(config_.capture_time_ms),
                           static_cast<unsigned>(config_.capture_sample_count));
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

        case State::ApplyPrecondition:
            updateApplyPrecondition();
            break;

        case State::Precondition:
            updatePrecondition();
            break;

        case State::ApplyTestPWM:
            updateApplyTestPWM();
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
    }
}

int FeedForwardCalibrationRunner::signedPreconditionPwm() const {
    const int magnitude = std::max(std::abs(current_pwm_), config_.precondition_pwm);

    return current_pwm_ > 0 ? magnitude : -magnitude;
}

void FeedForwardCalibrationRunner::updateApplyPrecondition() {
    const int pwm = signedPreconditionPwm();
    motor_.setPwmSpeed(pwm);

    printPrefix();
    logger_.printf("precondition: direction=%s, PWM=%d for %lu ms before test_PWM=%d\n",
                   current_pwm_ > 0 ? "forward" : "reverse", pwm,
                   static_cast<unsigned long>(config_.precondition_time_ms), current_pwm_);

    enterState(State::Precondition);
}

void FeedForwardCalibrationRunner::updatePrecondition() {
    (void)measureVelocity();

    if (state_time_ms_ >= config_.precondition_time_ms) {
        enterState(State::ApplyTestPWM);
    }
}

void FeedForwardCalibrationRunner::updateApplyTestPWM() {
    motor_.setPwmSpeed(current_pwm_);

    printPrefix();
    logger_.printf("applying TEST %s PWM %d after calibration-only preconditioning\n",
                   current_pwm_ < 0 ? "reverse" : "forward", current_pwm_);

    enterState(State::Settling);
}

void FeedForwardCalibrationRunner::updateSettling() {
    (void)measureVelocity();

    if (state_time_ms_ >= config_.settle_time_ms) {
        enterState(State::WaitForSteadyState);
    }
}

bool FeedForwardCalibrationRunner::steadyStateReached() {
    newest_velocity_ = measureVelocity();

    if (!std::isfinite(newest_velocity_) || velocity_window_count_ < kVelocityWindowSize ||
        state_time_ms_ < config_.steady_time_ms) {
        return false;
    }

    computeWindowStatistics();
    newest_mean_deviation_ = std::fabs(newest_velocity_ - velocity_mean_);

    const bool direction_matches =
        std::signbit(velocity_mean_) == std::signbit(static_cast<float>(current_pwm_));

    const bool speed_is_sufficient =
        std::fabs(velocity_mean_) >= config_.minimum_regression_velocity;

    return direction_matches && speed_is_sufficient &&
           velocity_stddev_ <= config_.velocity_stddev_limit &&
           newest_mean_deviation_ <= config_.velocity_mean_deviation_limit;
}

void FeedForwardCalibrationRunner::updateSteadyState() {
    if (steadyStateReached()) {
        enterState(State::CaptureSample);
        return;
    }

    if (state_time_ms_ >= config_.max_steady_wait_ms) {
        computeWindowStatistics();
        newest_mean_deviation_ = std::fabs(newest_velocity_ - velocity_mean_);

        printPrefix();
        logger_.printf(
            "test PWM %d rejected before capture: steady-state timeout; "
            "mean=%8.3f, newest=%8.3f, sigma=%.4f/%.4f, "
            "mean_delta=%.4f/%.4f, min|v|=%.2f\n",
            current_pwm_, velocity_mean_, newest_velocity_, velocity_stddev_,
            config_.velocity_stddev_limit, newest_mean_deviation_,
            config_.velocity_mean_deviation_limit, config_.minimum_regression_velocity);

        enterState(State::Brake);
    }
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
    newest_velocity_ = 0.0F;
    newest_mean_deviation_ = 0.0F;
}

void FeedForwardCalibrationRunner::computeWindowStatistics() {
    if (velocity_window_count_ == 0) {
        velocity_mean_ = 0.0F;
        velocity_stddev_ = 0.0F;
        return;
    }

    double sum = 0.0;
    for (size_t i = 0; i < velocity_window_count_; ++i) {
        sum += velocity_window_[i];
    }

    velocity_mean_ = static_cast<float>(sum / static_cast<double>(velocity_window_count_));

    double squared_error_sum = 0.0;
    for (size_t i = 0; i < velocity_window_count_; ++i) {
        const double error = static_cast<double>(velocity_window_[i]) - velocity_mean_;
        squared_error_sum += error * error;
    }

    velocity_stddev_ = static_cast<float>(
        std::sqrt(squared_error_sum / static_cast<double>(velocity_window_count_)));
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
            logger_.printf("capture timeout at PWM %d; finalizing %u samples\n", current_pwm_,
                           static_cast<unsigned>(capture_sample_count_));
            finishSampleCapture();
        } else {
            logger_.printf("capture timeout at PWM %d; no valid samples\n", current_pwm_);
        }

        enterState(State::Brake);
    }
}

const char* FeedForwardCalibrationRunner::sampleRejectionReason(const Sample& sample) const {
    if (!std::isfinite(sample.velocity) || !std::isfinite(sample.stddev)) {
        return "non-finite velocity statistics";
    }

    if (sample.pwm == 0) {
        return "zero PWM";
    }

    if (std::fabs(sample.velocity) < config_.minimum_regression_velocity) {
        return "velocity below regression minimum";
    }

    if (sample.stddev > config_.max_regression_stddev) {
        return "capture velocity standard deviation too high";
    }

    if (std::signbit(sample.velocity) != std::signbit(static_cast<float>(sample.pwm))) {
        return "velocity direction does not match PWM direction";
    }

    return nullptr;
}

bool FeedForwardCalibrationRunner::sampleIsUsable(const Sample& sample) const {
    return sampleRejectionReason(sample) == nullptr;
}

void FeedForwardCalibrationRunner::finishSampleCapture() {
    if (capture_sample_count_ == 0) {
        return;
    }

    const double count = static_cast<double>(capture_sample_count_);
    const double mean = capture_velocity_sum_ / count;
    const double mean_square = capture_velocity_squared_sum_ / count;
    const double variance = std::max(0.0, mean_square - mean * mean);

    const Sample candidate{
        .pwm = current_pwm_,
        .velocity = static_cast<float>(mean),
        .stddev = static_cast<float>(std::sqrt(variance)),
    };

    printPrefix();
    logger_.printf(
        "capture: test_PWM=%4d, velocity=%8.3f rad/s, sigma=%6.4f, "
        "capture_count=%u, capture_time=%lu ms\n",
        candidate.pwm, candidate.velocity, candidate.stddev,
        static_cast<unsigned>(capture_sample_count_), static_cast<unsigned long>(state_time_ms_));

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
        enterState(State::ApplyPrecondition);
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
    // separate in regression, so direction-specific Ks is retained.
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
        logger_.printf("regression sample: %s |PWM|=%d |v|=%.3f sigma=%.4f\n",
                       sample.pwm > 0 ? "forward" : "reverse", std::abs(sample.pwm), x,
                       sample.stddev);
    }

    if (forward.count < config_.minimum_regression_points_per_direction ||
        reverse.count < config_.minimum_regression_points_per_direction) {
        printPrefix();
        logger_.printf(
            "regression failed: forward points=%u reverse points=%u; "
            "minimum per direction=%u\n",
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
    logger_.println("model: one Kv with direction-specific Ks; no startup PWM");

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
        logger_.println("WARNING: RMSE > 5 PWM; inspect low-speed stability");
    }

    printPrefix();
    logger_.println("========================================");
}