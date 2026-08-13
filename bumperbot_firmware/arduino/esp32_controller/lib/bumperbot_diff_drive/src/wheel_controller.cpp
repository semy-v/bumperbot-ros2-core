#include "diff_drive/wheel_controller.hpp"

WheelController::WheelController(uint8_t direction_pin,
                                 uint8_t speed_control_pin,
                                 uint8_t speed_state_pin,
                                 float ticks_per_rev,
                                 bool invert_logic)
    : motor_(direction_pin, speed_control_pin, invert_logic),
      encoder_(direction_pin, speed_state_pin, invert_logic),
      velocity_estimator_(ticks_per_rev),
      pid_(&current_velocity_, &pid_correction_pwm_, &target_velocity_) {}

void WheelController::configure(const uint32_t control_period_ms, const WheelConfig& wheel_config) {
    velocity_estimator_.configure(control_period_ms);

    feedforward_ks_forward_ = wheel_config.feedforward_ks_forward;
    feedforward_ks_reverse_ = wheel_config.feedforward_ks_reverse;
    feedforward_kv_ = wheel_config.feedforward_kv;
    max_pid_correction_pwm_ = wheel_config.max_feedback_pwm;

    pid_.SetTunings(wheel_config.feedback_kp, wheel_config.feedback_ki, wheel_config.feedback_kd);
    pid_.SetSampleTimeUs(control_period_ms * 1000U);
    pid_.SetAntiWindupMode(QuickPID::iAwMode::iAwCondition);

    feedforward_pwm_ = calculateFeedForwardPwm();
    updatePidOutputLimits();
    resetPid();

    pid_.SetMode(QuickPID::Control::automatic);
}

void WheelController::begin() {
    encoder_.begin();
    motor_.begin();
}

void WheelController::setActive(bool active) {
    if (active) {
        if (ControlState::Inactive == state_) {
            if (target_velocity_ == 0.0F) {
                enterStopped();
            } else {
                prepareMotionStart();
            }
        }
    } else {
        if (ControlState::Inactive != state_) {
            // Clear target velocity
            target_velocity_ = 0.0f;

            stopMotor();

            resetPid();

            velocity_estimator_.reset();
            current_velocity_ = 0.0f;

            state_ = ControlState::Inactive;
        }
    }
}

void WheelController::setTargetVelocity(float target) {
    constexpr float kTargetZeroEpsilon{0.05F};

    if (std::fabs(target) < kTargetZeroEpsilon) {
        target = 0.0f;
    }

    if (target == target_velocity_) {
        return;
    }

    const bool stop = target == 0.0f;
    const bool reversal = !stop && target_velocity_ != 0.0f && target != 0.0f &&
                          std::signbit(target) != std::signbit(target_velocity_);

    target_velocity_ = target;

    feedforward_pwm_ = calculateFeedForwardPwm();
    updatePidOutputLimits();

    switch (state_) {
        case ControlState::Inactive:
            // Activation chooses Stopped or Normal.
            break;

        case ControlState::Stopped:
            if (!stop) {
                prepareMotionStart();
            }
            break;

        case ControlState::Normal:
            if (stop || reversal) {
                beginBraking();
            }
            break;

        case ControlState::Brake:
            /*
             * Continue braking. The newest target determines whether braking
             * completes into Stopped or Normal.
             */
            break;
    }
}

void WheelController::updatePidOutputLimits() {
    const float actuator_minimum_correction = kMinPwm - feedforward_pwm_;
    const float actuator_maximum_correction = kMaxPwm - feedforward_pwm_;
    const float min_pid = std::max(-max_pid_correction_pwm_, actuator_minimum_correction);
    const float max_pid = std::min(max_pid_correction_pwm_, actuator_maximum_correction);

    pid_.SetOutputLimits(min_pid, max_pid);
}

void WheelController::resetPid() {
    pid_.Reset();
    pid_.SetOutputSum(0.0f);
    pid_correction_pwm_ = 0.0f;
}

void WheelController::update(uint32_t dt_ms) {
    switch (state_) {
        case ControlState::Inactive:
        case ControlState::Stopped:
            current_velocity_ = 0.0F;
            break;

        case ControlState::Normal:
            updateNormal();
            break;

        case ControlState::Brake:
            if (updateBrake(dt_ms)) {
                completeBraking();
            }
            break;
    }
}

void WheelController::updateNormal() {
    current_velocity_ = velocity_estimator_.update(encoder_.getEdgeData());
    pid_.Compute();
    motor_.setPwmSpeed(calculateMotorSpeedPwm());
}

bool WheelController::updateBrake(const uint32_t dt_ms) {
    constexpr float kVelocityThreshold = 2.0;  // rad/s
    constexpr uint32_t kMaxBrakePeriodMs{40};

    current_velocity_ = velocity_estimator_.update(encoder_.getEdgeData());

    if (std::fabs(current_velocity_) < kVelocityThreshold) {
        return true;
    }

    motor_.setPwmSpeed(brake_pwm_);

    brake_total_period_ms_ += dt_ms;

    return brake_total_period_ms_ >= kMaxBrakePeriodMs;
}

void WheelController::prepareMotionStart() {
    current_velocity_ = 0.0f;

    velocity_estimator_.reset(encoder_.getLastEdgeTimeUs(), true);
    resetPid();
    state_ = ControlState::Normal;
}

void WheelController::beginBraking() {
    brake_pwm_ = calculateBrakePwm();

    brake_total_period_ms_ = 0U;

    resetPid();

    state_ = ControlState::Brake;
}

void WheelController::completeBraking() {
    if (target_velocity_ == 0.0F) {
        enterStopped();
    } else {
        stopMotor();
        /*
         * Reversal or a new nonzero target received during braking.
         * Synchronize the estimator to the current encoder timestamp without
         * resetting PCNT.
         */
        prepareMotionStart();
    }
}

void WheelController::enterStopped() {
    stopMotor();

    current_velocity_ = 0.0f;
    feedforward_pwm_ = 0.0f;

    resetPid();
    updatePidOutputLimits();

    state_ = ControlState::Stopped;
}

float WheelController::calculateFeedForwardPwm() const {
    if (target_velocity_ == 0.0f) {
        return 0.0f;
    }
    const float ks = target_velocity_ > 0.0f ? feedforward_ks_forward_ : feedforward_ks_reverse_;

    const float magnitude =
        std::clamp(ks + feedforward_kv_ * std::fabs(target_velocity_), 0.0F, kMaxPwm);

    return std::copysign(magnitude, target_velocity_);
}

int WheelController::calculateMotorSpeedPwm() const {
    float combined_pwm = std::clamp(feedforward_pwm_ + pid_correction_pwm_, kMinPwm, kMaxPwm);

    if (target_velocity_ > 0.0f) {
        combined_pwm = std::max(0.0f, combined_pwm);
    } else if (target_velocity_ < 0.0f) {
        combined_pwm = std::min(0.0f, combined_pwm);
    } else {
        combined_pwm = 0.0f;
    }

    return std::lround(combined_pwm);
}