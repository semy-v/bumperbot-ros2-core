#include "diff_drive/wheel_controller.hpp"

#include <algorithm>
#include <cmath>

// -----------------------------------------------------------------------------
// StateBase
// -----------------------------------------------------------------------------

auto WheelController::StateBase::handle(const DeactivateEvent&) noexcept -> TransitionRequest {
    // Deactivation is an unconditional software stop. Clear the target and all
    // controller history before transitioning to Inactive so stale PID or
    // estimator state cannot affect a later activation.
    controller.target_velocity_ = 0.0f;
    controller.stopMotor();
    controller.resetPid();
    controller.velocity_estimator_.reset();
    controller.current_velocity_ = 0.0f;
    controller.feedforward_pwm_ = 0.0f;
    controller.updatePidOutputLimits();

    return TransitionRequest::ToInactive;
}

// -----------------------------------------------------------------------------
// StoppedState
// -----------------------------------------------------------------------------

void WheelController::StoppedState::onEnter() {
    // Stopped is a logical active state with no motor command. The controller
    // reports zero velocity and does not retain a feed-forward/PID command.
    controller.stopMotor();
    controller.current_velocity_ = 0.0f;
    controller.feedforward_pwm_ = 0.0f;
    controller.resetPid();
    controller.updatePidOutputLimits();
}

// -----------------------------------------------------------------------------
// NormalState
// -----------------------------------------------------------------------------

void WheelController::NormalState::prepareMotionStart() noexcept {
    // The first FG interval after a stop/reversal may contain time spent
    // stationary or braking. Synchronize the estimator to the latest edge and
    // discard that interval so the first reported sample represents the new
    // motion segment.
    controller.current_velocity_ = 0.0f;
    controller.velocity_estimator_.reset(controller.encoder_.getLastEdgeTimeUs(), true);
    controller.resetPid();
}

auto WheelController::NormalState::update(const uint32_t) noexcept -> TransitionRequest {
    // Convert the latest complete FG falling-edge period into the filtered wheel
    // velocity estimate, then run PID and command feed-forward + PID PWM.
    controller.current_velocity_ =
        controller.velocity_estimator_.update(controller.encoder_.getEdgeData());

    controller.pid_.Compute();
    controller.motor_.setPwmSpeed(calculateMotorSpeedPwm());

    return TransitionRequest::None;
}

int WheelController::NormalState::calculateMotorSpeedPwm() const noexcept {
    // PID correction is limited relative to the feed-forward term, but clamp
    // once more at the physical actuator limits as a final safety boundary.
    float combined_pwm =
        std::clamp(controller.feedforward_pwm_ + controller.pid_correction_pwm_, kMinPwm, kMaxPwm);

    // Feedback may reduce the requested command but must never independently
    // reverse the motor. A direction change must pass through Brake.
    if (controller.target_velocity_ > 0.0f) {
        combined_pwm = std::max(0.0f, combined_pwm);
    } else if (controller.target_velocity_ < 0.0f) {
        combined_pwm = std::min(0.0f, combined_pwm);
    } else {
        combined_pwm = 0.0f;
    }

    return std::lround(combined_pwm);
}

// -----------------------------------------------------------------------------
// BrakeState
// -----------------------------------------------------------------------------

auto WheelController::BrakeState::update(const uint32_t dt_ms) noexcept -> TransitionRequest {
    constexpr float kVelocityThresholdRadSec{2.0f};
    constexpr uint32_t kMaxBrakePeriodMs{50U};

    // Continue consuming FG feedback while the low brake PWM keeps the driver
    // active enough to generate timing information.
    controller.current_velocity_ =
        controller.velocity_estimator_.update(controller.encoder_.getEdgeData());

    // Completion is declared either when velocity is sufficiently low or when
    // the bounded brake interval expires.
    if (std::fabs(controller.current_velocity_) < kVelocityThresholdRadSec) {
        return finishBrake();
    }

    // Maintain the existing motion direction during braking. Reversal is only
    // allowed after Brake completes and a new NormalState is entered.
    controller.motor_.setPwmSpeed(brake_pwm_);

    total_period_ms_ += dt_ms;
    if (total_period_ms_ >= kMaxBrakePeriodMs) {
        return finishBrake();
    }

    return TransitionRequest::None;
}

auto WheelController::BrakeState::finishBrake() noexcept -> TransitionRequest {
    if (controller.target_velocity_ == 0.0f) {
        // StoppedState::onEnter() will keep the actuator stopped and clear the
        // reported velocity/feed-forward state.
        return TransitionRequest::ToStopped;
    }

    // A nonzero target means the latest requested command should start after
    // braking. Disable the old-direction brake command before NormalState's
    // onEnter() resynchronizes the estimator and resets PID for the new segment.
    controller.stopMotor();
    return TransitionRequest::ToNormal;
}

// -----------------------------------------------------------------------------
// WheelController
// -----------------------------------------------------------------------------

WheelController::WheelController(const uint8_t direction_pin,
                                 const uint8_t speed_control_pin,
                                 const uint8_t speed_state_pin,
                                 const float ticks_per_rev,
                                 const bool invert_logic)
    : motor_(direction_pin, speed_control_pin, invert_logic),
      encoder_(direction_pin, speed_state_pin, invert_logic),
      velocity_estimator_(ticks_per_rev),
      // QuickPID produces only the feedback correction. The feed-forward term is
      // calculated independently and added immediately before commanding PWM.
      pid_(&current_velocity_, &pid_correction_pwm_, &target_velocity_),
      state_(std::in_place_type<InactiveState>, *this) {}

void WheelController::configure(const uint32_t control_period_ms, const WheelConfig& wheel_config) {
    // Configure the estimator for the controller's nominal update period.
    velocity_estimator_.configure(control_period_ms);

    // Load the calibrated direction-dependent static/friction terms and shared
    // velocity slope used to construct the feed-forward PWM command.
    feedforward_ks_forward_ = wheel_config.feedforward_ks_forward;
    feedforward_ks_reverse_ = wheel_config.feedforward_ks_reverse;
    feedforward_kv_ = wheel_config.feedforward_kv;

    // This limit applies only to PID correction. Feed-forward retains access to
    // the complete actuator range, subject to the final PWM saturation.
    max_pid_correction_pwm_ = wheel_config.max_feedback_pwm;

    pid_.SetTunings(wheel_config.feedback_kp, wheel_config.feedback_ki, wheel_config.feedback_kd);
    pid_.SetSampleTimeUs(control_period_ms * 1000U);

    // Clamp the integral/output contribution to the currently configured output
    // range instead of allowing it to accumulate outside available PWM headroom.
    pid_.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);

    // Recompute the feed-forward term in case a target was set before configure.
    feedforward_pwm_ = calculateFeedForwardPwm();
    updatePidOutputLimits();
    resetPid();

    pid_.SetMode(QuickPID::Control::automatic);
}

void WheelController::begin() {
    // The motor must initialize its direction output before the encoder enables
    // PCNT direction tracking based on that shared signal.
    motor_.begin();
    encoder_.begin();
}

void WheelController::setActive(const bool active) {
    if (active) {
        dispatch(ActivateEvent{});
    } else {
        dispatch(DeactivateEvent{});
    }
}

void WheelController::update(const uint32_t dt_ms) {
    // State update handlers only return a transition request. The replacement
    // state is constructed after std::visit() has returned.
    dispatchUpdate(dt_ms);
}

void WheelController::setTargetVelocity(float target) {
    constexpr float kTargetZeroEpsilon{0.05F};

    // Treat very small numerical commands as an intentional zero command so a
    // tiny floating-point residual cannot keep the motor in motion.
    if (std::fabs(target) < kTargetZeroEpsilon) {
        target = 0.0f;
    }

    // Avoid unnecessary recalculation and state dispatch when nothing changed.
    if (target == target_velocity_) {
        return;
    }

    const bool stop = target == 0.0f;

    //  A requested sign change is handled as a state transition, not as an
    //  immediate motor-direction change. This is important because BLDC2430 FG
    //  direction is inferred from the direction command and because the motor
    //  should be slowed before reversing.
    const bool reversal =
        !stop && target_velocity_ != 0.0f && std::signbit(target) != std::signbit(target_velocity_);

    // Store the newest target before dispatching the event. This is important
    // when a target changes while Brake is active: Brake intentionally ignores
    // the event itself but uses this latest value when braking completes.
    target_velocity_ = target;

    // Update feed-forward immediately so the newest target is ready when Normal
    // resumes after braking. PID correction limits depend on this PWM bias.
    feedforward_pwm_ = calculateFeedForwardPwm();
    updatePidOutputLimits();

    dispatch(TargetVelocityEvent{stop, reversal});
}

void WheelController::updatePidOutputLimits() {
    // PID correction is bounded both by configured feedback authority and by
    // the remaining actuator headroom after applying feed-forward.
    const float actuator_minimum_correction = kMinPwm - feedforward_pwm_;
    const float actuator_maximum_correction = kMaxPwm - feedforward_pwm_;

    // Apply both the configured feedback-authority limit and physical actuator
    // headroom. At saturated feed-forward, PID is allowed only to reduce PWM.
    const float min_pid = std::max(-max_pid_correction_pwm_, actuator_minimum_correction);
    const float max_pid = std::min(max_pid_correction_pwm_, actuator_maximum_correction);

    pid_.SetOutputLimits(min_pid, max_pid);
}

void WheelController::resetPid() {
    // Clear integral/derivative history and explicitly remove the previous
    // correction so no feedback contribution leaks across state boundaries.
    pid_.Reset();
    pid_.SetOutputSum(0.0f);
    pid_correction_pwm_ = 0.0f;
}

float WheelController::calculateFeedForwardPwm() const {
    if (target_velocity_ == 0.0f) {
        return 0.0f;
    }

    const float ks = target_velocity_ > 0.0f ? feedforward_ks_forward_ : feedforward_ks_reverse_;

    const float magnitude =
        std::clamp(ks + feedforward_kv_ * std::fabs(target_velocity_), 0.0f, kMaxPwm);

    return std::copysign(magnitude, target_velocity_);
}

void WheelController::processTransition(const TransitionRequest transition) {
    // This function is invoked only after std::visit() has returned, so the
    // previous state object is no longer executing any member function when
    // emplace() destroys it.
    switch (transition) {
        case TransitionRequest::ToInactive:
            state_.emplace<InactiveState>(*this);
            break;

        case TransitionRequest::ToStopped:
            state_.emplace<StoppedState>(*this);
            break;

        case TransitionRequest::ToNormal:
            state_.emplace<NormalState>(*this);
            break;

        case TransitionRequest::ToBrake:
            state_.emplace<BrakeState>(*this);
            break;

        case TransitionRequest::None:
            [[fallthrough]];
        default:
            break;
    }
}