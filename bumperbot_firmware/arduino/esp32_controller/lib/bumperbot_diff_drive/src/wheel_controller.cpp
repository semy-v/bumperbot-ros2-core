#include "diff_drive/wheel_controller.hpp"

#include <algorithm>
#include <cmath>

WheelController::WheelController(uint8_t direction_pin,
                                 uint8_t speed_control_pin,
                                 uint8_t speed_state_pin,
                                 float ticks_per_rev,
                                 bool invert_logic)
    : motor_(direction_pin, speed_control_pin, invert_logic),
      encoder_(direction_pin, speed_state_pin, invert_logic),
      velocity_estimator_(ticks_per_rev),
      // QuickPID produces only the feedback correction. The feed-forward term is
      // calculated independently and added immediately before commanding PWM.
      pid_(&current_velocity_, &pid_correction_pwm_, &target_velocity_) {}

void WheelController::configure(const uint32_t control_period_ms, const WheelConfig& wheel_config) {
    // Configure reciprocal-period filtering for the nominal controller period.
    velocity_estimator_.configure(control_period_ms);

    // Directional feed-forward coefficients identified from moving-state
    // calibration. Ks captures direction-dependent friction; Kv is shared.
    feedforward_ks_forward_ = wheel_config.feedforward_ks_forward;
    feedforward_ks_reverse_ = wheel_config.feedforward_ks_reverse;
    feedforward_kv_ = wheel_config.feedforward_kv;

    // This limits feedback authority only; it does not limit feed-forward PWM.
    max_pid_correction_pwm_ = wheel_config.max_feedback_pwm;

    pid_.SetTunings(wheel_config.feedback_kp, wheel_config.feedback_ki, wheel_config.feedback_kd);
    pid_.SetSampleTimeUs(control_period_ms * 1000U);

    // Clamp the integral/output contribution to the currently configured output
    // range instead of allowing it to accumulate outside available PWM headroom.
    pid_.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);

    // calculateFeedForwardPwm() depends on the current target, which may already
    // have been set before reconfiguration.
    feedforward_pwm_ = calculateFeedForwardPwm();
    updatePidOutputLimits();
    resetPid();

    pid_.SetMode(QuickPID::Control::automatic);
}

void WheelController::begin() {
    /*
     * The BLDC2430Encoder uses the motor direction signal as the PCNT control
     * input. Initialize the motor first so that direction_pin has a deterministic
     * logic level before the encoder enables PCNT event processing.
     */
    motor_.begin();
    encoder_.begin();
}

void WheelController::setActive(bool active) {
    if (active) {
        if (state_ == ControlState::Inactive) {
            // Activation does not invent a target. Resume according to the most
            // recently stored target velocity.
            if (target_velocity_ == 0.0f) {
                enterStopped();
            } else {
                prepareMotionStart();
            }
        }
        return;
    }

    if (state_ == ControlState::Inactive) {
        return;
    }

    // Deactivation is an unconditional software stop: discard both command and
    // controller history so stale PID/estimator state cannot affect reactivation.
    target_velocity_ = 0.0f;
    stopMotor();
    resetPid();
    velocity_estimator_.reset();
    current_velocity_ = 0.0f;
    state_ = ControlState::Inactive;
}

void WheelController::setTargetVelocity(float target) {
    // Avoid repeatedly applying direction-specific static feed-forward for tiny
    // numerical commands that are intended to represent zero.
    constexpr float kTargetZeroEpsilon{0.05F};

    if (std::fabs(target) < kTargetZeroEpsilon) {
        target = 0.0f;
    }

    if (target == target_velocity_) {
        return;
    }

    const bool stop = target == 0.0f;

    /*
     * A requested sign change is handled as a state transition, not as an
     * immediate motor-direction change. This is important because BLDC2430 FG
     * direction is inferred from the direction command and because the motor
     * should be slowed before reversing.
     */
    const bool reversal =
        !stop && target_velocity_ != 0.0f && std::signbit(target) != std::signbit(target_velocity_);

    target_velocity_ = target;

    // Update feed-forward immediately so the newest target is ready when Normal
    // resumes after braking. PID correction limits depend on this PWM bias.
    feedforward_pwm_ = calculateFeedForwardPwm();
    updatePidOutputLimits();

    switch (state_) {
        case ControlState::Inactive:
            // Store the target only. setActive(true) decides whether motion starts.
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
            // Same-direction speed changes remain in Normal; feed-forward and
            // setpoint update immediately while PID history is preserved.
            break;

        case ControlState::Brake:
            /*
             * Do not interrupt the deceleration sequence. The latest target is
             * retained; completeBraking() will either enter Stopped (target=0)
             * or prepare a new nonzero motion start.
             */
            break;
    }
}

void WheelController::updatePidOutputLimits() {
    /*
     * QuickPID output is a correction around feedforward_pwm_, not an absolute
     * motor command. First determine the correction range still available before
     * the combined actuator command would exceed +/-255.
     */
    const float actuator_minimum_correction = kMinPwm - feedforward_pwm_;
    const float actuator_maximum_correction = kMaxPwm - feedforward_pwm_;

    // Apply both the configured feedback-authority limit and physical actuator
    // headroom. At saturated feed-forward, PID is allowed only to reduce PWM.
    const float min_pid = std::max(-max_pid_correction_pwm_, actuator_minimum_correction);
    const float max_pid = std::min(max_pid_correction_pwm_, actuator_maximum_correction);

    pid_.SetOutputLimits(min_pid, max_pid);
}

void WheelController::resetPid() {
    // Clear integral/derivative history and explicitly remove the last correction
    // so no feedback term survives a stop, brake transition, or reconfiguration.
    pid_.Reset();
    pid_.SetOutputSum(0.0f);
    pid_correction_pwm_ = 0.0f;
}

void WheelController::update(uint32_t dt_ms) {
    switch (state_) {
        case ControlState::Inactive:
        case ControlState::Stopped:
            /*
             * These states intentionally report zero controller velocity and do
             * not consume encoder measurements. prepareMotionStart() later
             * resynchronizes the estimator to the latest edge timestamp.
             */
            current_velocity_ = 0.0f;
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
    // Convert the latest complete FG falling-edge period into filtered rad/s.
    current_velocity_ = velocity_estimator_.update(encoder_.getEdgeData());

    // QuickPID may internally honor its configured sample-time schedule; the
    // stored pid_correction_pwm_ remains the most recent correction otherwise.
    pid_.Compute();

    // Feed-forward supplies the nominal operating PWM; PID corrects only the
    // remaining model/load error.
    motor_.setPwmSpeed(calculateMotorSpeedPwm());
}

bool WheelController::updateBrake(const uint32_t dt_ms) {
    /*
     * Brake completion is currently heuristic: either measured speed falls below
     * this threshold or the bounded brake interval expires.
     *
     * IMPORTANT: 2 rad/s is not "physically stopped". For a direction reversal,
     * these constants must be validated against the installed BLDC2430 mechanics
     * because completing Brake can allow the opposite direction to be commanded.
     */
    constexpr float kVelocityThresholdRadSec{2.0f};
    constexpr uint32_t kMaxBrakePeriodMs{50U};

    // Keep consuming FG periods while the low brake PWM still allows the driver
    // to produce speed feedback.
    current_velocity_ = velocity_estimator_.update(encoder_.getEdgeData());

    if (std::fabs(current_velocity_) < kVelocityThresholdRadSec) {
        return true;
    }

    // Command a low speed in the existing motion direction. This decelerates
    // without intentionally reversing the direction line during Brake.
    motor_.setPwmSpeed(brake_pwm_);

    brake_total_period_ms_ += dt_ms;
    return brake_total_period_ms_ >= kMaxBrakePeriodMs;
}

void WheelController::prepareMotionStart() {
    /*
     * The first FG period after a stop/reversal can span time during which the
     * wheel was stationary or braking. Synchronize to the latest encoder edge
     * and discard that first future interval; the following falling-edge period
     * is the first valid reciprocal-period velocity sample.
     */
    current_velocity_ = 0.0f;
    velocity_estimator_.reset(encoder_.getLastEdgeTimeUs(), true);

    // Start every new motion segment without integral memory from the previous
    // operating point or direction.
    resetPid();
    state_ = ControlState::Normal;
}

void WheelController::beginBraking() {
    /*
     * Capture the brake direction before target-driven motor commands can change
     * direction. Under normal operation current_velocity_ still reflects the
     * previous physical/commanded motion direction.
     */
    brake_pwm_ = calculateBrakePwm();
    brake_total_period_ms_ = 0U;

    // Feedback is not used in Brake; remove PID history before the next segment.
    resetPid();
    state_ = ControlState::Brake;
}

void WheelController::completeBraking() {
    if (target_velocity_ == 0.0f) {
        enterStopped();
        return;
    }

    /*
     * A nonzero target means either a reversal or a new command received while
     * braking. Disable drive first, then prepare a fresh estimator/PID segment.
     * The actual target-direction PWM is issued by the next Normal update.
     */
    stopMotor();
    prepareMotionStart();
}

void WheelController::enterStopped() {
    stopMotor();

    // Stopped is a logical controller state: no command, no retained velocity,
    // no feed-forward bias, and no PID history.
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

    // Calibration identified direction-dependent static/friction intercepts but
    // a common velocity slope for the BLDC2430 wheel assembly.
    const float ks = target_velocity_ > 0.0f ? feedforward_ks_forward_ : feedforward_ks_reverse_;

    const float magnitude =
        std::clamp(ks + feedforward_kv_ * std::fabs(target_velocity_), 0.0f, kMaxPwm);

    return std::copysign(magnitude, target_velocity_);
}

int WheelController::calculateMotorSpeedPwm() const {
    // The feedback term is already bounded relative to available feed-forward
    // headroom, but clamp again at the physical actuator boundary.
    float combined_pwm = std::clamp(feedforward_pwm_ + pid_correction_pwm_, kMinPwm, kMaxPwm);

    /*
     * PID feedback is not allowed to reverse the motor in Normal. If error is
     * large enough that correction would cross zero, command zero instead.
     * Direction changes must pass through the explicit Brake state.
     */
    if (target_velocity_ > 0.0f) {
        combined_pwm = std::max(0.0f, combined_pwm);
    } else if (target_velocity_ < 0.0f) {
        combined_pwm = std::min(0.0f, combined_pwm);
    } else {
        combined_pwm = 0.0f;
    }

    return std::lround(combined_pwm);
}