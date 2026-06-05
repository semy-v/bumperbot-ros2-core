#include "wheel_controller.hpp"

// #define DEAD_BAND_CALIBRATION 23

WheelController::WheelController(const L298NMotor& motor,
                    const QuadratureEncoder& encoder,
                    const double pid_control_rate,
                    const WheelConfig& wheel_config,
                    const double ticks_per_rev,
                    bool invert_logic = false)
  : motor_(motor)
  , encoder_(encoder)
  , ticks_per_rev_(ticks_per_rev)
  , invert_logic_(invert_logic)
  , pid_(&current_velocity_, &pwm_cmd_, &target_velocity_,
          wheel_config.kp, wheel_config.ki, wheel_config.kd, DIRECT) 
{
    configure(pid_control_rate, wheel_config);
}

void WheelController::configure(const double pid_control_rate,
                const WheelConfig& wheel_config)
{
    reset();

    // PID calculates PWM commands commands
    // forward (positive) and reverse (negative)
    pid_.SetOutputLimits(kMinPwm, kMaxPwm);

    pid_.SetTunings(wheel_config.kp, wheel_config.ki, wheel_config.kd);

    control_loop_period_ms_ = 1000.0 / pid_control_rate;

    pwm_deadband_ = static_cast<double>(wheel_config.pwm_deadband);

    // Safety check to ensure 0 not passed to the PID library
    const int sample_time = max(1, control_loop_period_ms_);
    pid_.SetSampleTime(sample_time);
}

void WheelController::begin() {
    encoder_.begin();
    motor_.begin();
    last_time_ms_ = millis();
    last_ticks_ = encoder_.read();
    if (invert_logic_) last_ticks_ = -last_ticks_;
}

void WheelController::setActive(bool active = true) {
    if (is_active_ == active) {
        return; 
    }

    is_active_ = active;
    if (!is_active_) {
        // Clear target velocity
        target_velocity_ = 0.0;
        
        // Stop motor. 
        pwm_cmd_ = 0.0;
        motor_.setPWM(0); 

        // PID internal memory so it doesn't wind up while deactivated
        reset(); 
    }
}

void WheelController::update() {
    const unsigned long current_time_ms = millis();
    const unsigned long dt_ms = current_time_ms - last_time_ms_;

    // Prevent unnecessary updates if called too rapidly
    if (dt_ms < control_loop_period_ms_ || 0 == dt_ms) {
        return;
    }

    long current_ticks = encoder_.read();
    if (invert_logic_) {
        current_ticks = -current_ticks;
    }

    const long delta_ticks = current_ticks - last_ticks_;

    // kinematic calculation using actual dt
    const double delta_revs = static_cast<double>(delta_ticks) / ticks_per_rev_;
    const double delta_rads = delta_revs * 2.0 * PI;
    const double dt_sec = static_cast<double>(dt_ms) / 1000.0;

    current_velocity_ = delta_rads / dt_sec;

    // Skip PID and motor updates while deactivated
    if (!is_active_) {
        last_ticks_ = current_ticks;
        last_time_ms_ = current_time_ms;
        return; 
    }

    // Zero-Velocity Clamp
    if (target_velocity_ == 0.0) {
        // Force the motor command to exactly 0
        pwm_cmd_ = 0.0;

        // Reset the PID's internal Integral (I) memory 
        // so it doesn't wind up while sitting still.
        reset();
    } else {
        // Normal bidirectional PID operation
        pid_.Compute();

        // Wheel motor deadband compensation
        if (pwm_cmd_ > 0.0) {
            pwm_cmd_ += pwm_deadband_;
        } else if (pwm_cmd_ < 0.0) {
            pwm_cmd_ -= pwm_deadband_;
        }

        // Clamp to limits after adding deadband
        pwm_cmd_ = constrain(pwm_cmd_, kMinPwm, kMaxPwm);
    }

#ifndef DEAD_BAND_CALIBRATION
    motor_.setPWM(static_cast<int16_t>(pwm_cmd_));
#else // DEAD_BAND_CALIBRATION
    if (target_velocity_ != 0.0) {
        motor_.setPWM(
            target_velocity_ > 0.0 ? DEAD_BAND_CALIBRATION : -DEAD_BAND_CALIBRATION);
    } else {
        motor_.setPWM(0);
    }
#endif // DEAD_BAND_CALIBRATION

    last_ticks_ = current_ticks;
    last_time_ms_ = current_time_ms;
}