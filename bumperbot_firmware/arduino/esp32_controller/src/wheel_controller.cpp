#include "wheel_controller.hpp"

// #define DEAD_BAND_CALIBRATION 120

WheelController::WheelController(const L298NMotor& motor,
                    const QuadratureEncoder& encoder,
                    const double pid_control_rate,
                    const WheelConfig& wheel_config,
                    const double ticks_per_rev,
                    bool invert_logic)
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

    pid_.SetTunings(wheel_config.kp, wheel_config.ki, wheel_config.kd);
    pwm_deadband_ = static_cast<double>(wheel_config.pwm_deadband);

    // PID calculates PWM commands commands
    // forward (positive) and reverse (negative)
    pid_.SetOutputLimits(kMinPwm, kMaxPwm);

    // Safety check to ensure 0 not passed to the PID library
    const int sample_time = max(1.0, 1000.0 / pid_control_rate);
    pid_.SetSampleTime(sample_time);
}

void WheelController::begin() {
    encoder_.begin();
    motor_.begin();
    last_ticks_ = encoder_.read();
    if (invert_logic_) {
        last_ticks_ = -last_ticks_;
    }
}

void WheelController::setActive(bool active) {
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

        // reset PID internal memory so
        // it doesn't wind up while deactivated
        reset(); 
    }
}

void WheelController::update(const double dt_sec) {
    long current_ticks = encoder_.read();
    if (invert_logic_) {
        current_ticks = -current_ticks;
    }

    // kinematic calculation dt_sec
    const long delta_ticks = current_ticks - last_ticks_;
    const double delta_revs = static_cast<double>(delta_ticks) / ticks_per_rev_;
    const double delta_rads = delta_revs * 2.0 * PI;
    current_velocity_ = delta_rads / dt_sec;

    // Skip PID and motor updates while deactivated
    if (!is_active_) {
        last_ticks_ = current_ticks;
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
}