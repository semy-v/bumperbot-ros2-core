#ifndef WHEEL_CONTROLLER_HPP
#define WHEEL_CONTROLLER_HPP

#include <QuickPID.h>

#include "diff_drive/bl2418_encoder.hpp"
#include "diff_drive/bl2418_motor.hpp"
#include "diff_drive/wheel_velocity_estimator.hpp"
#include "protocol/system_data.hpp"

class WheelController {
 public:
    enum class ControlState : uint8_t { Inactive, Stopped, Normal, Brake };

    WheelController(uint8_t direction_pin,
                    uint8_t speed_control_pin,
                    uint8_t speed_state_pin,
                    float ticks_per_rev,
                    bool invert_logic = false);

    ~WheelController() = default;

    WheelController(const WheelController&) = delete;
    WheelController(WheelController&&) = delete;
    WheelController& operator=(const WheelController&) = delete;
    WheelController& operator=(WheelController&&) = delete;

    void configure(const uint32_t control_period_ms, const WheelConfig& wheel_config);

    void begin();

    void setActive(bool active = true);

    void update(const uint32_t dt_ms);

    void setTargetVelocity(float target);

    void updatePidOutputLimits();

    void resetPid();

    float getCurrentVelocity() const { return current_velocity_; }

 private:
    static constexpr float kMaxPwm{255.0F};
    static constexpr float kMinPwm{-255.0F};

    BL2418Motor motor_;
    BL2418Encoder encoder_;
    WheelVelocityEstimator velocity_estimator_;
    QuickPID pid_;

    ControlState state_{ControlState::Inactive};

    float target_velocity_{};
    float current_velocity_{};

    float max_pid_correction_pwm_{};
    float pid_correction_pwm_{};

    float feedforward_static_friction_gain_{};
    float feedforward_velocity_gain_{};
    float feedforward_pwm_{};

    uint32_t brake_total_period_ms_{};
    int brake_pwm_{};

    void updateNormal();
    bool updateBrake(const uint32_t dt_ms);

    void prepareMotionStart();
    void enterStopped();
    void completeBraking();
    void beginBraking();
    void stopMotor() { motor_.setPwmSpeed(0); }

    float calculateFeedForwardPwm() const;

    int calculateMotorSpeedPwm() const;

    int calculateBrakePwm() const {
        // the BL2418 motor driver brakes at the minimum PWM while still producing the FG encoder
        // pulses
        // in contrast to the 0.0 pwm where the motor driver stops producing any FG encoder pulses
        constexpr float kBrakeDeadBandPwm{15.0};
        return static_cast<int>(std::copysign(kBrakeDeadBandPwm, current_velocity_));
    }
};

#endif  // WHEEL_CONTROLLER_HPP