#ifndef WHEEL_CONTROLLER_HPP
#define WHEEL_CONTROLLER_HPP

#include <PID_v1.h>

#include "quadrature_encoder.hpp"
#include "l298n_motor.hpp"
#include "protocol/diff_drive_data.hpp"

class WheelController {
public:
    WheelController(const L298NMotor& motor,
                    const QuadratureEncoder& encoder,
                    const double pid_control_rate,
                    const WheelConfig& wheel_config,
                    const double ticks_per_rev,
                    bool invert_logic = false);

    ~WheelController() = default;

    WheelController(const WheelController&) = delete;
    WheelController(WheelController&&) = delete;
    WheelController& operator=(const WheelController&) = delete;
    WheelController& operator=(WheelController&&) = delete;

    void configure(const double pid_control_rate,
                   const WheelConfig& wheel_config);

    void begin();

    void setActive(bool active = true);

    void update();

    void reset() {
        // Reset the PID's internal state
        pid_.SetMode(MANUAL);
        pid_.SetMode(AUTOMATIC);
    }

    QuadratureEncoder& encoder() {
        return encoder_;
    }

    void setTargetVelocity(double rad_per_sec) {
        target_velocity_ = rad_per_sec;
    }

    double getCurrentVelocity() const {
        return current_velocity_;
    }


private:
    static constexpr double kMaxPwm{255};
    static constexpr double kMinPwm{-255};

    L298NMotor motor_;
    QuadratureEncoder encoder_;
    const double ticks_per_rev_;
    const bool invert_logic_;

    double control_loop_period_ms_;

    double target_velocity_{0.0};
    double current_velocity_{0.0};
    double pwm_cmd_{0.0};
    double pwm_deadband_{0.0}; 
    
    unsigned long last_time_ms_{0};
    long last_ticks_{0};

    bool is_active_{false};

    PID pid_;
};

#endif // WHEEL_CONTROLLER_HPP