#include <algorithm>
#include <cstdint>

#include "diff_drive/bl2418_motor.hpp"

BL2418Motor::BL2418Motor(uint8_t direction_pin, uint8_t speed_control_pin, bool invert_logic)
    : direction_pin_(direction_pin),
      speed_control_pin_(speed_control_pin),
      speed_channel_{next_channel_++},
      forward_direction_value_{static_cast<uint8_t>(invert_logic ? LOW : HIGH)},
      reverse_direction_value_{static_cast<uint8_t>(invert_logic ? HIGH : LOW)} {
    assert(speed_channel_ < SOC_LEDC_CHANNEL_NUM);
}

void BL2418Motor::begin() {
    pinMode(direction_pin_, OUTPUT);

    // Configure the default logical forward direction before enabling
    // the PWM output to avoid an unintended direction change during
    // initialization.
    digitalWrite(direction_pin_, forward_direction_value_);

    attachSpeedPwm();
    writeSpeedPwm(kMotorOffDuty);
}

void BL2418Motor::setPwmSpeed(int speed) {
    if (speed >= 0) {
        digitalWrite(direction_pin_, forward_direction_value_);
    } else {
        digitalWrite(direction_pin_, reverse_direction_value_);
        speed = -speed;
    }

    speed = std::clamp(speed, kMinPwmSpeed, kMaxPwmSpeed);
    // Convert the requested speed into the active-low PWM duty
    const uint8_t duty = kMotorOffDuty - static_cast<uint8_t>(speed);
    writeSpeedPwm(duty);
}