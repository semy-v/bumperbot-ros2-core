#ifndef DIFF_DRIVE_DATA_HPP
#define DIFF_DRIVE_DATA_HPP


#pragma pack(push, 1)

struct WheelConfig {
    float kp;
    float ki;
    float kd;
    uint8_t pwm_deadband;

#if defined(__cpp_impl_three_way_comparison)
    bool operator==(const WheelConfig&) const = default;
#endif
};

struct ConfigData {
    float pid_rate;
    WheelConfig r_wheel;
    WheelConfig l_wheel;

// default comparison operator if supported by compiler
#if defined(__cpp_impl_three_way_comparison)
    bool operator==(const ConfigData&) const = default;
#endif
};

struct VelocityData {
    float right_wheel_velocity;
    float left_wheel_velocity;
};

#pragma pack(pop)


static_assert(sizeof(ConfigData) == 30, "ConfigData size mismatch!");
static_assert(sizeof(VelocityData) == 8, "VelocityData size mismatch!");


#endif // DIFF_DRIVE_DATA_HPP
