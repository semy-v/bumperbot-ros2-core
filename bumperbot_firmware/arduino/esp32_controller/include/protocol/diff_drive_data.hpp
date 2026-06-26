#ifndef DIFF_DRIVE_DATA_HPP
#define DIFF_DRIVE_DATA_HPP


#pragma pack(push, 1)

struct WheelConfig {
    double kp;
    double ki;
    double kd;
    uint8_t pwm_deadband;

    bool operator<=>(const WheelConfig&) const = default;
};

struct ConfigData {
    double pid_rate;
    WheelConfig r_wheel;
    WheelConfig l_wheel;

    bool operator<=>(const ConfigData&) const = default;
};

struct VelocityData {
    double right_wheel_velocity;
    double left_wheel_velocity;
};

#pragma pack(pop)


static_assert(sizeof(ConfigData) == 58, "ConfigData size mismatch!");
static_assert(sizeof(VelocityData) == 16, "VelocityData size mismatch!");


#endif // DIFF_DRIVE_DATA_HPP
