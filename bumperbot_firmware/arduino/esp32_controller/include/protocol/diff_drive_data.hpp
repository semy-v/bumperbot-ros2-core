#ifndef DIFF_DRIVE_DATA_HPP
#define DIFF_DRIVE_DATA_HPP

#include <cstdint>

#pragma pack(push, 1)

struct WheelConfig {
    double kp;
    double ki;
    double kd;
    uint8_t pwm_deadband;

    bool operator<=>(const WheelConfig&) const = default;
};

struct DiffDriveConfigData {
    double pid_rate;
    WheelConfig r_wheel;
    WheelConfig l_wheel;

    bool operator<=>(const DiffDriveConfigData&) const = default;
};

struct ImuConfigData {
    uint16_t calibrate_period_ms;
    bool result;
};

struct DiffDriveVelocityData {
    float right_wheel_velocity;
    float left_wheel_velocity;
};

struct DiffDriveCommandData {
    DiffDriveVelocityData velocity;
    uint8_t response_delay_ms;
};

struct DiffDriveStateData {
    DiffDriveVelocityData velocity;
};

#pragma pack(pop)


static_assert(sizeof(DiffDriveConfigData) == 58, "DiffDriveConfigData size mismatch!");
static_assert(sizeof(ImuConfigData) == 3, "ImuConfigData size mismatch!");
static_assert(sizeof(DiffDriveCommandData) == 9, "DiffDriveCommandData size mismatch!");
static_assert(sizeof(DiffDriveStateData) == 8, "DiffDriveStateData size mismatch!");

#endif // DIFF_DRIVE_DATA_HPP
