#ifndef SYSTEM_DATA_HPP
#define SYSTEM_DATA_HPP

#include <cstdint>

#pragma pack(push, 1)

struct WheelConfig {
    // Static-friction feed-forward term [PWM].
    float feedforward_ks;

    // Velocity feed-forward gain [PWM / (rad/s)].
    float feedforward_kv;

    // Feedback PID gains
    float feedback_kp;
    float feedback_ki;
    float feedback_kd;

    // Maximum absolute PID feedback correction [PWM].
    uint16_t max_feedback_pwm;

    bool operator==(const WheelConfig&) const = default;

    bool valid() const {
        return max_feedback_pwm > 0 && max_feedback_pwm <= 255 && feedforward_ks >= 0.0f &&
               feedforward_kv >= 0.0f && feedback_kp >= 0.0f && feedback_ki >= 0.0f &&
               feedback_kd >= 0.0f;
    }
};

struct DiffDriveConfigData {
    WheelConfig right_wheel;
    WheelConfig left_wheel;
    uint16_t control_rate_hz;

    bool operator==(const DiffDriveConfigData&) const = default;

    bool valid() const {
        return right_wheel.valid() && left_wheel.valid() && control_rate_hz > 0;
    }
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

struct ImuStateData {
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;

    float linear_acceleration_x;
    float linear_acceleration_y;
    float linear_acceleration_z;
};

enum class SystemStateFlags : uint8_t {
    None = 0,
    ImuUnavailable = 1 << 0,
};

struct SystemStateData {
    SystemStateFlags status;
    ImuStateData imu;
    DiffDriveStateData diff_drive;
};

#pragma pack(pop)

static_assert(sizeof(WheelConfig) == 22, "WheelConfig size mismatch!");
static_assert(sizeof(DiffDriveConfigData) == 46, "DiffDriveConfigData size mismatch!");
static_assert(sizeof(ImuConfigData) == 3, "ImuConfigData size mismatch!");
static_assert(sizeof(DiffDriveCommandData) == 9, "DiffDriveCommandData size mismatch!");
static_assert(sizeof(DiffDriveStateData) == 8, "DiffDriveStateData size mismatch!");
static_assert(sizeof(ImuStateData) == 24, "DiffDriveStateData size mismatch!");
static_assert(sizeof(SystemStateData) == 33, "SystemStateData size mismatch");

#endif  // SYSTEM_DATA_HPP
