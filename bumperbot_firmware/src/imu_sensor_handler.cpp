
#include <limits>

#include "bumperbot_firmware/imu_sensor_handler.hpp"

namespace bumperbot_firmware {

void ImuSensorHandler::updateFromState(const ImuStateData& imu_data, const bool robot_stationary) {
    imu_data_.linear_acceleration_x = imu_data.linear_acceleration_x;
    imu_data_.linear_acceleration_y = imu_data.linear_acceleration_y;
    imu_data_.linear_acceleration_z = imu_data.linear_acceleration_z;

    imu_data_.angular_velocity_x = imu_data.angular_velocity_x;
    imu_data_.angular_velocity_y = imu_data.angular_velocity_y;

    constexpr double kGyroZDeadbandRadPerSec{6.0e-3}; // Deadband threshold for angular velocity around Z-axis (rad/s)
    if (robot_stationary && std::fabs(imu_data.angular_velocity_z) <= kGyroZDeadbandRadPerSec) {
        imu_data_.angular_velocity_z = 0.0; // Set to zero if within deadband and robot is stationary
        return;
    }

    imu_data_.angular_velocity_z = imu_data.angular_velocity_z;
}

void ImuSensorHandler::setAvailability(bool available, const rclcpp::Logger& logger) {
    if (data_available_ != available) {
        data_available_ = available;
        RCLCPP_WARN(logger, "IMU sensor data is now %s. Toggling interface availability.",
                    available ? "AVAILABLE" : "UNAVAILABLE (Degraded Mode)");
    }

    if (!data_available_) {
        // Use Quiet NaN to invalidate the data
        constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

        imu_data_.angular_velocity_x = kNan;
        imu_data_.angular_velocity_y = kNan;
        imu_data_.angular_velocity_z = kNan;

        imu_data_.linear_acceleration_x = kNan;
        imu_data_.linear_acceleration_y = kNan;
        imu_data_.linear_acceleration_z = kNan;
    }
}

ImuConfigData ImuSensorHandler::getDefaultConfig(uint16_t calibration_period_ms) const {
    return ImuConfigData{.calibrate_period_ms = calibration_period_ms, .result = true};
}

std::span<hardware_interface::StateInterface::ConstSharedPtr>
ImuSensorHandler::exportStateInterfaces() {
    state_interfaces_.clear();
    state_interfaces_.reserve(kStateInterfaceNum);
    auto add_interface = [this](const std::string& name, double& val) {
        state_interfaces_.push_back(
            std::make_shared<hardware_interface::StateInterface>("imu_sensor", name, &val));
    };

    add_interface("angular_velocity.x", imu_data_.angular_velocity_x);
    add_interface("angular_velocity.y", imu_data_.angular_velocity_y);
    add_interface("angular_velocity.z", imu_data_.angular_velocity_z);
    add_interface("linear_acceleration.x", imu_data_.linear_acceleration_x);
    add_interface("linear_acceleration.y", imu_data_.linear_acceleration_y);
    add_interface("linear_acceleration.z", imu_data_.linear_acceleration_z);
    add_interface("orientation.x", imu_data_.orientation_x);
    add_interface("orientation.y", imu_data_.orientation_y);
    add_interface("orientation.z", imu_data_.orientation_z);
    add_interface("orientation.w", imu_data_.orientation_w);

    return state_interfaces_;
}

}  // namespace bumperbot_firmware