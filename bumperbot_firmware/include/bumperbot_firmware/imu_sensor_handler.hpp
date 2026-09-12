#ifndef IMU_SENSOR_HANDLER_HPP
#define IMU_SENSOR_HANDLER_HPP

#include <array>
#include <memory>
#include <span>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "protocol/system_data.hpp"

namespace bumperbot_firmware {

class ImuSensorHandler {
 public:
    struct ImuSensorData {
        double angular_velocity_x{0.0}, angular_velocity_y{0.0}, angular_velocity_z{0.0};
        double linear_acceleration_x{0.0}, linear_acceleration_y{0.0}, linear_acceleration_z{0.0};
        // orientation values will not be updated
        // keeping for compatibility with IMU broadcaster controller
        double orientation_x{0.0}, orientation_y{0.0}, orientation_z{0.0}, orientation_w{1.0};
    };

    ImuSensorHandler() = default;

    void updateFromState(const ImuStateData& imu_data, const bool robot_stationary = false);
    void setAvailability(bool available, const rclcpp::Logger& logger);
    bool isAvailable() const { return data_available_; }

    std::span<hardware_interface::StateInterface::ConstSharedPtr> exportStateInterfaces();
    ImuConfigData getDefaultConfig(uint16_t calibration_period_ms) const;

 private:
    constexpr static size_t kStateInterfaceNum{10};

    ImuSensorData imu_data_{};
    bool data_available_{true};
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces_;
};

}  // namespace bumperbot_firmware

#endif  // IMU_SENSOR_HANDLER_HPP