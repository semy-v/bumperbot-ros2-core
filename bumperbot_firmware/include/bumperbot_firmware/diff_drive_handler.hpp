#ifndef DIFF_DRIVE_HANDLER_HPP
#define DIFF_DRIVE_HANDLER_HPP

#include <array>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "protocol/system_data.hpp"

namespace bumperbot_firmware {

class DifferentialDriveHandler {
 public:
    DifferentialDriveHandler() = default;

    bool init(const hardware_interface::HardwareInfo& info, const rclcpp::Logger& logger);
    void resetStates();
    void integrateOdometry(double dt);
    void updateFromState(const DiffDriveStateData& state_data);
    DiffDriveCommandData getCommandData(uint8_t response_delay_ms);

    std::span<hardware_interface::StateInterface::ConstSharedPtr> exportStateInterfaces();
    std::span<hardware_interface::CommandInterface::SharedPtr> exportCommandInterfaces();

    const DiffDriveConfigData& getConfig() const { return config_data_; }
    double getMinVelocity() const { return wheels_min_velocity_; }

    constexpr bool isStationary() const {
        return std::abs(interface_data_[kLeftWheelIndex].velocity_state) < kZeroThreshold &&
               std::abs(interface_data_[kRightWheelIndex].velocity_state) < kZeroThreshold;
    }

 private:
    struct WheelData {
        double velocity_command{0.0};
        double velocity_state{0.0};
        double position_state{0.0};
    };

    bool validateWheelNames(const hardware_interface::HardwareInfo& info,
                            const rclcpp::Logger& logger) const;

    static constexpr std::array<std::string_view, 2> kWheelNames{"wheel_left_joint",
                                                                 "wheel_right_joint"};
    static constexpr size_t kLeftWheelIndex{0};
    static constexpr size_t kRightWheelIndex{1};
    static constexpr double kZeroThreshold{1e-4}; // A tiny epsilon threshold to account for floating-point noise

    double wheels_min_velocity_{0.0};
    DiffDriveConfigData config_data_{};
    std::array<WheelData, 2> interface_data_{};

    // Keep internal shared pointers to interfaces
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
};

}  // namespace bumperbot_firmware

#endif  // DIFF_DRIVE_HANDLER_HPP