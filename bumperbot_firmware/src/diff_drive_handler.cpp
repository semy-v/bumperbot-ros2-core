#include <algorithm>

#include "bumperbot_firmware/diff_drive_handler.hpp"
#include "bumperbot_firmware/hardware_interface_helpers.hpp"

namespace bumperbot_firmware {

[[nodiscard]] bool DifferentialDriveHandler::validateWheelNames(
    const hardware_interface::HardwareInfo& info,
    const rclcpp::Logger& logger) const {
    for (const auto& expected_name : kWheelNames) {
        const bool found = std::any_of(
            info.joints.begin(), info.joints.end(),
            [&expected_name](const auto& joint_info) { return joint_info.name == expected_name; });
        if (!found) {
            RCLCPP_FATAL(logger, "Wheel joint '%s' not found in URDF joints configuration!",
                         std::string(expected_name).c_str());
            return false;
        }
    }
    return true;
}

bool DifferentialDriveHandler::init(const hardware_interface::HardwareInfo& info,
                                    const rclcpp::Logger& logger) {
    if (!validateWheelNames(info, logger)) {
        return false;
    }

    wheels_min_velocity_ =
        getHwParam<double>(info.hardware_parameters, "wheels_min_velocity", 0.0, logger);
    config_data_.pid_rate =
        getHwParam<double>(info.hardware_parameters, "wheels_pid_rate", 50.0, logger);

    config_data_.r_wheel.kp =
        getHwParam<double>(info.hardware_parameters, "wheel_right_kp", 15.5, logger);
    config_data_.r_wheel.ki =
        getHwParam<double>(info.hardware_parameters, "wheel_right_ki", 39.0, logger);
    config_data_.r_wheel.kd =
        getHwParam<double>(info.hardware_parameters, "wheel_right_kd", 0.0, logger);
    config_data_.r_wheel.pwm_deadband =
        getHwParam<int>(info.hardware_parameters, "wheel_right_pwm_deadband", 17, logger);

    config_data_.l_wheel.kp =
        getHwParam<double>(info.hardware_parameters, "wheel_left_kp", 14.0, logger);
    config_data_.l_wheel.ki =
        getHwParam<double>(info.hardware_parameters, "wheel_left_ki", 43.0, logger);
    config_data_.l_wheel.kd =
        getHwParam<double>(info.hardware_parameters, "wheel_left_kd", 0.0, logger);
    config_data_.l_wheel.pwm_deadband =
        getHwParam<int>(info.hardware_parameters, "wheel_left_pwm_deadband", 18, logger);

    // Logging the parsed/fallback hardware parameters
    RCLCPP_INFO(logger, "Differential Drive parameters successfully loaded:");
    RCLCPP_INFO(logger, "  - Min velocity limit: %f rad/s", wheels_min_velocity_);
    RCLCPP_INFO(logger, "  - Wheels PID rate: %f Hz", config_data_.pid_rate);
    RCLCPP_INFO(logger, "  - Right wheel configs -> Kp: %.4f, Ki: %.4f, Kd: %.4f, Deadband: %u",
                config_data_.r_wheel.kp, config_data_.r_wheel.ki, config_data_.r_wheel.kd,
                config_data_.r_wheel.pwm_deadband);
    RCLCPP_INFO(logger, "  - Left wheel configs  -> Kp: %.4f, Ki: %.4f, Kd: %.4f, Deadband: %u",
                config_data_.l_wheel.kp, config_data_.l_wheel.ki, config_data_.l_wheel.kd,
                config_data_.l_wheel.pwm_deadband);

    return true;
}

void DifferentialDriveHandler::resetStates() {
    for (auto& wheel : interface_data_) {
        wheel = {0.0, 0.0, 0.0};
    }
}

void DifferentialDriveHandler::integrateOdometry(double dt) {
    interface_data_[kRightWheelIndex].position_state +=
        interface_data_[kRightWheelIndex].velocity_state * dt;
    interface_data_[kLeftWheelIndex].position_state +=
        interface_data_[kLeftWheelIndex].velocity_state * dt;
}

void DifferentialDriveHandler::updateFromState(const DiffDriveStateData& state_data) {
    interface_data_[kRightWheelIndex].velocity_state = state_data.velocity.right_wheel_velocity;
    interface_data_[kLeftWheelIndex].velocity_state = state_data.velocity.left_wheel_velocity;
}

DiffDriveCommandData DifferentialDriveHandler::getCommandData(uint8_t response_delay_ms) {
    DiffDriveCommandData command{
        .velocity =
            {
                .right_wheel_velocity =
                    static_cast<float>(interface_data_[kRightWheelIndex].velocity_command),
                .left_wheel_velocity =
                    static_cast<float>(interface_data_[kLeftWheelIndex].velocity_command),
            },
        .response_delay_ms = response_delay_ms};

    // A tiny epsilon threshold to account for floating-point noise
    constexpr double kZeroThreshold = 1e-4;
    auto clamp_vel = [this, kZeroThreshold](float& vel) {
        if (std::abs(vel) <= kZeroThreshold) {
            // Force a clean stop if the command is microscopic noise
            vel = 0.0f;
        } else if (std::abs(vel) < wheels_min_velocity_) {
            // Clamp up to minimum velocity if trying to move but under mechanical limits
            vel = std::copysign(wheels_min_velocity_, vel);
        }
    };

    clamp_vel(command.velocity.right_wheel_velocity);
    clamp_vel(command.velocity.left_wheel_velocity);

    return command;
}

std::span<hardware_interface::StateInterface::ConstSharedPtr>
DifferentialDriveHandler::exportStateInterfaces() {
    state_interfaces_.clear();
    for (size_t i = 0; i < kWheelNames.size(); i++) {
        state_interfaces_.push_back(std::make_shared<hardware_interface::StateInterface>(
            std::string(kWheelNames[i]), hardware_interface::HW_IF_POSITION,
            &interface_data_[i].position_state));
        state_interfaces_.push_back(std::make_shared<hardware_interface::StateInterface>(
            std::string(kWheelNames[i]), hardware_interface::HW_IF_VELOCITY,
            &interface_data_[i].velocity_state));
    }

    return state_interfaces_;
}

std::span<hardware_interface::CommandInterface::SharedPtr>
DifferentialDriveHandler::exportCommandInterfaces() {
    command_interfaces_.clear();
    for (size_t i = 0; i < kWheelNames.size(); i++) {
        command_interfaces_.push_back(std::make_shared<hardware_interface::CommandInterface>(
            std::string(kWheelNames[i]), hardware_interface::HW_IF_VELOCITY,
            &interface_data_[i].velocity_command));
    }

    return command_interfaces_;
}

}  // namespace bumperbot_firmware