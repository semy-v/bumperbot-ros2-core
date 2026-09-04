#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

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
        getHwParam<double>(info.hardware_parameters, "wheels_min_velocity", 3.0, logger);

    // Parse integer parameters into signed temporary values before converting
    // them to uint16_t. This prevents negative values from wrapping.
    const int control_rate_hz =
        getHwParam<int>(info.hardware_parameters, "wheels_control_rate_hz", 200, logger);
    const int right_max_feedback_pwm =
        getHwParam<int>(info.hardware_parameters, "wheel_right_max_feedback_pwm", 50, logger);
    const int left_max_feedback_pwm =
        getHwParam<int>(info.hardware_parameters, "wheel_left_max_feedback_pwm", 50, logger);

    constexpr int kMaxControlRateHz = static_cast<int>(std::numeric_limits<uint16_t>::max());
    if (control_rate_hz <= 0 || control_rate_hz > kMaxControlRateHz) {
        RCLCPP_ERROR(logger, "Invalid control_rate_hz: %d; expected range [1, %d].",
                     control_rate_hz, kMaxControlRateHz);
        return false;
    }

    if (right_max_feedback_pwm <= 0 || right_max_feedback_pwm > 255 || left_max_feedback_pwm <= 0 ||
        left_max_feedback_pwm > 255) {
        RCLCPP_ERROR(logger,
                     "Invalid maximum feedback PWM: right=%d, left=%d; expected range [1, 255].",
                     right_max_feedback_pwm, left_max_feedback_pwm);
        return false;
    }

    config_data_.control_rate_hz = static_cast<uint16_t>(control_rate_hz);

    auto& right = config_data_.right_wheel;
    right.feedforward_ks_forward = getHwParam<float>(
        info.hardware_parameters, "wheel_right_feedforward_ks_forward", 17.34, logger);
    right.feedforward_ks_reverse = getHwParam<float>(
        info.hardware_parameters, "wheel_right_feedforward_ks_reverse", 15.26, logger);
    right.feedforward_kv =
        getHwParam<float>(info.hardware_parameters, "wheel_right_feedforward_kv", 14.72, logger);
    right.feedback_kp =
        getHwParam<float>(info.hardware_parameters, "wheel_right_feedback_kp", 3.0, logger);
    right.feedback_ki =
        getHwParam<float>(info.hardware_parameters, "wheel_right_feedback_ki", 27.0, logger);
    right.feedback_kd =
        getHwParam<float>(info.hardware_parameters, "wheel_right_feedback_kd", 0.0, logger);
    right.max_feedback_pwm = static_cast<uint16_t>(right_max_feedback_pwm);

    auto& left = config_data_.left_wheel;
    left.feedforward_ks_forward = getHwParam<float>(
        info.hardware_parameters, "wheel_left_feedforward_ks_forward", 17.86, logger);
    left.feedforward_ks_reverse = getHwParam<float>(
        info.hardware_parameters, "wheel_left_feedforward_ks_reverse", 15.68, logger);
    left.feedforward_kv =
        getHwParam<float>(info.hardware_parameters, "wheel_left_feedforward_kv", 14.26, logger);
    left.feedback_kp =
        getHwParam<float>(info.hardware_parameters, "wheel_left_feedback_kp", 3.0, logger);
    left.feedback_ki =
        getHwParam<float>(info.hardware_parameters, "wheel_left_feedback_ki", 25.0, logger);
    left.feedback_kd =
        getHwParam<float>(info.hardware_parameters, "wheel_left_feedback_kd", 0.0, logger);
    left.max_feedback_pwm = static_cast<uint16_t>(left_max_feedback_pwm);

    const auto wheel_values_are_finite = [](const WheelConfig& wheel) {
        return std::isfinite(wheel.feedforward_ks_forward) &&
               std::isfinite(wheel.feedforward_ks_reverse) &&
               std::isfinite(wheel.feedforward_kv) && std::isfinite(wheel.feedback_kp) &&
               std::isfinite(wheel.feedback_ki) && std::isfinite(wheel.feedback_kd);
    };

    if (!std::isfinite(wheels_min_velocity_) || wheels_min_velocity_ < 0.0 ||
        !wheel_values_are_finite(right) || !wheel_values_are_finite(left) ||
        !config_data_.valid()) {
        RCLCPP_ERROR(logger,
                     "Invalid differential-drive configuration: velocity limits and all gains "
                     "must be finite and nonnegative, maximum feedback PWM must be in "
                     "[1, 255], and control rate must be positive.");
        return false;
    }

    RCLCPP_INFO(logger, "Differential Drive parameters successfully loaded:");
    RCLCPP_INFO(logger, "  - Min velocity limit: %.4f rad/s", wheels_min_velocity_);
    RCLCPP_INFO(logger, "  - Control rate: %u Hz",
                static_cast<unsigned>(config_data_.control_rate_hz));
    RCLCPP_INFO(logger,
                "  - Right wheel -> FF {Ks_fwd: %.4f PWM, Ks_rev: %.4f PWM, "
                "Kv: %.4f PWM/(rad/s)} | "
                "PID {Kp: %.4f, Ki: %.4f, Kd: %.4f, max feedback: %u PWM}",
                right.feedforward_ks_forward, right.feedforward_ks_reverse, right.feedforward_kv,
                right.feedback_kp, right.feedback_ki, right.feedback_kd,
                static_cast<unsigned>(right.max_feedback_pwm));
    RCLCPP_INFO(logger,
                "  - Left wheel  -> FF {Ks_fwd: %.4f PWM, Ks_rev: %.4f PWM, "
                "Kv: %.4f PWM/(rad/s)} | "
                "PID {Kp: %.4f, Ki: %.4f, Kd: %.4f, max feedback: %u PWM}",
                left.feedforward_ks_forward, left.feedforward_ks_reverse, left.feedforward_kv,
                left.feedback_kp, left.feedback_ki, left.feedback_kd,
                static_cast<unsigned>(left.max_feedback_pwm));

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