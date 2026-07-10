#ifndef HARDWARE_INTERFACE_HELPERS_HPP
#define HARDWARE_INTERFACE_HELPERS_HPP

#include <string>
#include <type_traits>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

namespace bumperbot_firmware {

template <typename T>
T getHwParam(const std::unordered_map<std::string, std::string>& hw_params,
             const std::string& param_name,
             const T& default_value,
             const rclcpp::Logger& logger) {
    auto it = hw_params.find(param_name);
    if (it == hw_params.end()) {
        RCLCPP_WARN(logger, "Parameter '%s' not found. Using default.", param_name.c_str());
        return default_value;
    }
    try {
        if constexpr (std::is_same_v<T, std::string>) {
            return it->second;
        } else if constexpr (std::is_same_v<T, double>) {
            return std::stod(it->second);
        } else if constexpr (std::is_same_v<T, int>) {
            return std::stoi(it->second);
        } else if constexpr (std::is_same_v<T, float>) {
            return std::stof(it->second);
        }
    } catch (...) {
        RCLCPP_ERROR(logger, "Failed to parse '%s'. Using default.", param_name.c_str());
    }
    return default_value;
}

}  // namespace bumperbot_firmware

#endif  // HARDWARE_INTERFACE_HELPERS_HPP