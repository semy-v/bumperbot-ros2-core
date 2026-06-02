#include "bumperbot_firmware/diff_drive/diff_drive_interface.hpp"

#include <algorithm>
#include <string_view>
#include <array>
#include <tuple>
#include <chrono>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "diff_drive_data.hpp"

namespace {

using namespace std::string_view_literals;

constexpr std::array kWheelNames{
  "wheel_left_joint"sv,
  "wheel_right_joint"sv
};

constexpr size_t kLeftWheelIndex{0};
constexpr size_t kRightWheelIndex{1};

template<typename T>
T getHwParam(
  const std::unordered_map<std::string, std::string>& hw_params,
  const std::string& param_name,
  const T& default_value,
  const rclcpp::Logger& logger) 
{
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
  } catch (const std::invalid_argument& e) {
    RCLCPP_ERROR(logger, "Failed to parse '%s'. Using default.", param_name.c_str());
  } catch (const std::out_of_range& e) {
    RCLCPP_ERROR(logger, "Value for '%s' out of range. Using default.", param_name.c_str());
  }
    
  return default_value;
}

} // namespace

namespace bumperbot_firmware
{

DiffDriveInterface::DiffDriveInterface() {
  static_assert(TwoWheelsData{}.size() == kWheelNames.size(),
    "Differential drive interface must operate 2 wheels");
}

DiffDriveInterface::~DiffDriveInterface() {
  std::ignore = closeSerialConnection();
}

[[nodiscard]] bool DiffDriveInterface::validateWheelJointNamesConfig() const {
  for (const auto& expected_name : kWheelNames) {
    const bool found = std::any_of(cbegin(info_.joints), cend(info_.joints),
      [&expected_name](const auto& joint_info) {
        return joint_info.name == expected_name; 
      });
    
    if (not found) {
      RCLCPP_FATAL(rclcpp::get_logger("DiffDriveInterface"),
            "Wheel joint '%s' not found in joints configuration!",
            std::string(expected_name).c_str()
        );
      return false;
    }
  }

  return true;
}

CallbackReturn DiffDriveInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  auto logger = rclcpp::get_logger("DiffDriveInterface");
  RCLCPP_INFO(logger, "Initializing hardware.");

  CallbackReturn result = hardware_interface::SystemInterface::on_init(params);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  if (!validateWheelJointNamesConfig()) {
    return CallbackReturn::ERROR;
  }

  port_ = getHwParam<std::string>(
    info_.hardware_parameters, "port", "/dev/ttyUSB0", logger);
  wheels_min_velocity_ = getHwParam<double>(
    info_.hardware_parameters, "wheels_min_velocity", 0.0, logger);
  config_data_.pid_rate = getHwParam<float>(
    info_.hardware_parameters, "wheels_pid_rate", 50.0, logger);
  config_data_.r_wheel.kp = getHwParam<float>(
    info_.hardware_parameters, "wheel_right_kp", 15.5, logger);
  config_data_.r_wheel.ki = getHwParam<float>(
    info_.hardware_parameters, "wheel_right_ki", 39.0, logger);
  config_data_.r_wheel.kd = getHwParam<float>(
    info_.hardware_parameters, "wheel_right_kd", 0.0, logger);
  config_data_.r_wheel.pwm_deadband = getHwParam<int>(
    info_.hardware_parameters, "wheel_right_pwm_deadband", 17, logger);
  config_data_.l_wheel.kp = getHwParam<float>(
    info_.hardware_parameters, "wheel_left_kp", 14.0, logger);
  config_data_.l_wheel.ki = getHwParam<float>(
    info_.hardware_parameters, "wheel_left_ki", 43.0, logger);
  config_data_.l_wheel.kd = getHwParam<float>(
    info_.hardware_parameters, "wheel_left_kd", 0.0, logger);
  config_data_.l_wheel.pwm_deadband = getHwParam<int>(
    info_.hardware_parameters, "wheel_left_pwm_deadband", 18, logger);

  RCLCPP_INFO(logger, "init params: port '%s' min angular velocity %.1f rad/s | PID rate %.1f Hz | "
    "Right wheel config: { kp %.1f, ki %.1f, kd %.1f, pwm_deadband +-%d } | "
    "Left wheel config: { kp %.1f, ki %.1f, kd %.1f, pwm_deadband +-%d }"
    , port_.c_str(), wheels_min_velocity_, config_data_.pid_rate
    , config_data_.r_wheel.kp, config_data_.r_wheel.ki, config_data_.r_wheel.kd, config_data_.r_wheel.pwm_deadband
    , config_data_.l_wheel.kp, config_data_.l_wheel.ki, config_data_.l_wheel.kd, config_data_.l_wheel.pwm_deadband);
  
  RCLCPP_INFO(logger, "hardware initialized");

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveInterface::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"), "Configuring hardware ...");

  transceiver_.openPort(port_, LibSerial::BaudRate::BAUD_115200);

  // wait for Arduino wake up
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  constexpr size_t kMaxConfigHandshakeAttempts{10};
  const auto optional_response_data = sendReceiveMessageData(config_data_, kMaxConfigHandshakeAttempts);
  if (not optional_response_data) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"),
        "Configuration response message error: '%s'"
          , transceiver_.lastErrorMessage().c_str());
    return CallbackReturn::ERROR;
  }

  const auto response_data = *optional_response_data;
  if (response_data != config_data_) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"),
        "Response config mismatch: PID rate %.1f Hz | "
        "Right wheel config: { kp - %.1f, ki - %.1f, kd - %.1f } | "
        "Left wheel config: { kp - %.1f, ki - %.1f, kd - %.1f }" , response_data.pid_rate
        , response_data.r_wheel.kp, response_data.r_wheel.ki, response_data.r_wheel.kd
        , response_data.l_wheel.kp, response_data.l_wheel.ki, response_data.l_wheel.kd);
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"), "hardware configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveInterface::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"), "Activating hardware ...");

  // Reset commands and states
  for (auto& wheel_data: wheels_data_) {
    wheel_data = {0.0, 0.0, 0.0};
  }

  constexpr size_t kMaxReqRespAttempts{5};
  constexpr VelocityData kZeroWheelVelocity{0.0, 0.0};

  const auto optional_response_data = sendReceiveMessageData(
    kZeroWheelVelocity, kMaxReqRespAttempts);

  if (not optional_response_data) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"),
        "Activation response message error: '%s'"
          , transceiver_.lastErrorMessage().c_str());
    return CallbackReturn::FAILURE;
  }

  const auto response_data = *optional_response_data;
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
    "Current wheels angular velocity (rad/sec): right wheel %.1f | left wheel %.1f"
        , response_data.right_wheel_velocity, response_data.left_wheel_velocity);

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "hardware activated, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"), "Deactivating hardware.");

  constexpr size_t kMaxReqRespAttempts{5};
  if (false == sendReceiveMessage<MsgId::Deactivate>(kMaxReqRespAttempts)) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"),
        "Deactivation response message error: '%s'"
          , transceiver_.lastErrorMessage().c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
              "hardware deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveInterface::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"), "Cleaning up hardware.");

  transceiver_.closePort();

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
    "hardware cleaned up, serial port closed.");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveInterface::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"), "Shutting down hardware.");

  transceiver_.closePort();

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"), "hardware shut down, serial port closed.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveInterface::on_error(const rclcpp_lifecycle::State &) {
  RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"), "Error occured in hardware.");

  if (closeSerialConnection()) {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"), "hardware error handled, serial port closed.");
    return CallbackReturn::SUCCESS;
  }

  return CallbackReturn::FAILURE;
}

std::vector<hardware_interface::StateInterface> DiffDriveInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < kWheelNames.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        std::string(kWheelNames[i]), hardware_interface::HW_IF_POSITION, &wheels_data_[i].position_state));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        std::string(kWheelNames[i]), hardware_interface::HW_IF_VELOCITY, &wheels_data_[i].velocity_state));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity command interface
  for (size_t i = 0; i < kWheelNames.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        std::string(kWheelNames[i]), hardware_interface::HW_IF_VELOCITY, &wheels_data_[i].velocity_command));
  }

  return command_interfaces;
}

hardware_interface::return_type DiffDriveInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration & period)
{
  constexpr size_t kVelocityReadErrorThreshold{10};

  if (!processVelocityStateMessage()) {
    if (velocity_read_error_count_ > kVelocityReadErrorThreshold) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveInterface"),
        "Exceeded maximum velocity message read error threshold: %zu"
          , kVelocityReadErrorThreshold);
      return hardware_interface::return_type::ERROR;
    }
  }

  const auto dt = period.seconds();
  wheels_data_[kRightWheelIndex].position_state +=
    wheels_data_[kRightWheelIndex].velocity_state * dt;
  wheels_data_[kLeftWheelIndex].position_state +=
    wheels_data_[kLeftWheelIndex].velocity_state * dt;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  double r_wheel_target = wheels_data_[kRightWheelIndex].velocity_command;
  double l_wheel_target = wheels_data_[kLeftWheelIndex].velocity_command;

  // clamp off if target wheel angular velocity is less than
  // minimum configured one (with regards to the motor pwm deadband)
  if (std::abs(r_wheel_target) < wheels_min_velocity_) {
    r_wheel_target = 0.0;
  }

  if (std::abs(l_wheel_target) < wheels_min_velocity_) {
    l_wheel_target = 0.0;
  }

  const VelocityData data{
    static_cast<float>(r_wheel_target),
    static_cast<float>(l_wheel_target)
  };

  transceiver_.writeMessage(data);

  return hardware_interface::return_type::OK;
}

size_t DiffDriveInterface::waitDataAvailableToRead(const size_t wait_time_ms) {
  const auto start = std::chrono::steady_clock::now();

  // Wait read data to become available
  while (!transceiver_.isDataAvailable()) {
      if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(wait_time_ms)) {
          break; // Timeout
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  const auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start);

  return elapsed_time_ms.count();
}

bool DiffDriveInterface::processVelocityStateMessage() {
  if (!transceiver_.isDataAvailable()) {
    ++velocity_read_error_count_;
    return false;
  }

  VelocityData data;
  if (!transceiver_.readLastMessageData(data)) {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveInterface"),
      "Failed to process wheels velocity state message: '%s'"
        , transceiver_.lastErrorMessage().c_str());

    ++velocity_read_error_count_;
    return false;
  }

  wheels_data_[kRightWheelIndex].velocity_state = data.right_wheel_velocity;
  wheels_data_[kLeftWheelIndex].velocity_state = data.left_wheel_velocity;
  velocity_read_error_count_ = 0;

  return true;
}

bool DiffDriveInterface::closeSerialConnection() noexcept {
  bool result{true};
  try {
    transceiver_.closePort();
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("DiffDriveInterface"),
            "Exception occured while closing connection at port: %s"
                , port_.c_str());
    result = false;
  }

  return result;
}

}  // namespace bumperbot_firmware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::DiffDriveInterface, hardware_interface::SystemInterface)