/**
 * @file robot_system_interface.cpp
 *
 * SERIAL COMMUNICATION PROTOCOL SEQUENCE (ROS 2 PERSPECTIVE)
 * ----------------------------------------------------------
 * This hardware interface acts as the "Master" in the communication protocol. 
 * It drives the state machine transitions and pushes real-time commands to the microcontroller.
 *
 * 1. STARTUP & HANDSHAKE (on_configure)
 * - Opens the serial port and waits 200ms to ensure the Arduino bootloader finishes.
 * - Sends a `Config` message containing PID parameters and deadband limits.
 * - Blocks and waits for the Arduino to echo the exact same `Config` message back.
 * - Validates the echoed message to guarantee the MCU is running the correct parameters.
 *
 * 2. ACTIVATION (on_activate)
 * - Sends an initial `Velocity` command of {0.0, 0.0} to instruct the Arduino to
 * energize the motor drivers (activate torque).
 * - Blocks and waits for the Arduino to reply with its current zeroed `Velocity` state.
 *
 * 3. REAL-TIME LOOP (read / write)
 * - `read()`: Pulls asynchronous `Velocity` responses from the serial buffer. These
 * responses are the physical feedback triggered by the *previous* cycle's write() command. 
 * It integrates this velocity over the loop period (dt) to calculate odometry/position.
 * - `write()`: Pushes the new target `Velocity` to the Arduino. This acts as a continuous
 * heartbeat. If the Arduino does not receive this, it will trigger an emergency stop.
 *
 * 4. DEACTIVATION & CLEANUP (on_deactivate / on_cleanup / on_shutdown)
 * - Sends a `Deactivate` message commanding the MCU to drop motor torque.
 * - Blocks and waits for the Arduino to echo the `Deactivate` message as confirmation.
 * - Safely closes the serial port connection.
 */

#include "bumperbot_firmware/diff_drive/robot_system_interface.hpp"

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

RobotSystemInterface::RobotSystemInterface() {
  static_assert(TwoWheelsData{}.size() == kWheelNames.size(),
    "Differential drive interface must operate 2 wheels");
}

RobotSystemInterface::~RobotSystemInterface() {
  std::ignore = closeSerialConnection();
}

[[nodiscard]] bool RobotSystemInterface::validateWheelJointNamesConfig() const {
  for (const auto& expected_name : kWheelNames) {
    const bool found = std::any_of(cbegin(info_.joints), cend(info_.joints),
      [&expected_name](const auto& joint_info) {
        return joint_info.name == expected_name; 
      });
    
    if (not found) {
      RCLCPP_FATAL(rclcpp::get_logger("RobotSystemInterface"),
            "Wheel joint '%s' not found in joints configuration!",
            std::string(expected_name).c_str()
        );
      return false;
    }
  }

  return true;
}

CallbackReturn RobotSystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  auto logger = rclcpp::get_logger("RobotSystemInterface");
  RCLCPP_INFO(logger, "Initializing hardware.");

  CallbackReturn result = hardware_interface::SystemInterface::on_init(params);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  if (!validateWheelJointNamesConfig()) {
    return CallbackReturn::ERROR;
  }

  port_ = getHwParam<std::string>(
    info_.hardware_parameters, "port", "/dev/ttyACM0", logger);
  wheels_min_velocity_ = getHwParam<double>(
    info_.hardware_parameters, "wheels_min_velocity", 0.0, logger);
  config_data_.pid_rate = getHwParam<double>(
    info_.hardware_parameters, "wheels_pid_rate", 50.0, logger);
  config_data_.r_wheel.kp = getHwParam<double>(
    info_.hardware_parameters, "wheel_right_kp", 15.5, logger);
  config_data_.r_wheel.ki = getHwParam<double>(
    info_.hardware_parameters, "wheel_right_ki", 39.0, logger);
  config_data_.r_wheel.kd = getHwParam<double>(
    info_.hardware_parameters, "wheel_right_kd", 0.0, logger);
  config_data_.r_wheel.pwm_deadband = getHwParam<int>(
    info_.hardware_parameters, "wheel_right_pwm_deadband", 17, logger);
  config_data_.l_wheel.kp = getHwParam<double>(
    info_.hardware_parameters, "wheel_left_kp", 14.0, logger);
  config_data_.l_wheel.ki = getHwParam<double>(
    info_.hardware_parameters, "wheel_left_ki", 43.0, logger);
  config_data_.l_wheel.kd = getHwParam<double>(
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

CallbackReturn RobotSystemInterface::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Configuring hardware ...");

  transceiver_.openPort(port_, LibSerial::BaudRate::BAUD_115200);

  // wait for Arduino wake up
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // First send IMU configuration message with calibration data and wait for echo response
  constexpr size_t kMaxImuConfigAttempts{1};
  constexpr size_t kImuCalibrationPeriodMs{2000};
  constexpr size_t kImuConfigReqRespTimeMs{kImuCalibrationPeriodMs + 1000};

  ImuConfigData imu_config_data{
    .calibrate_period_ms = kImuCalibrationPeriodMs,
    .result = true
  };

  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), 
    "Sending IMU configuration message with calibration period %zu ms (keep robot flat on the ground)..."
      , kImuCalibrationPeriodMs);
  const auto imu_config_resp_opt =
    sendReceiveMessageData(imu_config_data, kMaxImuConfigAttempts, kImuConfigReqRespTimeMs);
  if (false == imu_config_resp_opt.has_value()) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"),
        "IMU configuration response message error: '%s'"
          , transceiver_.lastErrorMessage().c_str());
    return CallbackReturn::ERROR;
  }
  if (imu_config_resp_opt.value().result) {
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
      "IMU sensor initialized and calibrated successfully");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"),
        "IMU configuration/calibration failed");
    return CallbackReturn::ERROR;
  }

  // Next send configuration message and wait for echo response
  constexpr size_t kMaxConfigHandshakeAttempts{10};
  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), 
    "Sending Wheels configuration message, waiting for echo response (max %zu attempts) ..."
      , kMaxConfigHandshakeAttempts);

  const auto opt_response = sendReceiveMessageData(config_data_, kMaxConfigHandshakeAttempts);
  if (not opt_response) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"),
        "Configuration response message error: '%s'"
          , transceiver_.lastErrorMessage().c_str());
    return CallbackReturn::ERROR;
  }

  const auto response = *opt_response;
  if (response != config_data_) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"),
        "Response config mismatch: PID rate %.1f Hz | "
        "Right wheel config: { kp - %.1f, ki - %.1f, kd - %.1f } | "
        "Left wheel config: { kp - %.1f, ki - %.1f, kd - %.1f }" , response.pid_rate
        , response.r_wheel.kp, response.r_wheel.ki, response.r_wheel.kd
        , response.l_wheel.kp, response.l_wheel.ki, response.l_wheel.kd);
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "hardware configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Activating hardware ...");

  // Reset commands and states
  for (auto& wheel_data: wheels_data_) {
    wheel_data = {0.0, 0.0, 0.0};
  }

  constexpr size_t kMaxReqRespAttempts{5};
  constexpr DiffDriveCommandData kZeroWheelVelocity{
    .velocity = {
      .right_wheel_velocity = 0.0,
      .left_wheel_velocity = 0.0,
    },
    .response_delay_ms = 0
  };

  const auto opt_response =
    sendReceiveMessageData<DiffDriveCommandData, SystemStateData>(
        kZeroWheelVelocity, kMaxReqRespAttempts);

  if (not opt_response) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"),
        "Activation response message error: '%s'"
          , transceiver_.lastErrorMessage().c_str());
    return CallbackReturn::FAILURE;
  }

  // send extra velocity command for response
  // message to be available before first read()
  transceiver_.writeMessage(kZeroWheelVelocity);

  const auto& velocity_data = opt_response.value().diff_drive.velocity;
  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
    "Current wheels angular velocity (rad/sec): right wheel %.1f | left wheel %.1f"
        , velocity_data.right_wheel_velocity
        , velocity_data.left_wheel_velocity);

  // set communication budget with added 2ms
  // for serial transmission and sensor read overhead
  communication_budget_ms_ =
    static_cast<size_t>(std::lround(measured_roundtrip_ms_)) + 2u;

  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
    "hardware activated, ready to receive commands");

  // Reset response delay setup flag
  // for the next write() cycle
  response_delay_ms_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Deactivating hardware.");

  constexpr size_t kMaxReqRespAttempts{5};
  if (false == sendReceiveMessage<MsgId::Deactivate>(kMaxReqRespAttempts)) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"),
        "Deactivation response message error: '%s'"
          , transceiver_.lastErrorMessage().c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
              "hardware deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Cleaning up hardware.");

  transceiver_.closePort();

  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
    "hardware cleaned up, serial port closed.");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Shutting down hardware.");

  transceiver_.closePort();

  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "hardware shut down, serial port closed.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_error(const rclcpp_lifecycle::State &) {
  RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"), "Error occured in hardware.");

  if (closeSerialConnection()) {
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "hardware error handled, serial port closed.");
    return CallbackReturn::SUCCESS;
  }

  return CallbackReturn::FAILURE;
}

std::vector<hardware_interface::StateInterface> RobotSystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export state interfaces for DiffDriveController
  for (size_t i = 0; i < kWheelNames.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        std::string(kWheelNames[i]), hardware_interface::HW_IF_POSITION, &wheels_data_[i].position_state));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        std::string(kWheelNames[i]), hardware_interface::HW_IF_VELOCITY, &wheels_data_[i].velocity_state));
  }

  // export state interfaces for IMU broadcaster
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.x", &imu_sensor_data_.angular_velocity_x));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.y", &imu_sensor_data_.angular_velocity_y));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.z", &imu_sensor_data_.angular_velocity_z));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.x", &imu_sensor_data_.linear_acceleration_x));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.y", &imu_sensor_data_.linear_acceleration_y));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.z", &imu_sensor_data_.linear_acceleration_z));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.x", &imu_sensor_data_.orientation_x));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.y", &imu_sensor_data_.orientation_y));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.z", &imu_sensor_data_.orientation_z));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.w", &imu_sensor_data_.orientation_w));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity command interface
  for (size_t i = 0; i < kWheelNames.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        std::string(kWheelNames[i]), hardware_interface::HW_IF_VELOCITY, &wheels_data_[i].velocity_command));
  }

  return command_interfaces;
}

hardware_interface::return_type RobotSystemInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration & period)
{
  constexpr size_t kVelocityReadErrorThreshold{10};

  if (!processSystemStateMessage()) {
    if (velocity_read_error_count_ > kVelocityReadErrorThreshold) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"),
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

hardware_interface::return_type RobotSystemInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &period)
{
  if (!response_delay_ms_) {
    computeResponseDelay(period);
  }

  DiffDriveCommandData command{
    .velocity = {
      .right_wheel_velocity = static_cast<float>(wheels_data_[kRightWheelIndex].velocity_command),
      .left_wheel_velocity = static_cast<float>(wheels_data_[kLeftWheelIndex].velocity_command),
    },
    .response_delay_ms = *response_delay_ms_
  };

  // A tiny epsilon threshold to account for floating-point noise
  constexpr double kZeroThreshold = 1e-4;

  // --- Right Wheel Control ---
  if (std::abs(command.velocity.right_wheel_velocity) <= kZeroThreshold) {
    // Force a clean stop if the command is microscopic noise
    command.velocity.right_wheel_velocity = 0.0;
  }
  else if (std::abs(command.velocity.right_wheel_velocity) < wheels_min_velocity_) {
    // Clamp up to minimum velocity if trying to move but under mechanical limits
    command.velocity.right_wheel_velocity = std::copysign(wheels_min_velocity_, command.velocity.right_wheel_velocity);
  }

  // --- Left Wheel Control ---
  if (std::abs(command.velocity.left_wheel_velocity) <= kZeroThreshold) {
    // Force a clean stop if the command is microscopic noise
    command.velocity.left_wheel_velocity = 0.0;
  }
  else if (std::abs(command.velocity.left_wheel_velocity) < wheels_min_velocity_) {
    // Clamp up to minimum velocity if trying to move but under mechanical limits
    command.velocity.left_wheel_velocity = std::copysign(wheels_min_velocity_, command.velocity.left_wheel_velocity);
  }

  transceiver_.writeMessage(command);

  return hardware_interface::return_type::OK;
}

bool RobotSystemInterface::processSystemStateMessage() {
  if (!transceiver_.isDataAvailable()) {
    RCLCPP_WARN(rclcpp::get_logger("RobotSystemInterface"),
      "Wheels velocity state message not available, count: '%lu'"
        , ++velocity_read_error_count_);
    return false;
  }

  const auto opt_state = transceiver_.readLastMessageData<SystemStateData>();
  if (!opt_state) {
    RCLCPP_WARN(rclcpp::get_logger("RobotSystemInterface"),
      "Failed to process wheels velocity state message: '%s'"
        , transceiver_.lastErrorMessage().c_str());

    ++velocity_read_error_count_;
    return false;
  }

  if (opt_state.value().status != SystemStateFlags::ImuUnavailable) {
    if (!imu_sensor_data_.data_available) {
      RCLCPP_WARN(rclcpp::get_logger("RobotSystemInterface"),
        "IMU sensor data available again");
      imu_sensor_data_.data_available = true;
    }

    const auto& imu_state = opt_state.value().imu;
    imu_sensor_data_.angular_velocity_x = imu_state.angular_velocity_x;
    imu_sensor_data_.angular_velocity_y = imu_state.angular_velocity_y;
    imu_sensor_data_.angular_velocity_z = imu_state.angular_velocity_z;
    imu_sensor_data_.linear_acceleration_x = imu_state.linear_acceleration_x;
    imu_sensor_data_.linear_acceleration_y = imu_state.linear_acceleration_y;
    imu_sensor_data_.linear_acceleration_z = imu_state.linear_acceleration_z;
  } else {
    if (imu_sensor_data_.data_available) {
      RCLCPP_WARN(rclcpp::get_logger("RobotSystemInterface"),
        "IMU sensor data unavailable");
      imu_sensor_data_.data_available = false;
    }
  }

  const auto& velocity_data = opt_state.value().diff_drive.velocity;
  wheels_data_[kRightWheelIndex].velocity_state = velocity_data.right_wheel_velocity;
  wheels_data_[kLeftWheelIndex].velocity_state = velocity_data.left_wheel_velocity;
  velocity_read_error_count_ = 0;

  return true;
}

bool RobotSystemInterface::closeSerialConnection() noexcept {
  bool result{true};
  try {
    transceiver_.closePort();
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("RobotSystemInterface"),
            "Exception occured while closing connection at port: %s"
                , port_.c_str());
    result = false;
  }

  return result;
}

void RobotSystemInterface::computeResponseDelay(const rclcpp::Duration & period) {
  // Calculate the response delay to account for the roundtrip time
  // and ensure the Arduino has enough time to process the command
  const auto period_ms = static_cast<size_t>(
    std::lround(period.seconds() * 1000));

  response_delay_ms_ = (period_ms > communication_budget_ms_)
    ? static_cast<uint8_t>(period_ms - communication_budget_ms_) : 0;

  RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
    "Response message delay: %u ms (Period: %.2f ms, Communication Budget: %zu ms)"
      , *response_delay_ms_, period.seconds() * 1000, communication_budget_ms_);
}

template<typename SendData, typename ReceiveData>
std::optional<ReceiveData> RobotSystemInterface::sendReceiveMessageData(
      const SendData& send_data, const size_t max_attempts, const size_t wait_time_ms)
{
  for (size_t attempt = 1; attempt <= max_attempts; attempt++) {
    transceiver_.writeMessage(send_data);

    const auto start = std::chrono::steady_clock::now();
    const auto opt_response_data = transceiver_.waitForNextMessageData<ReceiveData>(wait_time_ms);
    const auto elapsed = std::chrono::steady_clock::now() - start;

    if (opt_response_data) {
      measured_roundtrip_ms_ = std::chrono::duration<double, std::milli>(elapsed).count();

      RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
        "Successfull req/resp attempt %zu took time %f milliseconds"
          , attempt, measured_roundtrip_ms_);

      return opt_response_data;
    }
  }

  return std::nullopt;
}

template<MsgId TargetId>
bool RobotSystemInterface::sendReceiveMessage(const size_t max_attempts) {
  for (size_t attempt = 1; attempt <= max_attempts; attempt++) {
    transceiver_.template writeMessage<TargetId>();

    constexpr size_t kWaitTimeMs{100};
    const auto start = std::chrono::steady_clock::now();
    const bool result = transceiver_.waitForNextMessage<TargetId>(kWaitTimeMs);
    const auto elapsed = std::chrono::steady_clock::now() - start;

    if (result) {
      measured_roundtrip_ms_ = std::chrono::duration<double, std::milli>(elapsed).count();

      RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
        "Successfull req/resp attempt %zu took time %f milliseconds"
          , attempt, measured_roundtrip_ms_);

      return true;
    }
  }

  return false;
}

}  // namespace bumperbot_firmware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::RobotSystemInterface, hardware_interface::SystemInterface)