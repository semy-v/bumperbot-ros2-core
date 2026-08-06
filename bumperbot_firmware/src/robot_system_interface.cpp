/**
 * @file robot_system_interface.cpp
 *
 * SERIAL COMMUNICATION PROTOCOL SEQUENCE (ROS 2 PERSPECTIVE)
 * ----------------------------------------------------------
 * This hardware interface acts as the "Master" in the communication protocol.
 * It drives the state machine transitions and pushes real-time commands to the
 * microcontroller while delegating state/command processing to modular subsystem
 * handlers (`DifferentialDriveHandler` and `ImuSensorHandler`).
 *
 * 1. STARTUP & HANDSHAKE (on_configure)
 * - Opens the serial port (`/dev/ttyACM0` by default) and waits 200ms for the
 * Arduino bootloader to settle.
 * - Phase 1 (IMU Calibration): Sends an initial `ImuConfigData` message specifying
 * the calibration period (2000ms) and blocks while waiting for an echo confirmation.
 * - Phase 2 (Wheel Config): Sends a `DiffDriveConfigData` message containing PID rate,
 * gains (kp, ki, kd), and PWM deadband limits for both wheels.
 * - Blocks and waits for the Arduino to echo the exact same config parameters back,
 * validating the MCU is running the correct parameters before continuing.
 *
 * 2. ACTIVATION (on_activate)
 * - Resets all internal odometry and wheel command/state values to zero via the handler.
 * - Sends an initial `Velocity` command of {0.0, 0.0} to instruct the Arduino
 * to energize the motor drivers (activate torque).
 * - Blocks and waits for the Arduino to reply with its current zeroed `SystemStateData`.
 * - Measures communication roundtrip latency to dynamically compute a communications budget
 * and calculates the appropriate `response_delay_ms` for subsequent control loop cycles.
 *
 * 3. REAL-TIME LOOP (read / write)
 * - `read()`: Pulls asynchronous `SystemStateData` responses from the serial buffer.
 * These responses represent the physical feedback triggered by the *previous* cycle's
 * write() command.
 * - Delegates wheel velocity parsing and odometry integration (dt) to `diff_drive_handler_`.
 * - Delegates IMU telemetry extraction (angular velocity, linear acceleration) to `imu_handler_`.
 * - **Dynamic Degradation**: Checks `SystemStateFlags::ImuUnavailable`. If IMU data drops out,
 * it invalidates the IMU state interfaces (e.g., by setting values to quiet NaN) without
 * halting the differential drive control loop.
 * - `write()`: Pushes the new target `DiffDriveCommandData` (wheel velocities and computed
 * response delay) to the Arduino. This acts as a continuous heartbeat; missing packets will
 * trigger an emergency stop on the MCU.
 *
 * 4. DEACTIVATION & CLEANUP (on_deactivate / on_cleanup / on_shutdown / on_error)
 * - Sends a `Deactivate` message commanding the MCU to safely drop motor torque.
 * - Blocks and waits for the Arduino to echo the `Deactivate` message as confirmation.
 * - Safely terminates open serial connections and releases interface locks.
 */

#include "bumperbot_firmware/robot_system_interface.hpp"

#include <algorithm>
#include <chrono>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "bumperbot_firmware/hardware_interface_helpers.hpp"
#include "protocol/system_data.hpp"

namespace bumperbot_firmware {

RobotSystemInterface::~RobotSystemInterface() {
    std::ignore = closeSerialConnection();
}

CallbackReturn RobotSystemInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    auto logger = rclcpp::get_logger("RobotSystemInterface");
    RCLCPP_INFO(logger, "Initializing hardware...");

    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    if (!diff_drive_handler_.init(info_, logger)) {
        return CallbackReturn::ERROR;
    }

    port_ = getHwParam<std::string>(info_.hardware_parameters, "port", "/dev/ttyACM0", logger);
    RCLCPP_INFO(logger, "Hardware initialized successfully with port '%s'.", port_.c_str());
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
RobotSystemInterface::on_export_state_interfaces() {
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Exporting state interfaces.");

    auto diff_states = diff_drive_handler_.exportStateInterfaces();
    auto imu_states = imu_handler_.exportStateInterfaces();

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> all_states;
    all_states.reserve(diff_states.size() + imu_states.size());
    all_states.insert(all_states.end(), diff_states.begin(), diff_states.end());
    all_states.insert(all_states.end(), imu_states.begin(), imu_states.end());

    return all_states;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
RobotSystemInterface::on_export_command_interfaces() {
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Exporting command interfaces.");

    auto diff_commands = diff_drive_handler_.exportCommandInterfaces();
    return {diff_commands.begin(), diff_commands.end()};
}

CallbackReturn RobotSystemInterface::on_configure(const rclcpp_lifecycle::State&) {
    auto logger = rclcpp::get_logger("RobotSystemInterface");
    RCLCPP_INFO(logger, "Configuring hardware...");

    transceiver_.openPort(port_, LibSerial::BaudRate::BAUD_115200);
    // wait for Arduino wake up
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO(logger, "Serial connection opened.");

    // Send IMU config message and wait for echo response
    constexpr size_t kImuCalibMs{2000};
    constexpr size_t kImuConfigAttempts{1};
    constexpr size_t kImuConfigMs{kImuCalibMs + 1000};

    RCLCPP_INFO(logger, "Initializing and Calibrating IMU sensor (%zu ms)...", kImuCalibMs);
    auto imu_resp = sendReceiveMessageData(imu_handler_.getDefaultConfig(kImuCalibMs),
                                           kImuConfigAttempts, kImuConfigMs);
    if (!imu_resp || !imu_resp->result) {
        RCLCPP_ERROR(logger, "IMU calibration failed: '%s'",
                     transceiver_.lastErrorMessage().c_str());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger, "IMU sensor initialized and calibrated.");

    // Send Differential drive configuration message and wait for echo response
    RCLCPP_INFO(logger, "Configuring differential drive...");
    constexpr size_t kDiffDriveConfigAttempts{10};
    auto cfg_resp =
        sendReceiveMessageData(diff_drive_handler_.getConfig(), kDiffDriveConfigAttempts);
    if (!cfg_resp) {
        RCLCPP_ERROR(logger, "Differential drive config handshake message error: '%s'",
                     transceiver_.lastErrorMessage().c_str());
        return CallbackReturn::ERROR;
    }
    if (*cfg_resp != diff_drive_handler_.getConfig()) {
        const auto& right = cfg_resp->right_wheel;
        const auto& left = cfg_resp->left_wheel;
        RCLCPP_ERROR(logger,
                     "Differential drive handshake mismatch: control rate %u Hz | "
                     "Right wheel: { Ks=%.4f, Kv=%.4f, Kp=%.4f, Ki=%.4f, Kd=%.4f, "
                     "max feedback=%u PWM } | "
                     "Left wheel: { Ks=%.4f, Kv=%.4f, Kp=%.4f, Ki=%.4f, Kd=%.4f, "
                     "max feedback=%u PWM }",
                     static_cast<unsigned>(cfg_resp->control_rate_hz), right.feedforward_ks,
                     right.feedforward_kv, right.feedback_kp, right.feedback_ki, right.feedback_kd,
                     static_cast<unsigned>(right.max_feedback_pwm), left.feedforward_ks,
                     left.feedforward_kv, left.feedback_kp, left.feedback_ki, left.feedback_kd,
                     static_cast<unsigned>(left.max_feedback_pwm));
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger, "Hardware successfully configured.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_activate(const rclcpp_lifecycle::State&) {
    auto logger = rclcpp::get_logger("RobotSystemInterface");
    RCLCPP_INFO(logger, "Activating hardware...");

    diff_drive_handler_.resetStates();

    constexpr DiffDriveCommandData kZeroVel{.velocity = {0.0f, 0.0f}, .response_delay_ms = 0};
    auto opt_resp = sendReceiveMessageData<DiffDriveCommandData, SystemStateData>(kZeroVel, 5);
    if (!opt_resp) {
        RCLCPP_ERROR(logger, "Activation failed: '%s'", transceiver_.lastErrorMessage().c_str());
        return CallbackReturn::FAILURE;
    }

    // send extra velocity command for response
    // message to be available before first read()
    transceiver_.writeMessage(kZeroVel);

    // set communication budget with added 2ms
    // for serial transmission and sensor read overhead
    communication_budget_ms_ = static_cast<size_t>(std::ceil(measured_roundtrip_ms_)) + 2u;

    // Reset response delay setup flag
    // for the next write() cycle
    response_delay_ms_.reset();

    RCLCPP_INFO(logger, "Hardware activated. Ready for real-time loop.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_deactivate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Deactivating hardware.");

    constexpr size_t kMaxReqRespAttempts{5};
    if (!sendReceiveMessage<MsgId::Deactivate>(kMaxReqRespAttempts)) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"),
                     "Deactivation response message error: '%s'",
                     transceiver_.lastErrorMessage().c_str());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "hardware deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_cleanup(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Cleaning up hardware.");

    transceiver_.closePort();

    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
                "hardware cleaned up, serial port closed.");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_shutdown(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"), "Shutting down hardware.");

    transceiver_.closePort();

    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
                "hardware shut down, serial port closed.");

    return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystemInterface::on_error(const rclcpp_lifecycle::State&) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"), "Error occured in hardware.");

    if (closeSerialConnection()) {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
                    "hardware error handled, serial port closed.");
        return CallbackReturn::SUCCESS;
    }

    return CallbackReturn::FAILURE;
}

hardware_interface::return_type RobotSystemInterface::read(const rclcpp::Time&,
                                                           const rclcpp::Duration& period) {
    constexpr size_t kVelocityReadErrorThreshold{10};
    if (!processSystemStateMessage()) {
        if (velocity_read_error_count_ > kVelocityReadErrorThreshold) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotSystemInterface"),
                         "Exceeded maximum velocity message read error threshold: %zu",
                         kVelocityReadErrorThreshold);
            return hardware_interface::return_type::ERROR;
        }
    }

    diff_drive_handler_.integrateOdometry(period.seconds());
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSystemInterface::write(const rclcpp::Time&,
                                                            const rclcpp::Duration& period) {
    if (!response_delay_ms_) {
        computeResponseDelay(period);
    }

    transceiver_.writeMessage(diff_drive_handler_.getCommandData(*response_delay_ms_));
    return hardware_interface::return_type::OK;
}

bool RobotSystemInterface::processSystemStateMessage() {
    auto logger = rclcpp::get_logger("RobotSystemInterface");
    if (0 == transceiver_.numberOfBytesAvailable()) {
        velocity_read_error_count_++;
        return false;
    }

    const auto opt_state = transceiver_.readLastMessageData<SystemStateData>();
    if (!opt_state) {
        RCLCPP_WARN(rclcpp::get_logger("RobotSystemInterface"),
                    "Failed to process wheels velocity state message: '%s'",
                    transceiver_.lastErrorMessage().c_str());
        ++velocity_read_error_count_;
        return false;
    }

    const bool imu_ok = (opt_state->status != SystemStateFlags::ImuUnavailable);
    imu_handler_.setAvailability(imu_ok, logger);

    if (imu_ok) {
        imu_handler_.updateFromState(opt_state->imu);
    }

    diff_drive_handler_.updateFromState(opt_state->diff_drive);
    velocity_read_error_count_ = 0;
    return true;
}

bool RobotSystemInterface::closeSerialConnection() noexcept {
    bool result{true};
    try {
        transceiver_.closePort();
    } catch (...) {
        RCLCPP_FATAL(rclcpp::get_logger("RobotSystemInterface"),
                     "Exception occured while closing connection at port: %s", port_.c_str());
        result = false;
    }

    return result;
}

void RobotSystemInterface::computeResponseDelay(const rclcpp::Duration& period) {
    // Calculate the response delay to account for the roundtrip time
    // and ensure the Arduino has enough time to process the command
    const auto period_ms = static_cast<size_t>(std::lround(period.seconds() * 1000));

    response_delay_ms_ = (period_ms > communication_budget_ms_)
                             ? static_cast<uint8_t>(period_ms - communication_budget_ms_)
                             : 0;

    RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
                "Response message delay: %u ms (Period: %zu ms, Communication "
                "Budget: %zu ms)",
                *response_delay_ms_, period_ms, communication_budget_ms_);
}

template <typename SendData, typename ReceiveData>
std::optional<ReceiveData> RobotSystemInterface::sendReceiveMessageData(const SendData& send_data,
                                                                        const size_t max_attempts,
                                                                        const size_t wait_time_ms) {
    for (size_t attempt = 1; attempt <= max_attempts; attempt++) {
        const auto start = std::chrono::steady_clock::now();
        transceiver_.writeMessage(send_data);
        const auto opt_response = transceiver_.waitForNextMessageData<ReceiveData>(wait_time_ms);
        const auto elapsed = std::chrono::steady_clock::now() - start;

        if (opt_response) {
            measured_roundtrip_ms_ = std::chrono::duration<double, std::milli>(elapsed).count();

            RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
                        "Successfull req/resp attempt %zu took time %f milliseconds", attempt,
                        measured_roundtrip_ms_);
            return opt_response;
        }
    }

    return std::nullopt;
}

template <MsgId TargetId>
bool RobotSystemInterface::sendReceiveMessage(const size_t max_attempts) {
    for (size_t attempt = 1; attempt <= max_attempts; attempt++) {
        constexpr size_t kWaitTimeMs{100};
        const auto start = std::chrono::steady_clock::now();
        transceiver_.template writeMessage<TargetId>();
        const bool result = transceiver_.waitForNextMessage<TargetId>(kWaitTimeMs);
        const auto elapsed = std::chrono::steady_clock::now() - start;

        if (result) {
            measured_roundtrip_ms_ = std::chrono::duration<double, std::milli>(elapsed).count();

            RCLCPP_INFO(rclcpp::get_logger("RobotSystemInterface"),
                        "Successfull req/resp attempt %zu took time %f milliseconds", attempt,
                        measured_roundtrip_ms_);
            return true;
        }
    }

    return false;
}

}  // namespace bumperbot_firmware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::RobotSystemInterface,
                       hardware_interface::SystemInterface)