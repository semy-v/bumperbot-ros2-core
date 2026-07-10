#ifndef ROBOT_SYSTEM_INTERFACE_HPP
#define ROBOT_SYSTEM_INTERFACE_HPP

#include <optional>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <libserial/SerialPort.h>

#include "serial_message_protocol.hpp"
#include "serial_message_transceiver.hpp"
#include "system_data.hpp"
#include "system_messages.hpp"

#include "bumperbot_firmware/diff_drive_handler.hpp"
#include "bumperbot_firmware/imu_sensor_handler.hpp"

namespace bumperbot_firmware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class RobotSystemInterface
 * @brief ROS 2 System Hardware Interface for the Bumperbot differential drive and IMU subsystem.
 *
 * @details This class acts as the communication Master in a serial protocol with a microcontroller (e.g., Arduino).
 * It bridges the ROS 2 `ros2_control` ecosystem with physical hardware by:
 * - Managing the lifecycle of the serial connection and hardware subsystems.
 * - Delegating wheel odometry and velocity control to `DifferentialDriveHandler`.
 * - Delegating inertial measurement processing and degradation handling to `ImuSensorHandler`.
 * - Implementing a synchronized, low-latency read/write loop with communication budgeting.
 */
class RobotSystemInterface : public hardware_interface::SystemInterface {
 public:
    /**
     * @brief Default constructor.
     */
    RobotSystemInterface() = default;

    /**
     * @brief Virtual destructor. Safely terminates any open serial connections.
     */
    virtual ~RobotSystemInterface();

    // Disable copy and move semantics to ensure exclusive hardware resource ownership
    RobotSystemInterface(const RobotSystemInterface&) = delete;
    RobotSystemInterface(RobotSystemInterface&&) = delete;
    RobotSystemInterface& operator=(const RobotSystemInterface&) = delete;
    RobotSystemInterface& operator=(RobotSystemInterface&&) = delete;

    // =========================================================================
    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface Implementation
    // =========================================================================

    /**
     * @brief Initializes the hardware interface from URDF configuration parameters.
     * @details Parses hardware parameters (such as serial port name and PID rates), initializes
     *          the differential drive handler, and verifies that the URDF joint configuration matches
     *          expected wheel joint names.
     * @param params Structure containing parameters and joint information defined in the URDF `<ros2_control>` tag.
     * @return CallbackReturn::SUCCESS if initialization and validation succeed, CallbackReturn::ERROR otherwise.
     */
    CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    /**
     * @brief Configures the hardware and performs the initial startup handshake with the MCU.
     * @details Opens the serial port, waits for the microcontroller bootloader to settle, and executes a two-phase
     *          configuration handshake:
     *          1. Sends IMU calibration parameters and waits for confirmation.
     *          2. Sends differential drive PID/deadband configurations and verifies the echoed response.
     * @param previous_state The lifecycle state from which this transition was triggered.
     * @return CallbackReturn::SUCCESS if serial communication and handshakes succeed, CallbackReturn::ERROR otherwise.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Activates the hardware, energizing motor actuators and preparing the real-time loop.
     * @details Resets odometry states, sends a zero-velocity heartbeat command to enable MCU motor driver
     *          torque, measures serial roundtrip latency to compute the communication budget, and prepares
     *          response delay synchronization.
     * @param previous_state The lifecycle state from which this transition was triggered.
     * @return CallbackReturn::SUCCESS if hardware energizes successfully, CallbackReturn::FAILURE otherwise.
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Deactivates the hardware, commanding a safe shutdown of motor torque.
     * @details Sends a `Deactivate` command to the microcontroller to drop motor driver torque and waits
     *          for an echo confirmation.
     * @param previous_state The lifecycle state from which this transition was triggered.
     * @return CallbackReturn::SUCCESS if the deactivation handshake confirms, CallbackReturn::ERROR otherwise.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Cleans up internal states and releases hardware resources.
     * @details Safely closes the serial port connection and resets internal communication variables.
     * @param previous_state The lifecycle state from which this transition was triggered.
     * @return CallbackReturn::SUCCESS always upon cleanup completion.
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Performs an emergency shutdown of the hardware interface.
     * @details Immediately closes the serial port to ensure hardware fails safely during system shutdown.
     * @param previous_state The lifecycle state from which this transition was triggered.
     * @return CallbackReturn::SUCCESS always upon shutdown completion.
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Handles system errors and attempts automated resource recovery.
     * @details Triggered when a lifecycle or real-time loop fault occurs. Attempts to cleanly close
     *          the serial port connection so the system can be safely re-initialized.
     * @param previous_state The lifecycle state in which the error occurred.
     * @return CallbackReturn::SUCCESS if recovery/cleanup succeeded, CallbackReturn::FAILURE otherwise.
     */
    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

    // =========================================================================
    // hardware_interface::SystemInterface Implementation (ROS 2 Jazzy)
    // =========================================================================

    /**
     * @brief Exports immutable state interfaces to the `ros2_control` framework.
     * @details Combines position and velocity state handles from the `DifferentialDriveHandler`
     *          with angular velocity, and linear acceleration handles from the `ImuSensorHandler`.
     * @return A vector of shared pointers to read-only StateInterface objects.
     */
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
        override;

    /**
     * @brief Exports mutable command interfaces to the `ros2_control` framework.
     * @details Provides velocity command handles for the left and right wheel joints via the
     *          `DifferentialDriveHandler`.
     * * @return A vector of shared pointers to writable CommandInterface objects.
     */
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
        override;

    /**
     * @brief Reads asynchronous feedback from the serial port and updates state interfaces.
     * @details Pulls the latest `SystemStateData` packet from the transceiver buffer, processes
     *          dynamic sensor degradation (invalidating IMU interfaces if unavailable), and integrates
     *          wheel velocities over the elapsed loop time (`dt`) to update odometry.
     * @param time The current ROS time.
     * @param period The duration elapsed since the last read/write cycle.
     * @return hardware_interface::return_type::OK if successful, return_type::ERROR if read error thresholds are exceeded.
     */
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /**
     * @brief Writes velocity commands to the microcontroller via the serial protocol.
     * @details Retrieves target wheel velocities, computes the necessary `response_delay_ms` based on
     *          the control loop period and communication budget, and transmits the heartbeat `DiffDriveCommandData` packet.
     * @param time The current ROS time.
     * @param period The duration of the current control loop cycle.
     * @return hardware_interface::return_type::OK upon successful serial transmission.
     */
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
    using SystemMessageSerialProtocol = SerialMessageProtocol<DiffDriveMessageRegistry>;

    // Sub-system handlers
    DifferentialDriveHandler diff_drive_handler_; ///< Handles wheel joint math, odometry integration, and command formatting.
    ImuSensorHandler imu_handler_;                ///< Handles IMU data extraction and dynamic interface availability toggling.

    // Hardware communication members
    std::string port_;                            ///< Device path for the serial connection (e.g., "/dev/ttyACM0").
    double measured_roundtrip_ms_{0.0};           ///< Measured latency of a complete request/response serial cycle.
    size_t communication_budget_ms_{0};           ///< Total time allocated for serial overhead and sensor reading.
    size_t velocity_read_error_count_{0};         ///< Counter tracking consecutive missed or corrupted serial read packets.
    std::optional<uint8_t> response_delay_ms_{};  ///< Dynamic delay commanded to the MCU before it transmits its reply.
    SerialMessageTransceiver<SystemMessageSerialProtocol> transceiver_; ///< Low-level serial transceiver instance.

    // Private helper methods

    /**
     * @brief Pulls and processes the latest system state message from the transceiver.
     * @details Checks buffer availability, reads `SystemStateData`, updates subsystem handler states,
     *          and manages dynamic IMU degradation if `SystemStateFlags::ImuUnavailable` is asserted.
     * @return True if a message was successfully read and processed, false if unavailable or corrupted.
     */
    bool processSystemStateMessage();

    /**
     * @brief Safely terminates the serial port connection.
     * @return True if the port closed cleanly or was already closed, false if an exception occurred.
     */
    bool closeSerialConnection() noexcept;

    /**
     * @brief Calculates the required MCU response delay based on loop period and latency budget.
     * @param period The control loop duration provided by the resource manager.
     */
    void computeResponseDelay(const rclcpp::Duration& period);

    /**
     * @brief Transmits a data payload and waits for a specific typed response payload.
     * @template SendData The data structure type being transmitted.
     * @template ReceiveData The expected response data structure type (defaults to SendData for echo requests).
     * @param send_data The payload instance to transmit.
     * @param max_attempts Maximum number of transmission retry attempts before failing.
     * @param wait_time_ms Timeout duration in milliseconds to wait for a response per attempt.
     * @return An std::optional containing the received payload upon success, or std::nullopt upon failure.
     */
    template <typename SendData, typename ReceiveData = SendData>
    std::optional<ReceiveData> sendReceiveMessageData(const SendData& send_data,
                                                      const size_t max_attempts,
                                                      const size_t wait_time_ms = 100);

    /**
     * @brief Transmits a signal message by ID and waits for an identical echo acknowledgement.
     * @template TargetId The compile-time message identifier (`MsgId`) to transmit and await.
     * @param max_attempts Maximum number of transmission retry attempts before failing.
     * @return True if the target message ID was successfully acknowledged, false otherwise.
     */
    template <MsgId TargetId>
    bool sendReceiveMessage(const size_t max_attempts);
};

}  // namespace bumperbot_firmware

#endif  // ROBOT_SYSTEM_INTERFACE_HPP