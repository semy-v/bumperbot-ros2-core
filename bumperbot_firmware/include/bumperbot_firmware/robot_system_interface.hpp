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

class RobotSystemInterface : public hardware_interface::SystemInterface {
 public:
    RobotSystemInterface() = default;
    virtual ~RobotSystemInterface();

	RobotSystemInterface(const RobotSystemInterface&) = delete;
    RobotSystemInterface(RobotSystemInterface&&) = delete;
    RobotSystemInterface& operator=(const RobotSystemInterface&) = delete;
    RobotSystemInterface& operator=(RobotSystemInterface&&) = delete;

    // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
    CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

    // Implementing hardware_interface::SystemInterface
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
        override;
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
        override;

    hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

 private:
    using SystemMessageSerialProtocol = SerialMessageProtocol<DiffDriveMessageRegistry>;

    // Sub-system handlers
    DifferentialDriveHandler diff_drive_handler_;
    ImuSensorHandler imu_handler_;

    // Hardware communication members
    std::string port_;
    double measured_roundtrip_ms_{0.0};
    size_t communication_budget_ms_{0};
    size_t velocity_read_error_count_{0};
    std::optional<uint8_t> response_delay_ms_{};
    SerialMessageTransceiver<SystemMessageSerialProtocol> transceiver_{};

    // Private methods
    bool processSystemStateMessage();
    bool closeSerialConnection() noexcept;
    void computeResponseDelay(const rclcpp::Duration& period);

    template <typename SendData, typename ReceiveData = SendData>
    std::optional<ReceiveData> sendReceiveMessageData(const SendData& send_data,
                                                      const size_t max_attempts,
                                                      const size_t wait_time_ms = 100);

    template <MsgId TargetId>
    bool sendReceiveMessage(const size_t max_attempts);
};

}  // namespace bumperbot_firmware

#endif  // ROBOT_SYSTEM_INTERFACE_HPP