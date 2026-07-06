#ifndef DIFF_DRIVE_INTERFACE_HPP
#define DIFF_DRIVE_INTERFACE_HPP

#include <vector>
#include <string>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "diff_drive_data.hpp"
#include "diff_drive_messages.hpp"
#include "serial_message_protocol.hpp"
#include "serial_message_transceiver.hpp"


namespace bumperbot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotSystemInterface : public hardware_interface::SystemInterface
{
public:
  RobotSystemInterface();
  virtual ~RobotSystemInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  struct WheelData {
    double velocity_command;
    double velocity_state;
    double position_state;
  };

  using TwoWheelsData = std::array<WheelData, 2>;
  using SystemMessageSerialProtocol = SerialMessageProtocol<DiffDriveMessageRegistry>;

  bool validateWheelJointNamesConfig() const;
  size_t waitDataAvailableToRead(const size_t wait_time_ms);
  bool processVelocityStateMessage();
  bool closeSerialConnection() noexcept;
  void computeResponseDelay(const rclcpp::Duration & period);

  template<typename SendData, typename ReceiveData = SendData>
  std::optional<ReceiveData> sendReceiveMessageData(
      const SendData& send_data, const size_t max_attempts, const size_t wait_time_ms = 100);

  template<MsgId TargetId>
  bool sendReceiveMessage(const size_t max_attempts);

  SerialMessageTransceiver<SystemMessageSerialProtocol> transceiver_{};
  TwoWheelsData wheels_data_{};
  size_t velocity_read_error_count_{0};
  double measured_roundtrip_ms_{0};
  size_t communication_budget_ms_{0};
  std::optional<uint8_t> response_delay_ms_{};

  // hardware parameters
  std::string port_;
  double wheels_min_velocity_;
  DiffDriveConfigData config_data_;
};

}  // namespace bumperbot_firmware


#endif  // DIFF_DRIVE_INTERFACE_HPP