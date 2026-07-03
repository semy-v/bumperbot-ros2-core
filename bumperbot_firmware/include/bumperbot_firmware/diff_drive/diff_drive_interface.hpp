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
#include "binary_message_protocol.hpp"
#include "diff_drive_serial_transceiver.hpp"


namespace bumperbot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DiffDriveInterface : public hardware_interface::SystemInterface
{
public:
  DiffDriveInterface();
  virtual ~DiffDriveInterface();

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

  DiffDriveSerialTransceiver<BinaryMessageProtocol> transceiver_{};
  TwoWheelsData wheels_data_{};
  size_t velocity_read_error_count_{0};
  size_t measured_roundtrip_ms_{0};
  size_t communication_budget_ms_{0};
  std::optional<uint8_t> response_delay_ms_{};

  // hardware parameters
  std::string port_;
  double wheels_min_velocity_;
  DiffDriveConfigData config_data_;
};


template<typename SendData, typename ReceiveData>
std::optional<ReceiveData> DiffDriveInterface::sendReceiveMessageData(
      const SendData& send_data, const size_t max_attempts, const size_t wait_time_ms)
{
  for (size_t attempt = 1; attempt <= max_attempts; attempt++) {
    transceiver_.writeMessage(send_data);

    measured_roundtrip_ms_ = waitDataAvailableToRead(wait_time_ms);

    ReceiveData response_data;
    if (transceiver_.readLastMessageData(response_data)) {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
        "Successfull req/resp attempt %zu took time %zu milliseconds"
          , attempt, measured_roundtrip_ms_);

      return std::optional<ReceiveData>{response_data};
    }
  }

  return std::nullopt;
}

template<MsgId TargetId>
bool DiffDriveInterface::sendReceiveMessage(const size_t max_attempts) {
  for (size_t attempt = 1; attempt <= max_attempts; attempt++) {
    transceiver_.template writeMessage<TargetId>();

    constexpr size_t kWaitTimeMs{100};
    measured_roundtrip_ms_ = waitDataAvailableToRead(kWaitTimeMs);

    if (transceiver_.template readLastMessage<TargetId>()) {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveInterface"),
        "Successfull req/resp attempt %zu took time %zu milliseconds"
          , attempt, measured_roundtrip_ms_);

      return true;
    }
  }

  return false;
}

}  // namespace bumperbot_firmware


#endif  // DIFF_DRIVE_INTERFACE_HPP