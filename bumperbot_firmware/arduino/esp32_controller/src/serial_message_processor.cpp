#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "serial_message_processor.hpp"

SerialInputProcessor::Result SerialInputProcessor::processAllSerialInputMessages(const MsgId expected_msg_id) {
    auto next_msg_result = processNextSerialInputMessage(expected_msg_id);
  bool valid_message_found = (Result::Success == next_msg_result);

  // MessageUnavailable is returned only if no serial input left
  // meaning all possible input messages are processed
  while(Result::MessageUnavailable != next_msg_result) {
    next_msg_result = processNextSerialInputMessage(expected_msg_id);
    // accumulate valid message success result
    valid_message_found |= (Result::Success == next_msg_result);
  }

  return valid_message_found ? Result::Success : next_msg_result;
}

SerialInputProcessor::Result SerialInputProcessor::processNextSerialInputMessage(const MsgId expected_msg_id) {
  Result result{Result::MessageUnavailable};

  while(Serial.available()) {
    const auto state = deserializer_.processByte(Serial.read());

    if (ProcessResult::SUCCESS == state) {
      const MsgId next_msg_id = deserializer_.getReceivedMessageId();
      if (expected_msg_id != AnyMsgId && expected_msg_id != next_msg_id) {
        // treat unexpected message as invalid
        return Result::MessageInvalid;
      }

      switch (next_msg_id) {
        case MsgId::Config:
        {
          ConfigData config_data;
          if (!deserializer_.getPayload(config_data)) {
            return Result::MessageInvalid;
          }

          // send config data to the control task
          xQueueOverwrite(task_shared_data_.config_message_queue, &config_data);

          // send same config data message in response
          sendSerialMessage(config_data);

          return Result::Success;
        }
        case MsgId::Velocity:
        {
          VelocityData vel_data;
          if (!deserializer_.getPayload(vel_data)) {
            return Result::MessageInvalid;
          }

          // send target velocity data to the control task
          xQueueOverwrite(task_shared_data_.target_velocity_message_queue, &vel_data);

          // send actual wheel velocities in response
          vel_data.right_wheel_velocity = 0.0;
          vel_data.left_wheel_velocity = 0.0;
          xQueuePeek(task_shared_data_.current_velocity_queue, &vel_data, 0);
          sendSerialMessage(vel_data);

          return Result::Success;
        }
        case MsgId::Deactivate:
          if (nullptr != task_shared_data_.diff_drive_control_task_handle) {
            xTaskNotifyGiveIndexed(
                task_shared_data_.diff_drive_control_task_handle, kDeactivateNotifyIndex);
          }
          // send Deactivate message in response
          sendSerialMessage<MsgId::Deactivate>();
          return Result::Success;
        default:
          return Result::MessageInvalid;
      }
    } else if (ProcessResult::INCOMPLETE != state) {
      // treat all error states as invalid message
      return Result::MessageInvalid;
    }
  }

  return result;
}
