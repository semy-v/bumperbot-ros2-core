#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include "serial_message_processor.hpp"

SerialInputProcessor::Result SerialInputProcessor::processAllSerialInputMessages(
    const MsgId expected_msg_id) {
    auto next_msg_result = processNextSerialInputMessage(expected_msg_id);
    bool valid_message_found = (Result::Success == next_msg_result);

    while (Result::MessageUnavailable != next_msg_result) {
        next_msg_result = processNextSerialInputMessage(expected_msg_id);
        valid_message_found |= (Result::Success == next_msg_result);
    }

    return valid_message_found ? Result::Success : next_msg_result;
}

SerialInputProcessor::Result SerialInputProcessor::processNextSerialInputMessage(
    const MsgId expected_msg_id) {
    Result result{Result::MessageUnavailable};

    while (Serial.available()) {
        const auto state = deserializer_.processByte(Serial.read());

        if (ProcessResult::SUCCESS == state) {
            const MsgId next_msg_id = deserializer_.getReceivedMessageId();
            if (expected_msg_id != AnyMsgId && expected_msg_id != next_msg_id) {
                return Result::MessageInvalid;
            }

            switch (next_msg_id) {
                case MsgId::DiffDriveConfig: {
                    const auto opt_config = deserializer_.getPayload<DiffDriveConfigData>();
                    if (!opt_config || !opt_config->valid()) {
                        return Result::MessageInvalid;
                    }

                    xQueueOverwrite(task_shared_data_.diff_drive_config_queue, &*opt_config);
                    sendSerialMessage(*opt_config);
                    return Result::Success;
                }

                case MsgId::ImuConfig: {
                    const auto opt_config = deserializer_.getPayload<ImuConfigData>();
                    if (!opt_config) {
                        return Result::MessageInvalid;
                    }

                    xQueueOverwrite(task_shared_data_.imu_config_queue, &*opt_config);

                    if (task_shared_data_.sensor_read_task_handle == nullptr) {
                        return Result::MessageInvalid;
                    }

                    // Payload is already safely stored in imu_config_queue.
                    // Notification is only the wake-up event.
                    xTaskNotifyGive(task_shared_data_.sensor_read_task_handle);
                    return Result::Success;
                }

                case MsgId::DiffDriveCommand: {
                    const auto opt_command = deserializer_.getPayload<DiffDriveCommandData>();
                    if (!opt_command) {
                        return Result::MessageInvalid;
                    }

                    const int64_t received_time_us = esp_timer_get_time();
                    const SystemStateResponseRequest response_request{
                        .due_time_us =
                            received_time_us +
                            static_cast<int64_t>(opt_command->response_delay_ms) * 1000LL,
                    };

                    // Reserve the response before applying the command. Queue
                    // length is one, therefore a second command cannot silently
                    // replace a SystemState response that is still pending.
                    if (xQueueSend(task_shared_data_.system_state_response_queue,
                                   &response_request, 0) != pdPASS) {
                        return Result::MessageInvalid;
                    }

                    xQueueOverwrite(task_shared_data_.diff_drive_command_queue,
                                    &opt_command->velocity);
                    return Result::Success;
                }

                case MsgId::Deactivate:
                    if (task_shared_data_.diff_drive_control_task_handle != nullptr) {
                        xTaskNotifyGiveIndexed(task_shared_data_.diff_drive_control_task_handle,
                                               kDeactivateNotifyIndex);
                    }
                    sendSerialMessage<MsgId::Deactivate>();
                    return Result::Success;

                default:
                    return Result::MessageInvalid;
            }
        }

        if (ProcessResult::INCOMPLETE != state) {
            return Result::MessageInvalid;
        }
    }

    return result;
}