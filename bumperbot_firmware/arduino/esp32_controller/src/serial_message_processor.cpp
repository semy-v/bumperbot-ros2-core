#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <bit>

#include "serial_message_processor.hpp"

SerialInputProcessor::Result SerialInputProcessor::processAllSerialInputMessages(const MsgId expected_msg_id) {
    auto next_msg_result = processNextSerialInputMessage(expected_msg_id);
    bool valid_message_found = (Result::Success == next_msg_result);

    // MessageUnavailable is returned only if no serial input left
    // meaning all possible input messages are processed
    while (Result::MessageUnavailable != next_msg_result) {
        next_msg_result = processNextSerialInputMessage(expected_msg_id);
        // accumulate valid message success result
        valid_message_found |= (Result::Success == next_msg_result);
    }

    return valid_message_found ? Result::Success : next_msg_result;
}

SerialInputProcessor::Result SerialInputProcessor::processNextSerialInputMessage(const MsgId expected_msg_id) {
    Result result{Result::MessageUnavailable};

    while (Serial.available()) {
        const auto state = deserializer_.processByte(Serial.read());

        if (ProcessResult::SUCCESS == state) {
            const MsgId next_msg_id = deserializer_.getReceivedMessageId();
            if (expected_msg_id != AnyMsgId && expected_msg_id != next_msg_id) {
                // treat unexpected message as invalid
                return Result::MessageInvalid;
            }

            switch (next_msg_id) {
                case MsgId::DiffDriveConfig: {
                    const auto opt_config = deserializer_.getPayload<DiffDriveConfigData>();
                    if (!opt_config.has_value()) {
                        return Result::MessageInvalid;
                    }

                    // send config data to the control task
                    xQueueOverwrite(task_shared_data_.diff_drive_config_queue, &opt_config.value());

                    // send same config data message in response
                    sendSerialMessage(opt_config.value());

                    return Result::Success;
                }
                case MsgId::ImuConfig: {
                    const auto opt_imu_config = deserializer_.getPayload<ImuConfigData>();
                    if (!opt_imu_config.has_value()) {
                        return Result::MessageInvalid;
                    }

                    const SensorTaskEvent imu_config_event{
                        .id = SensorTaskEventId::ImuConfig,
                        .payload = opt_imu_config.value().calibrate_period_ms};

                    // Collisions with other event(s) not expected due to sequential request/response messages
                    // so we can safely overwrite the notification value with the new event
                    xTaskNotify(
                        task_shared_data_.sensor_read_task_handle,
                        std::bit_cast<uint32_t>(imu_config_event),
                        eSetValueWithOverwrite);

                    return Result::Success;
                }
                case MsgId::DiffDriveCommand: {
                    const auto opt_command = deserializer_.getPayload<DiffDriveCommandData>();
                    if (!opt_command.has_value()) {
                        return Result::MessageInvalid;
                    }

                    // send target velocity data to the control task
                    xQueueOverwrite(task_shared_data_.diff_drive_command_queue, &opt_command.value().velocity);

                    SensorTaskEvent sensor_read_event{
                        .id = SensorTaskEventId::SensorRead,
                        .payload = opt_command.value().response_delay_ms};

                    // Collisions with other event(s) not expected due to sequential request/response messages 
                    // so we can safely overwrite the notification value with the new event
                    xTaskNotify(
                        task_shared_data_.sensor_read_task_handle,
                        std::bit_cast<uint32_t>(sensor_read_event),
                        eSetValueWithOverwrite);

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
