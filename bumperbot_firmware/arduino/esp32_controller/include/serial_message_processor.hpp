#ifndef SERIAL_MESSAGE_PROCESSOR_HPP
#define SERIAL_MESSAGE_PROCESSOR_HPP

#include <Arduino.h>

#include "protocol/diff_drive_messages.hpp"
#include "protocol/diff_drive_serialize.hpp"
#include "protocol/diff_drive_deserialize.hpp"
#include "task_shared_data.hpp"

// Serial output message process
using DiffDriveMessageSerializer = MessageSerializer<DiffDriveMessageRegistry>;

template<typename TData>
void sendSerialMessage(const TData& message_data) {
    uint8_t message[DiffDriveMessageSerializer::getFrameSize<TData>()];
    DiffDriveMessageSerializer::serialize(message_data, message);
    Serial.write(message, sizeof(message));
}

template<MsgId Id>
void sendSerialMessage() {
    uint8_t message[DiffDriveMessageSerializer::template getFrameSize<Id>()];
    DiffDriveMessageSerializer::template serialize<Id>(message);
    Serial.write(message, sizeof(message));
}


// Serial input message process
constexpr MsgId AnyMsgId = MsgId::End;

class SerialInputProcessor {
public:
    enum Result {
        Success = 0,
        MessageUnavailable = 1,
        MessageInvalid = 2
    };

    SerialInputProcessor(TaskSharedData& shared_data)
        : task_shared_data_(shared_data) 
    {}

    ~SerialInputProcessor() = default;

    SerialInputProcessor(const SerialInputProcessor&) = delete;
    SerialInputProcessor(SerialInputProcessor&&) = delete;
    SerialInputProcessor& operator=(const SerialInputProcessor&) = delete;
    SerialInputProcessor& operator=(SerialInputProcessor&&) = delete;

    Result processNextSerialInputMessage(const MsgId expected_msg_id = AnyMsgId);
    Result processAllSerialInputMessages(const MsgId expected_msg_id = AnyMsgId);

private:
    TaskSharedData& task_shared_data_;
    MessageStreamDeserializer<DiffDriveMessageRegistry> deserializer_{};
};

#endif // SERIAL_MESSAGE_PROCESSOR_HPP
