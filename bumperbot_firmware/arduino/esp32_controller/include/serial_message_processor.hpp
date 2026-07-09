#ifndef SERIAL_MESSAGE_PROCESSOR_HPP
#define SERIAL_MESSAGE_PROCESSOR_HPP

#include <Arduino.h>
#include <array>
#include "protocol/system_messages.hpp"
#include "protocol/message_serialize.hpp"
#include "protocol/message_deserialize.hpp"
#include "task_shared_data.hpp"

// Serial output message process
using DiffDriveMessageSerializer = MessageSerializer<DiffDriveMessageRegistry>;

template<typename TData>
void sendSerialMessage(const TData& message_data) {
    std::array<uint8_t, DiffDriveMessageSerializer::getFrameSize<TData>()> serial_message;
    DiffDriveMessageSerializer::serialize(message_data, serial_message);
    Serial.write(serial_message.data(), serial_message.size());
}

template<MsgId Id>
void sendSerialMessage() {
    auto serial_message = DiffDriveMessageSerializer::template serialize<Id>();
    Serial.write(serial_message.data(), serial_message.size());
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
