#ifndef SERIAL_MESSAGE_PROTOCOL_HPP
#define SERIAL_MESSAGE_PROTOCOL_HPP

#include <string>
#include <vector>

#include "message_deserialize.hpp"
#include "message_serialize.hpp"
#include "system_messages.hpp"

namespace bumperbot_firmware {

template <MessageRegistryConcept Registry>
class SerialMessageProtocol {
 public:
    SerialMessageProtocol() {
        // reserve max possible frame size in internal buffer
        // to prevent buffer reallocations during serialization
        send_msg_buffer_.reserve(SystemMessageSerializer::getMaxFrameSize());
    }

    ~SerialMessageProtocol() = default;

    // Get frame size for message with payload
    template <typename TData>
    static constexpr size_t getFrameSize() {
        return SystemMessageSerializer::template getFrameSize<TData>();
    }

    // Get frame size for message with zero-payload
    template <MsgId TargetId>
    static constexpr size_t getFrameSize() {
        return SystemMessageSerializer::template getFrameSize<TargetId>();
    }

    void reset() { deserializer_.reset(); }

    // Serialize Non-Zero Payload Message
    template <typename TData>
    const std::vector<uint8_t>& serializeMessage(const TData& data) {
        send_msg_buffer_.resize(SystemMessageSerializer::template getFrameSize<TData>());
        SystemMessageSerializer::template serialize(data, send_msg_buffer_);
        return send_msg_buffer_;
    }

    // Serialize Zero-Payload Message
    template <MsgId TargetId>
    const std::vector<uint8_t>& serializeMessage() {
        std::span<const uint8_t> serial_message =
            SystemMessageSerializer::template serialize<TargetId>();
        send_msg_buffer_.assign(serial_message.begin(), serial_message.end());
        return send_msg_buffer_;
    }

    template <typename TData>
    std::optional<TData> deserializeLastStreamMessage(std::vector<uint8_t>& stream_buffer,
                                                      std::string& error_message) {
        // reset deserialize stream state
        track_idx_ = 0;
        valid_message_found_ = false;
        std::optional<TData> out_data{};

        while (deserializeNextMessage(stream_buffer, error_message)) {
            const auto msg_id = deserializer_.getReceivedMessageId();
            if (msg_id == Registry::template getPayloadMsgId<TData>()) {
                if (out_data = deserializer_.template getPayload<TData>(); out_data.has_value()) {
                    valid_message_found_ = true;
                } else {
                    error_message = "Invalid message payload length";
                }
            } else {
                error_message = "Invalid message id";
            }
        }

        // Return last found message even if
        // overall stream message is INCOMPLETE
        return out_data;
    }

    template <MsgId TargetId>
    bool deserializeLastStreamMessage(std::vector<uint8_t>& stream_buffer,
                                      std::string& error_message) {
        static_assert(Registry::template isZeroPayload<TargetId>(),
                      "This overload is strictly for zero-payload messages");

        // reset deserialize stream state
        track_idx_ = 0;
        valid_message_found_ = false;

        while (deserializeNextMessage(stream_buffer, error_message)) {
            if (TargetId == deserializer_.getReceivedMessageId()) {
                valid_message_found_ = true;
            } else {
                error_message = "Invalid message id";
            }
        }

        // Return true if any valid message found,
        // even if the stream message is INCOMPLETE
        return valid_message_found_;
    }

 private:
    using SystemMessageSerializer = MessageSerializer<Registry>;
    using SystemMessageStreamDeserializer = MessageStreamDeserializer<Registry>;

    size_t track_idx_{0};
    bool valid_message_found_{false};
    SystemMessageStreamDeserializer deserializer_;

    // serialize members
    std::vector<uint8_t> send_msg_buffer_;

    bool deserializeNextMessage(std::vector<uint8_t>& stream_buffer, std::string& error_message) {
        if (stream_buffer.empty()) {
            error_message = "No message received";
            return false;
        }

        const size_t stream_size{stream_buffer.size()};
        auto state{ProcessResult::INCOMPLETE};
        for (; track_idx_ < stream_size; track_idx_++) {
            state = deserializer_.processByte(stream_buffer[track_idx_]);
            if (ProcessResult::SUCCESS == state) {
                error_message.clear();
                return true;
            }
            if (state != ProcessResult::INCOMPLETE) {
                processErrorState(state, error_message);
            }
        }

        // Clear the local stream_buffer as all the bytes
        // were pprocessed by statefull deserializer_.
        stream_buffer.clear();
        return false;
    }

    void processErrorState(const ProcessResult state, std::string& error_message) {
        switch (state) {
            case ProcessResult::ERROR_CRC:
                error_message = "CRC Validation failed";
                break;
            case ProcessResult::ERROR_SYNC:
                error_message = "Message process sync lost";
                break;
            case ProcessResult::ERROR_LENGTH:
                error_message = "Too big message header length";
                break;
            default:
                // ignore not error states
                break;
        }
    }
};

}  // namespace bumperbot_firmware

#endif  // SERIAL_MESSAGE_PROTOCOL_HPP