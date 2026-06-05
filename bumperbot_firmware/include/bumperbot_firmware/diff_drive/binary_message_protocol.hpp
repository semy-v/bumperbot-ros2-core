#ifndef BINARY_MESSAGE_PROTOCOL_HPP
#define BINARY_MESSAGE_PROTOCOL_HPP

#include <vector>
#include <string>
#include <format>

#include "diff_drive_deserialize.hpp"
#include "diff_drive_serialize.hpp"
#include "protocol_codec_concept.hpp"

namespace bumperbot_firmware {

class BinaryMessageProtocol {
public:
    BinaryMessageProtocol() {
        // reserve max possible frame size in internal buffer
        // to prevent buffer reallocations during serialization
        send_msg_buffer_.reserve(
            DiffDriveMessageSerializer::getMaxFrameSize());
    }

    ~BinaryMessageProtocol() = default;

    // Serialize Non-Zero Payload Message
    template <typename TData>
    const std::vector<uint8_t>& serializeMessage(const TData& data) {
        send_msg_buffer_.resize(DiffDriveMessageSerializer::getFrameSize<TData>());
        DiffDriveMessageSerializer::serialize(data, send_msg_buffer_.data());
        return send_msg_buffer_;
    }

    // Serialize Zero-Payload Message
    template <MsgId TargetId>
    const std::vector<uint8_t>& serializeMessage() {
        send_msg_buffer_.resize(DiffDriveMessageSerializer::template getFrameSize<TargetId>());
        DiffDriveMessageSerializer::template serialize<TargetId>(send_msg_buffer_.data());
        return send_msg_buffer_;
    }

    template <typename TData>
    bool deserializeLastStreamMessage(std::vector<uint8_t>& stream_buffer,
                                      TData& out_data,
                                      std::string& error_message)
    {
        // reset deserialize stream state
        stream_idx_ = 0;
        valid_message_found_ = false;

        while(find_next_message(stream_buffer, error_message)) {
            const auto msg_id = deserializer_.getReceivedMessageId();
            if (msg_id == DiffDriveMessageRegistry::getPayloadMsgId<TData>()) {
                if (deserializer_.getPayload(out_data)) {
                    valid_message_found_ = true;
                } else {
                    error_message = "Invalid message payload length";
                }
            } else {
                error_message = "Invalid message id";
            }
        }

        // Return true if any valid message found,
        // even if the stream message is INCOMPLETE
        return valid_message_found_;
    }

    template <MsgId TargetId>
    bool deserializeLastStreamMessage(std::vector<uint8_t>& stream_buffer,
                                      std::string& error_message)
    {
        static_assert(DiffDriveMessageRegistry::template isZeroPayload<TargetId>(),
            "This overload is strictly for zero-payload messages");

        // reset deserialize stream state
        stream_idx_ = 0;
        valid_message_found_ = false;

        while(find_next_message(stream_buffer, error_message)) {
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
    using DiffDriveMessageSerializer = MessageSerializer<DiffDriveMessageRegistry>;
    using DiffDriveMessageStreamDeserializer = MessageStreamDeserializer<DiffDriveMessageRegistry>;

    bool find_next_message(std::vector<uint8_t>& stream_buffer, std::string& error_message);
    void processErrorState(const ProcessResult state, std::string& error_message);

    // deserialize members
    size_t stream_idx_{0};
    bool valid_message_found_{false};
    DiffDriveMessageStreamDeserializer deserializer_;

    // serialize members
    std::vector<uint8_t> send_msg_buffer_;
};

static_assert(ProtocolCodec<BinaryMessageProtocol>,
    "BinaryMessageProtocol must fulfill ProtocolCodec concept requirements!");

} // namespace bumperbot_firmware

#endif // BINARY_MESSAGE_PROTOCOL_HPP