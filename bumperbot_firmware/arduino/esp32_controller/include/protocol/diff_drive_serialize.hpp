#ifndef DIFF_DRIVE_SERIALIZE_HPP
#define DIFF_DRIVE_SERIALIZE_HPP

#include "diff_drive_messages.hpp"

template<typename Registry>
class MessageSerializer {
public:
    static_assert(is_message_registry<Registry>::value,
        "MessageSerializer must be instantiated with a MessageRegistry type");

    static constexpr size_t getMaxFrameSize() {
        return sizeof(MessageHeader) + Registry::max_payload_size;
    }

    // Get frame size for message with payload
    template<typename TData>
    static constexpr size_t getFrameSize() {
        static_assert(Registry::template isValidPayload<TData>(),
            "Frame size requested for a Payload Type not present in MessageRegistry");

        return sizeof(MessageHeader) + sizeof(TData);
    }

    // Get frame size for message with zero-payload
    template<MsgId TargetId>
    static constexpr size_t getFrameSize() {
        static_assert(Registry::template isZeroPayload<TargetId>(),
            "Frame size by MsgId only supported for zero-payload messages");
        return sizeof(MessageHeader);
    }

    // Serialize message with Payloads
    template<typename TData>
    static void serialize(const TData& data, uint8_t* out_buffer) {
        constexpr MsgId msg_id = Registry::template getPayloadMsgId<TData>();
        constexpr uint8_t payload_len = static_cast<uint8_t>(sizeof(TData));

        uint8_t& out_crc = serializeHeader(msg_id, payload_len, out_buffer);
        // append payload CRC value to result buffer CRC reference
        out_crc ^= calculateLRC8(reinterpret_cast<const uint8_t*>(&data), payload_len);
        // copy the remaining payload data to the result buffer
        memcpy(out_buffer + sizeof(MessageHeader), &data, sizeof(TData));
    }

    // Serialize message with Zero-Payload
    template <MsgId TargetId>
    static void serialize(uint8_t* out_buffer) {
        static_assert(Registry::template isZeroPayload<TargetId>(),
            "Serialize called with an invalid or payload-bearing MsgId");

        // serialize header with TargetId to the result buffer
        static_cast<void>(serializeHeader(TargetId, 0, out_buffer));
    }

private:
    static uint8_t& serializeHeader(const MsgId id, const uint8_t payload_length, uint8_t* out_buffer) {
        MessageHeader header{
            .start_byte = kStartByte,
            .msg_id = id,
            .payload_length = payload_length,
            .crc = 0
        };

        const uint8_t* header_buf = reinterpret_cast<const uint8_t*>(&header);
        header.crc = calculateLRC8(header_buf, kCrcOffset);

        // copy the FULL header with CRC computed into the result buffer
        memcpy(out_buffer, header_buf, sizeof(header));

        // Return a pointer to the CRC byte sitting inside the result buffer
        // so the payload serializer can mutate it safely.
        return out_buffer[kCrcOffset];
    }
};

#endif // DIFF_DRIVE_SERIALIZE_HPP