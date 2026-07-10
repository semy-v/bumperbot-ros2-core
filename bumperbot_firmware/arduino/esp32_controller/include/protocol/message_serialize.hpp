#ifndef MESSAGE_SERIALIZE_HPP
#define MESSAGE_SERIALIZE_HPP

#include <algorithm>
#include <cstring>
#include <span>

#include "message_traits.hpp"

static_assert(
    std::endian::native == std::endian::little,
    "Message protocol requires unified endianness between communication parties");

template <MessageRegistryConcept Registry>
class MessageSerializer {
 public:
    static constexpr size_t getMaxFrameSize() {
        return sizeof(MessageHeader) + Registry::max_payload_size;
    }

    // Get frame size for message with payload
    template <typename TData>
    static constexpr size_t getFrameSize() {
        static_assert(Registry::template isValidPayload<TData>(),
                      "Frame size requested for a Payload Type not present in MessageRegistry");

        return sizeof(MessageHeader) + sizeof(TData);
    }

    // Get frame size for message with zero-payload
    template <MsgId TargetId>
    static constexpr size_t getFrameSize() {
        static_assert(Registry::template isZeroPayload<TargetId>(),
                      "Frame size by MsgId only supported for zero-payload messages");

        return sizeof(MessageHeader);
    }

    // Serialize message with non-zero payload and fixed-size output buffer
    template <typename TData, std::size_t N>
    static constexpr void serialize(const TData& data, std::array<uint8_t, N>& serial_message) {
        static_assert(N == getFrameSize<TData>(), "Buffer size contract violated");

        constexpr auto msg_id = Registry::template getPayloadMsgId<TData>();
        constexpr auto& header = Registry::template getMessageHeader<msg_id>();

        auto payload_bytes = std::as_bytes(std::span{&data, 1});
        auto header_bytes = Registry::template getHeaderBytes<msg_id>();
        auto message_bytes = std::span{serial_message};

        serializeHeader(header, header_bytes, payload_bytes, message_bytes);
        std::memcpy(&serial_message[header_bytes.size()], &data, sizeof(data));
    }

    // Serialize message with non-zero payload and dynamic extend output span
    template <typename TData>
    static constexpr void serialize(const TData& data, std::span<uint8_t> serial_message) {
        assert(serial_message.size() == getFrameSize<TData>());  // buffer size contract check

        constexpr auto msg_id = Registry::template getPayloadMsgId<TData>();
        constexpr auto& header = Registry::template getMessageHeader<msg_id>();

        auto payload_bytes = std::as_bytes(std::span{&data, 1});
        auto header_bytes = Registry::template getHeaderBytes<msg_id>();

        serializeHeader(header, header_bytes, payload_bytes, serial_message);
        std::memcpy(&serial_message[header_bytes.size()], &data, sizeof(data));
    }

    // Serialize message with Zero-Payload
    template <MsgId TargetId>
    static constexpr std::span<const uint8_t> serialize() {
        static_assert(Registry::template isZeroPayload<TargetId>(),
                      "Serialize called with an invalid or payload-bearing MsgId");

        return Registry::template getHeaderBytes<TargetId>();
    }

 private:
    static constexpr void serializeHeader(const MessageHeader& header,
                                          std::span<const uint8_t> header_bytes,
                                          std::span<const std::byte> payload_bytes,
                                          std::span<uint8_t> serial_message) {
        std::copy(header_bytes.begin(), header_bytes.end(), serial_message.begin());
        const uint16_t message_crc = calculateCRC16(payload_bytes, header.crc);
        std::memcpy(&serial_message[kCrcOffset], &message_crc, sizeof(message_crc));
    }
};

#endif  // MESSAGE_SERIALIZE_HPP