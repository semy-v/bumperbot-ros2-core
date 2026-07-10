#ifndef MESSAGE_DESERIALIZE_HPP
#define MESSAGE_DESERIALIZE_HPP

#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "message_traits.hpp"

static_assert(
    std::endian::native == std::endian::little,
    "Message protocol requires unified endianness between communication parties");

// Represents the result of feeding a single byte into the deserializer
enum class ProcessResult {
    INCOMPLETE,    // Frame is not yet fully received
    SUCCESS,       // A valid frame was completely received and CRC verified
    ERROR_SYNC,    // Lost synchronization (invalid start byte or unexpected data)
    ERROR_LENGTH,  // Header length mismatch message type payload length
    ERROR_CRC      // Payload failed CRC validation
};

template <MessageRegistryConcept Registry>
class MessageStreamDeserializer {
 public:
    enum State : uint8_t { WAIT_FOR_START, WAIT_FOR_HEADER, WAIT_FOR_PAYLOAD };

    MessageStreamDeserializer() { reset(); }

    ProcessResult processByte(const uint8_t c) { return processByte(static_cast<std::byte>(c)); }

    ProcessResult processByte(const std::byte b) {
        switch (state_) {
            case State::WAIT_FOR_START:
                if (b == kStartByte) {
                    header_bytes_[0] = b;
                    rx_index_ = 1;
                    state_ = State::WAIT_FOR_HEADER;
                }
                return ProcessResult::INCOMPLETE;

            case State::WAIT_FOR_HEADER:
                header_bytes_[rx_index_++] = b;

                // If received the full header
                if (rx_index_ == sizeof(MessageHeader)) {
                    // Verify ID exists and length matches compile-time registry exactly
                    if (!Registry::isValidPayloadSize(header_.msg_id, header_.payload_length)) {
                        reset();
                        return ProcessResult::ERROR_LENGTH;
                    }

                    if (header_.payload_length == 0) {
                        // Edge case: Message has no payload, just a header
                        return processHeaderOnly();
                    }

                    rx_index_ = 0;  // Reset index to start filling the payload buffer
                    state_ = State::WAIT_FOR_PAYLOAD;
                }
                return ProcessResult::INCOMPLETE;

            case State::WAIT_FOR_PAYLOAD:
                payload_buffer_[rx_index_++] = b;

                // If received exactly the number of bytes advertised in the header
                if (rx_index_ == header_.payload_length) {
                    // ref_header can't be nullptr due to previous isValidPayloadSize call
                    const MessageHeader& ref_header = *Registry::getMessageHeader(header_.msg_id);
                    const uint16_t expected_crc = calculateCRC16(
                        std::span{payload_buffer_.data(), header_.payload_length}, ref_header.crc);
                    // Validate Checksum
                    if (expected_crc == header_.crc) {
                        // Frame is valid and ready to be retrieved
                        state_ = State::WAIT_FOR_START;
                        return ProcessResult::SUCCESS;
                    } else {
                        // Frame corrupted
                        reset();
                        return ProcessResult::ERROR_CRC;
                    }
                }
                return ProcessResult::INCOMPLETE;
        }
        reset();
        return ProcessResult::ERROR_SYNC;
    }

    // Query what type of message just arrived
    // (call this after ProcessResult::SUCCESS)
    MsgId getReceivedMessageId() const { return header_.msg_id; }

    // Returns false if the requested type
    // doesn't match the received Message ID
    template <typename TData>
    std::optional<TData> getPayload() const {
        static_assert(Registry::template isValidPayload<TData>(),
                      "Requested payload type not supported");

        if (Registry::template getPayloadMsgId<TData>() != header_.msg_id) {
            return std::nullopt;
        }

        if (sizeof(TData) != header_.payload_length) {
            return std::nullopt;
        }

        TData out_data;
        std::memcpy(&out_data, payload_buffer_.data(), sizeof(TData));

        return out_data;
    }

    // Force a manual reset of the stream parsing state
    void reset() {
        state_ = State::WAIT_FOR_START;
        rx_index_ = 0;
    }

 private:
    static constexpr size_t kMaxMessagePayloadSize =
        (Registry::max_payload_size > 0 ? Registry::max_payload_size : 1);

    State state_;
    std::size_t rx_index_;

    MessageHeader header_;
    std::span<std::byte, sizeof(MessageHeader)> header_bytes_{
        std::as_writable_bytes(std::span{&header_, 1})};

    std::array<std::byte, kMaxMessagePayloadSize> payload_buffer_;

    ProcessResult processHeaderOnly() {
        // expected_header can't be nullptr due to previous isValidPayloadSize call
        const MessageHeader& expected_header = *Registry::getMessageHeader(header_.msg_id);
        if (expected_header == header_) {
            state_ = State::WAIT_FOR_START;
            return ProcessResult::SUCCESS;
        }
        reset();  // reset if message corrupted
        return ProcessResult::ERROR_CRC;
    }
};

#endif  // MESSAGE_DESERIALIZE_HPP