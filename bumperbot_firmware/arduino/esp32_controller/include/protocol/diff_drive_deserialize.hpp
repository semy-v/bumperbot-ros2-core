#ifndef DIFF_DRIVE_DESERIALIZE_HPP
#define DIFF_DRIVE_DESERIALIZE_HPP

#include <stdint.h>
#include <string.h>
#include "diff_drive_messages.hpp"

// Represents the result of feeding a single byte into the deserializer
enum class ProcessResult {
    INCOMPLETE,   // Frame is not yet fully received
    SUCCESS,      // A valid frame was completely received and CRC verified
    ERROR_SYNC,   // Lost synchronization (invalid start byte or unexpected data)
    ERROR_LENGTH, // Header length exceeded maximum allowed buffer size
    ERROR_CRC     // Payload failed CRC validation
};

template <typename Registry>
class MessageStreamDeserializer {
public:
    static_assert(is_message_registry<Registry>::value,
        "MessageStreamDeserializer must be instantiated with a MessageRegistry type");

    enum State {
        WAIT_FOR_START,
        WAIT_FOR_HEADER,
        WAIT_FOR_PAYLOAD
    };

    MessageStreamDeserializer() {
        reset();
    }

    // Core State Machine: Feed this method one byte at a time
    ProcessResult processByte(uint8_t c) {
        switch (state_) {
            case State::WAIT_FOR_START:
                if (c == kStartByte) {
                    header_buffer_[0] = c;
                    rx_index_ = 1;
                    state_ = State::WAIT_FOR_HEADER;
                }
                return ProcessResult::INCOMPLETE;

            case State::WAIT_FOR_HEADER:
                header_buffer_[rx_index_++] = c;

                // If received the full header
                if (rx_index_ == sizeof(MessageHeader)) {
                    memcpy(&current_header_, header_buffer_, sizeof(MessageHeader));

                    // Safety check: Ensure the advertised length doesn't overflow static buffer
                    if (current_header_.payload_length > Registry::max_payload_size) {
                        reset();
                        return ProcessResult::ERROR_LENGTH;
                    }

                    if (current_header_.payload_length == 0) {
                        // Edge case: Message has no payload, just a header
                        return processHeaderOnly();
                    }

                    rx_index_ = 0; // Reset index to start filling the payload buffer
                    state_ = State::WAIT_FOR_PAYLOAD;
                }
                return ProcessResult::INCOMPLETE;

            case State::WAIT_FOR_PAYLOAD:
                payload_buffer_[rx_index_++] = c;

                // If received exactly the number of bytes advertised in the header
                if (rx_index_ == current_header_.payload_length) {
                    // Validate Checksum
                    uint8_t expected_crc = calculateLRC8(
                        header_buffer_, kCrcOffset);
                    expected_crc ^= calculateLRC8(
                        payload_buffer_, current_header_.payload_length);

                    if (expected_crc == current_header_.crc) {
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
    MsgId getReceivedMessageId() const {
        return current_header_.msg_id;
    }

    // Returns false if the requested type
    // doesn't match the received Message ID
    template <typename TData>
    bool getPayload(TData& out_data) const {
        if (Registry::template getPayloadMsgId<TData>() != current_header_.msg_id) {
            return false;
        }
        
        if (sizeof(TData) != current_header_.payload_length) {
            return false;
        }

        memcpy(&out_data, payload_buffer_, sizeof(TData));
        return true;
    }

    // Force a manual reset of the stream parsing state
    void reset() {
        state_ = State::WAIT_FOR_START;
        rx_index_ = 0;
        memset(&current_header_, 0, sizeof(MessageHeader));
    }

private:
    ProcessResult processHeaderOnly() {
        if (calculateLRC8(header_buffer_, kCrcOffset) == current_header_.crc) {
            state_ = State::WAIT_FOR_START;
            return ProcessResult::SUCCESS;
        }
        reset(); // reset if message corrupted
        return ProcessResult::ERROR_CRC;
    }

    State state_;
    uint8_t rx_index_;
    MessageHeader current_header_;
    
    // Split buffers for clean memory layout
    uint8_t header_buffer_[sizeof(MessageHeader)];

    static constexpr size_t kMaxMessagePayloadSize =
        (Registry::max_payload_size > 0 ? Registry::max_payload_size : 1);
    uint8_t payload_buffer_[kMaxMessagePayloadSize];
};

#endif // DIFF_DRIVE_DESERIALIZE_HPP