#ifndef DIFF_DRIVE_SERIAL_TRANSCEIVER_HPP
#define DIFF_DRIVE_SERIAL_TRANSCEIVER_HPP

#include <vector>
#include <string>
#include <algorithm>
#include <libserial/SerialPort.h>

#include "protocol_codec_concept.hpp"

namespace bumperbot_firmware {

template <ProtocolCodec CodecPolicy>
class DiffDriveSerialTransceiver {
public:
    DiffDriveSerialTransceiver() {
        constexpr size_t kMaxReceiveBufferSize{256};
        constexpr size_t kMaxErrorMessageLength{64};

        receive_buffer_.reserve(kMaxReceiveBufferSize);
        error_message_.reserve(kMaxErrorMessageLength);
    }

    bool isDataAvailable() {
        // return serial_.IsDataAvailable();
        return serial_.GetNumberOfBytesAvailable();
    }

    void openPort(const std::string& port, const LibSerial::BaudRate& baudRate) {
        serial_.Open(port);
        serial_.SetBaudRate(baudRate);
    }

    void closePort() {
        if (serial_.IsOpen()) {
            serial_.Close();
        }
    }

    const std::string& lastErrorMessage() const {
        return error_message_;
    }

    template <typename TData>
    void writeMessage(const TData& data) {
        serial_.Write(
            codec_.template serializeMessage<TData>(data));
    }

    template <MsgId TargetId>
    void writeMessage() {
        serial_.Write(codec_.template serializeMessage<TargetId>());
    }

    template <typename TData>
    bool readLastMessageData(TData& out_data) {
        bool result{false};
        error_message_.clear();

        if (not receive_buffer_.empty()) {
            // consume possible remaining bytes from previous run
            // e.g. if the exception occured during serial read
            result = codec_.deserializeLastStreamMessage(
                receive_buffer_, out_data, error_message_);

            if (not receive_buffer_.empty()) {
                // not all bytes from previous run processed
                error_message_ = "Previous message process error";
                receive_buffer_.clear();
            }
        }

        if (readBytesAvailable()) {
            result |= codec_.deserializeLastStreamMessage(
                receive_buffer_, out_data, error_message_);
        }

        return result;
    }

    template <MsgId TargetId>
    bool readLastMessage() {
        bool result{false};
        error_message_.clear();

        if (not receive_buffer_.empty()) {
            // consume possible remaining bytes from previous run
            // e.g. if the exception occured during serial read
            result = codec_.template deserializeLastStreamMessage<TargetId>(
                receive_buffer_, error_message_);

            if (not receive_buffer_.empty()) {
                // not all bytes from previous run processed
                error_message_ = "Previous message process error";
                receive_buffer_.clear();
            }
        }

        if (readBytesAvailable()) {
            result |= codec_.template deserializeLastStreamMessage<TargetId>(
                receive_buffer_, error_message_);
        }

        return result;
    }

private:
    bool readBytesAvailable() {
        const size_t bytes_available = 
            serial_.GetNumberOfBytesAvailable();
        if (bytes_available == 0) {
            error_message_ = "No message available for read";
            return false;
        }

        // Limit read capacity to prevent buffer reallocation
        const size_t bytes_to_read = std::min(
            bytes_available, receive_buffer_.capacity());

        serial_.Read(receive_buffer_, bytes_to_read, 1);

        return true;
    }

    LibSerial::SerialPort serial_;
    CodecPolicy codec_; 
    std::vector<uint8_t> receive_buffer_;
    std::string error_message_;
};

} // namespace bumperbot_firmware

#endif // DIFF_DRIVE_SERIAL_TRANSCEIVER_HPP