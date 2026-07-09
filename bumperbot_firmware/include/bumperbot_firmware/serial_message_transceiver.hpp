#ifndef SERIAL_MESSAGE_TRANSCEIVER_HPP
#define SERIAL_MESSAGE_TRANSCEIVER_HPP

#include <libserial/SerialPort.h>
#include <algorithm>
#include <format>
#include <string>
#include <vector>

namespace bumperbot_firmware {

namespace {
constexpr size_t kMinReadWaitTimeMs{1};
}  // namespace

template <typename SerialProtocol>
class SerialMessageTransceiver {
   public:
    SerialMessageTransceiver() {
        constexpr size_t kReserveBufferSize{256};
        constexpr size_t kMaxErrorMessageLength{64};

        receive_buffer_.reserve(kReserveBufferSize);
        error_message_.reserve(kMaxErrorMessageLength);
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

    [[nodiscard]] bool isDataAvailable() { return serial_.GetNumberOfBytesAvailable(); }

    [[nodiscard]] const std::string& lastErrorMessage() const { return error_message_; }

    template <typename TData>
    void writeMessage(const TData& data) {
        writeRaw(protocol_.template serializeMessage<TData>(data));
    }

    template <MsgId TargetId>
    void writeMessage() {
        writeRaw(protocol_.template serializeMessage<TargetId>());
    }

    template <typename TData>
    std::optional<TData> waitForNextMessageData(const size_t wait_time_ms) {
        // If ANY message already in the buffer, then read and discard ALL of them
        readLastMessageData<TData>();

        // Wait and read next expected message
        const size_t expected_bytes = protocol_.template getFrameSize<TData>();
        if (!readExactBytes(expected_bytes, wait_time_ms)) {
            return std::nullopt;
        }

        return protocol_.template deserializeLastStreamMessage<TData>(receive_buffer_,
                                                                      error_message_);
    }

    template <MsgId TargetId>
    bool waitForNextMessage(const size_t wait_time_ms) {
        // If ANY message already in the buffer, then read and discard ALL of them
        readLastMessage<TargetId>();

        // Wait and read next expected message
        const size_t expected_bytes = protocol_.template getFrameSize<TargetId>();
        if (!readExactBytes(expected_bytes, wait_time_ms)) {
            return false;
        }

        return protocol_.template deserializeLastStreamMessage<TargetId>(receive_buffer_,
                                                                         error_message_);
    }

    template <typename TData>
    std::optional<TData> readLastMessageData() {
        return processStream<std::optional<TData>>([this](std::optional<TData>& result) {
            result = protocol_.template deserializeLastStreamMessage<TData>(receive_buffer_,
                                                                            error_message_);
            return result.has_value();
        });
    }

    template <MsgId TargetId>
    bool readLastMessage() {
        return processStream<bool>([this](bool& result) {
            result = protocol_.template deserializeLastStreamMessage<TargetId>(receive_buffer_,
                                                                               error_message_);
            return result;
        });
    }

   private:
    LibSerial::SerialPort serial_;
    SerialProtocol protocol_;
    std::vector<uint8_t> receive_buffer_;
    std::string error_message_;

    void setError(std::string_view msg) { error_message_ = msg; }

    void writeRaw(const std::vector<uint8_t>& payload) { serial_.Write(payload); }

    bool readExactBytes(size_t bytes_to_read, size_t wait_time_ms) {
        if (bytes_to_read == 0) {
            return true;
        }

        try {
            serial_.Read(receive_buffer_, bytes_to_read, wait_time_ms);
            return true;
        } catch (const LibSerial::ReadTimeout&) {
            setError("Serial read timeout");
            return false;
        } catch (const std::exception& e) {
            setError(std::format("Unexpected read error: {}", e.what()));
            return false;
        }
    }

    void readAvailableBytes() {
        const size_t available = serial_.GetNumberOfBytesAvailable();
        if (available > 0) {
            serial_.Read(receive_buffer_, available, kMinReadWaitTimeMs);
        }
    }

    template <typename ReturnType, typename DeserializerFunc>
    ReturnType processStream(DeserializerFunc&& deserialize) {
        error_message_.clear();
        ReturnType latest_response{};

        // Pull ALL currently available bytes from the OS serial buffer into receive_buffer_.
        // Note: Any incomplete frame bytes from the previous cycle are ALREADY safely stored
        // inside protocol_.deserializer_'s internal state machine.
        readAvailableBytes();

        // Drain stream_buffer internally and return the newest frame.
        // Incomplete trailing bytes are ingested into the stateful deserializer,
        // and receive_buffer_ is guaranteed to be left empty for the next read cycle.
        deserialize(latest_response);

        return latest_response;
    }
};

}  // namespace bumperbot_firmware

#endif  // SERIAL_MESSAGE_TRANSCEIVER_HPP