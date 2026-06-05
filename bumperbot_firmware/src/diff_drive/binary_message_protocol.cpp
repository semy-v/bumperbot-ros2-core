#include "bumperbot_firmware/diff_drive/binary_message_protocol.hpp"

namespace bumperbot_firmware {

bool BinaryMessageProtocol::find_next_message(
    std::vector<uint8_t>& stream_buffer, std::string& error_message)
{
    if (stream_buffer.empty()) {
        error_message = "No message received";
        return false;
    }

    const size_t stream_size = stream_buffer.size();
    auto state{ProcessResult::INCOMPLETE};

    for (; stream_idx_ < stream_size; ++stream_idx_) {
        state = deserializer_.processByte(stream_buffer[stream_idx_]);
        if (ProcessResult::SUCCESS == state) {
            ++stream_idx_;
            return true;
        }
    }

    // Clear consumed buffer after all bytes processed
    stream_buffer.clear();

    if (valid_message_found_) {
        // Clean up the error message if succeeded
        // at any point in the stream
        error_message.clear();
    } else {
        processErrorState(state, error_message);
    }

    return false;
}

void BinaryMessageProtocol::processErrorState(
    const ProcessResult state, std::string& error_message)
{
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

} // namespace bumperbot_firmware