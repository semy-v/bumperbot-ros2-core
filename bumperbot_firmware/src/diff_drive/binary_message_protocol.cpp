#include "bumperbot_firmware/diff_drive/binary_message_protocol.hpp"

namespace bumperbot_firmware {

bool BinaryMessageProtocol::deserializeNextMessage(
    std::vector<uint8_t>& stream_buffer, std::string& error_message)
{
    if (stream_buffer.empty()) {
        error_message = "No message received";
        return false;
    }

    const size_t stream_size{stream_buffer.size()};
    auto state{ProcessResult::INCOMPLETE};
    for(; track_idx_ < stream_size; track_idx_++) {
        state = deserializer_.processByte(stream_buffer[track_idx_]);
        if (ProcessResult::SUCCESS == state) {
            error_message.clear();
            return true;
        }
        if (state != ProcessResult::INCOMPLETE) {
            processErrorState(state, error_message);
        }
    }

    // Clear the local stream_buffer as all the bytes were fed into statefull deserializer_.
    stream_buffer.clear();
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