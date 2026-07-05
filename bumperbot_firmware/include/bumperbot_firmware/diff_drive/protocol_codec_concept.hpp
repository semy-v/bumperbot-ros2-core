#ifndef PROTOCOL_CODEC_CONCEPT_HPP
#define PROTOCOL_CODEC_CONCEPT_HPP

#include <vector>
#include <string>
#include <concepts>
#include <cstdint>
#include <optional>
#include "diff_drive_messages.hpp"

namespace bumperbot_firmware {

template <typename T>
concept ProtocolCodec = requires(T codec, 
                                 const DiffDriveConfigData& pid_data,
                                 const ImuConfigData& imu_config,
                                 const DiffDriveCommandData& diff_drive_cmd,
                                 std::vector<uint8_t>& stream_buffer, 
                                 DiffDriveStateData& diff_drive_state, 
                                 std::string& error_message) 
{
    // Verify that a reset function exists
    { codec.reset() } -> std::same_as<void>;

    // Verify that a generic template serialize function exists for all required types
    { codec.template serializeMessage<DiffDriveConfigData>(pid_data) } -> std::same_as<const std::vector<uint8_t>&>;
    { codec.template serializeMessage<ImuConfigData>(imu_config) } -> std::same_as<const std::vector<uint8_t>&>;
    { codec.template serializeMessage<DiffDriveCommandData>(diff_drive_cmd) } -> std::same_as<const std::vector<uint8_t>&>;
    { codec.template serializeMessage<MsgId::Deactivate>() } -> std::same_as<const std::vector<uint8_t>&>;

    // Verify the incoming stream processor function matches the signature
    { codec.template deserializeLastStreamMessage<DiffDriveStateData>(stream_buffer, error_message) } -> std::same_as<std::optional<DiffDriveStateData>>;
    { codec.template deserializeLastStreamMessage<MsgId::Deactivate>(stream_buffer, error_message) } -> std::same_as<bool>;
};

} // namespace bumperbot_firmware

#endif // PROTOCOL_CODEC_CONCEPT_HPP