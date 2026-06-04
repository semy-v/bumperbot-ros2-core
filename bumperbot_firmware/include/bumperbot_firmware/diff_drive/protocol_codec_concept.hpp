#ifndef PROTOCOL_CODEC_CONCEPT_HPP
#define PROTOCOL_CODEC_CONCEPT_HPP

#include <vector>
#include <string>
#include <concepts>
#include <cstdint>
#include "diff_drive_messages.hpp"

namespace bumperbot_firmware {

template <typename T>
concept ProtocolCodec = requires(T codec, 
                                 const ConfigData& pid_data,
                                 const VelocityData& vel_data,
                                 std::vector<uint8_t>& stream_buffer, 
                                 VelocityData& out_vel_data, 
                                 std::string& error_message) 
{
    // Verify that a generic template serialize function exists for all required types
    { codec.template serializeMessage<ConfigData>(pid_data) } -> std::same_as<const std::vector<uint8_t>&>;
    { codec.template serializeMessage<VelocityData>(vel_data) } -> std::same_as<const std::vector<uint8_t>&>;
    { codec.template serializeMessage<MsgId::Deactivate>() } -> std::same_as<const std::vector<uint8_t>&>;

    // Verify the incoming stream processor function matches the signature
    { codec.template deserializeLastStreamMessage<VelocityData>(stream_buffer, out_vel_data, error_message) } -> std::same_as<bool>;
    { codec.template deserializeLastStreamMessage<MsgId::Deactivate>(stream_buffer, error_message) } -> std::same_as<bool>;
};

} // namespace bumperbot_firmware

#endif // PROTOCOL_CODEC_CONCEPT_HPP