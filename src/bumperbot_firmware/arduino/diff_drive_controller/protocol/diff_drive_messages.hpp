#ifndef DIFF_DRIVE_MESSAGES_HPP
#define DIFF_DRIVE_MESSAGES_HPP

#include <stdint.h>
#include "diff_drive_data.hpp"
#include "diff_drive_message_traits.hpp"

// Unique identifiers for every valid command over the wire.
enum class MsgId : uint8_t {
    Config,
    Velocity,
    Deactivate,
    End // meta value designating end of MsgId enum
};

using ConfigMsg     = MessageDef<MsgId::Config, ConfigData>;
using VelocityMsg   = MessageDef<MsgId::Velocity, VelocityData>;
using DeactivateMsg = MessageDef<MsgId::Deactivate, void>; // zero-payload

// Single source of truth containing all valid protocol definitions.
using DiffDriveMessageRegistry = MessageRegistry<
    ConfigMsg,
    VelocityMsg,
    DeactivateMsg
>;

#endif // DIFF_DRIVE_MESSAGES_HPP