#ifndef DIFF_DRIVE_MESSAGES_HPP
#define DIFF_DRIVE_MESSAGES_HPP

#include <stdint.h>
#include "diff_drive_data.hpp"
#include "diff_drive_message_traits.hpp"

// Unique identifiers for every valid command over the wire.
enum class MsgId : uint8_t {
    DiffDriveConfig,
    ImuConfig,
    DiffDriveCommand,
    DiffDriveState,
    SystemState,
    Deactivate,
    End // meta value designating end of MsgId enum
};

using DiffDriveConfigMsg = MessageDef<MsgId::DiffDriveConfig, DiffDriveConfigData>;
using ImuConfigMsg = MessageDef<MsgId::ImuConfig, ImuConfigData>;
using DiffDriveCommandMsg = MessageDef<MsgId::DiffDriveCommand, DiffDriveCommandData>;
using DiffDriveStateMsg = MessageDef<MsgId::DiffDriveState, DiffDriveStateData>;
using SystemStateMsg = MessageDef<MsgId::SystemState, SystemStateData>;
using DeactivateMsg = MessageDef<MsgId::Deactivate, void>; // zero-payload

// Single source of truth containing all valid protocol definitions.
using DiffDriveMessageRegistry = MessageRegistry<
    DiffDriveConfigMsg,
    ImuConfigMsg,
    DiffDriveCommandMsg,
    DiffDriveStateMsg,
    SystemStateMsg,
    DeactivateMsg
>;

#endif // DIFF_DRIVE_MESSAGES_HPP