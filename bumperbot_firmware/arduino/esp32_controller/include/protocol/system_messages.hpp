#ifndef SYSTEM_MESSAGES_HPP
#define SYSTEM_MESSAGES_HPP

#include <stdint.h>
#include "message_traits.hpp"
#include "system_data.hpp"

// Unique identifiers for every valid command over the wire.
enum class MsgId : uint8_t {
    DiffDriveConfig,
    ImuConfig,
    DiffDriveCommand,
    SystemState,
    Deactivate,
    End  // meta value designating end of MsgId enum
};

using DiffDriveConfigMsg = MessageDef<MsgId::DiffDriveConfig, DiffDriveConfigData>;
using ImuConfigMsg = MessageDef<MsgId::ImuConfig, ImuConfigData>;
using DiffDriveCommandMsg = MessageDef<MsgId::DiffDriveCommand, DiffDriveCommandData>;
using SystemStateMsg = MessageDef<MsgId::SystemState, SystemStateData>;
using DeactivateMsg = MessageDef<MsgId::Deactivate, void>;  // zero-payload

// Single source of truth containing all valid protocol definitions.
using DiffDriveMessageRegistry = MessageRegistry<DiffDriveConfigMsg,
                                                 ImuConfigMsg,
                                                 DiffDriveCommandMsg,
                                                 SystemStateMsg,
                                                 DeactivateMsg>;

#endif  // SYSTEM_MESSAGES_HPP