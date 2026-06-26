#ifndef DIFF_DRIVE_MESSAGE_TRAITS_HPP
#define DIFF_DRIVE_MESSAGE_TRAITS_HPP

#include <cstdint>
#include <type_traits>

// --- Message internals ---

// forward declaration of MsgId enum type
enum class MsgId : uint8_t;

#pragma pack(push, 1)

struct MessageHeader {
    uint8_t start_byte;
    MsgId   msg_id;
    uint8_t payload_length;
    uint8_t crc;
};

#pragma pack(pop)

constexpr uint8_t kStartByte = 0xAA;
constexpr size_t kCrcOffset = offsetof(MessageHeader, crc);

// simple Longitudinal Redundancy Check (XOR Checksum) calculation
inline uint8_t calculateLRC8(const uint8_t* data, const size_t len) {
    uint8_t lrc{0};
    for (size_t i = 0; i < len; i++) {
        lrc ^= data[i];
    }
    return lrc;
}

// --- Message descriptor type ---

// Primaty template for messages with payloads
template <MsgId Id, typename TPayload>
struct MessageDef {
    static constexpr MsgId id = Id;
    using PayloadType = TPayload;
    static constexpr bool has_payload = true;
    static constexpr size_t payload_size = sizeof(TPayload);
};

// Partial specialization for zero-payload messages
template <MsgId Id>
struct MessageDef<Id, void> {
    static constexpr MsgId id = Id;
    using PayloadType = void;
    static constexpr bool has_payload = false;
    static constexpr size_t payload_size = 0;
};


// --- Message descriptors registry type ---

template <typename... MsgDefs>
struct MessageRegistry {
    static constexpr std::size_t max_payload_size = [] {
        std::size_t max = 0;
        ((max = std::max(max, MsgDefs::payload_size)), ...);
        return max;
    }();

    template <MsgId TargetId>
    static constexpr bool isZeroPayload() {
        return (((MsgDefs::id == TargetId) && !MsgDefs::has_payload) || ...);
    }
    
    template <typename TPayload>
    static constexpr bool isValidPayload() {
        return ((std::is_same<typename MsgDefs::PayloadType, TPayload>::value) || ...);
    }

    template <typename TPayload>
    static consteval MsgId getPayloadMsgId() {
        static_assert(isValidPayload<TPayload>(),
            "Requested Payload Type not present in MessageRegistry");

        MsgId target_id = static_cast<MsgId>(0);

        ((std::is_same_v<TPayload, typename MsgDefs::PayloadType>
            ? (target_id = MsgDefs::id, true)
            : false) || ...);

        return target_id;
    }
};

// A trait and concept to identify a valid MessageRegistry type
template <typename T>
struct is_message_registry {
    static constexpr bool value = false; 
};

template <typename... MsgDefs>
struct is_message_registry<MessageRegistry<MsgDefs...>> {
    static constexpr bool value = true;
};

template<typename T>
concept MessageRegistryConcept = is_message_registry<T>::value;

#endif // DIFF_DRIVE_MESSAGE_TRAITS_HPP