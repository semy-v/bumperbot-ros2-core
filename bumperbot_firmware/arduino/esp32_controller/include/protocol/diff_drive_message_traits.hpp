#ifndef DIFF_DRIVE_MESSAGE_TRAITS_HPP
#define DIFF_DRIVE_MESSAGE_TRAITS_HPP

#include <stdint.h>

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


// --- Internal helpers to bypass the Arduino limitations ---

namespace detail {

// STL type checker substitution 
template<typename T, typename U>
struct is_same {
    static const bool value = false;
};

template<typename T>
struct is_same<T, T> {
    static const bool value = true;
};

// Compile-Time Struct Resolver (Bypasses Arduino constexpr function limitation) ---
template <typename TPayload, typename... MsgDefs>
struct Resolver;

template <typename TPayload, typename MsgDef1, typename... Rest>
struct Resolver<TPayload, MsgDef1, Rest...> {
    static constexpr MsgId value = is_same<TPayload, typename MsgDef1::PayloadType>::value
        ? MsgDef1::id
        : Resolver<TPayload, Rest...>::value;
};

template <typename TPayload, typename Last>
struct Resolver<TPayload, Last> {
    static constexpr MsgId value = is_same<TPayload, typename Last::PayloadType>::value
        ? Last::id
        : static_cast<MsgId>(0); // fallback guarded by the outer static_assert
};

// Compile-Time Max Size Calculator (Bypasses Arduino constexpr function limitation)
template <size_t... Sizes>
struct MaxSize;

template <size_t First, size_t... Rest>
struct MaxSize<First, Rest...> {
    static constexpr size_t rest_max = MaxSize<Rest...>::value;
    static constexpr size_t value = (First > rest_max) ? First : rest_max;
};

template <size_t Last>
struct MaxSize<Last> {
    static constexpr size_t value = Last;
};

// Edge Case: Empty parameter pack
template <>
struct MaxSize<> {
    static constexpr size_t value = 0;
};

} // namespace detail


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
    static constexpr size_t max_payload_size = detail::MaxSize<MsgDefs::payload_size...>::value;

    template <MsgId TargetId>
    static constexpr bool isZeroPayload() {
        return (((MsgDefs::id == TargetId) && !MsgDefs::has_payload) || ...);
    }
    
    template <typename TPayload>
    static constexpr bool isValidPayload() {
        return ((detail::is_same<typename MsgDefs::PayloadType, TPayload>::value) || ...);
    }

    template <typename TPayload>
    static constexpr MsgId getPayloadMsgId() {
        static_assert(isValidPayload<TPayload>(),
            "Requested Payload Type not present in MessageRegistry");

        return detail::Resolver<TPayload, MsgDefs...>::value;
    }
};

// A trait to identify a valid MessageRegistry type
template <typename T>
struct is_message_registry {
    static constexpr bool value = false; 
};

template <typename... MsgDefs>
struct is_message_registry<MessageRegistry<MsgDefs...>> {
    static constexpr bool value = true;
};

#endif // DIFF_DRIVE_MESSAGE_TRAITS_HPP