#ifndef MESSAGE_TRAITS_HPP
#define MESSAGE_TRAITS_HPP

#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>

// #### Message internals ####

// forward declaration of MsgId enum type
enum class MsgId : uint8_t;

#pragma pack(push, 1)

struct MessageHeader {
    std::byte start_byte;
    MsgId msg_id;
    uint8_t payload_length;
    uint16_t crc;

    bool operator<=>(const MessageHeader&) const = default;
};

#pragma pack(pop)

static_assert(sizeof(MessageHeader) == 5);

constexpr std::byte kStartByte{0xAA};
constexpr size_t kCrcOffset = offsetof(MessageHeader, crc);

// #### CRC helpers ####

// Generate the 256-byte lookup table at compile time
consteval std::array<uint16_t, 256> generateCrcTable() {
    std::array<uint16_t, 256> table{};
    for (int i = 0; i < 256; ++i) {
        uint16_t crc = static_cast<uint16_t>(i) << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
        table[i] = crc;
    }
    return table;
}

inline constexpr auto kCrcTable = generateCrcTable();

constexpr uint16_t crcByte(uint16_t crc, std::byte byte) {
    uint8_t index = static_cast<uint8_t>((crc >> 8) ^ static_cast<uint8_t>(byte));
    return (crc << 8) ^ kCrcTable[index];
}

constexpr uint16_t calculateHeaderCRC(std::byte start, MsgId id, uint8_t length) {
    uint16_t crc = 0xFFFF;

    crc = crcByte(crc, start);
    crc = crcByte(crc, static_cast<std::byte>(id));
    crc = crcByte(crc, static_cast<std::byte>(length));

    return crc;
}

constexpr uint16_t calculateCRC16(std::span<const std::byte> data, uint16_t initial_crc = 0xFFFF) {
    uint16_t crc = initial_crc;

    for (const auto b : data) {
        crc = crcByte(crc, b);
    }

    return crc;
}

// #### Message descriptor type ####

// Primary template for messages with payloads
template <MsgId Id, typename TPayload>
struct MessageDef {
    static_assert(std::is_trivially_copyable_v<TPayload>,
                  "Message protocol supports only trivially copyable payload types");
    static_assert(std::is_standard_layout_v<TPayload>,
                  "Message protocol supports only standard memory layout payload types");
    static_assert(sizeof(TPayload) <=
                      std::numeric_limits<decltype(MessageHeader::payload_length)>::max(),
                  "Message payload size limit exceeded");

    static constexpr MsgId id = Id;
    using PayloadType = TPayload;
    static constexpr bool has_payload = true;
    static constexpr decltype(MessageHeader::payload_length) payload_size = sizeof(TPayload);
    static constexpr MessageHeader header = {
        .start_byte = kStartByte,
        .msg_id = id,
        .payload_length = payload_size,
        .crc = calculateHeaderCRC(kStartByte, id, payload_size)};
};

// Partial specialization for zero-payload messages
template <MsgId Id>
struct MessageDef<Id, void> {
    static constexpr MsgId id = Id;
    using PayloadType = void;
    static constexpr bool has_payload = false;
    static constexpr decltype(MessageHeader::payload_length) payload_size = 0;
    static constexpr MessageHeader header = {.start_byte = kStartByte,
                                             .msg_id = id,
                                             .payload_length = 0,
                                             .crc = calculateHeaderCRC(kStartByte, id, 0)};
};

// #### Message registry type ####

template <typename... MsgDefs>
struct MessageRegistry {
    static constexpr std::size_t max_payload_size = [] {
        decltype(MessageHeader::payload_length) max_size = 0;
        ((max_size = std::max(max_size, MsgDefs::payload_size)), ...);
        return max_size;
    }();

    template <MsgId TargetId>
    static consteval bool isZeroPayload() {
        return (((MsgDefs::id == TargetId) && !MsgDefs::has_payload) || ...);
    }

    template <typename TPayload>
    static consteval bool isValidPayload() {
        return ((std::is_same<typename MsgDefs::PayloadType, TPayload>::value) || ...);
    }

    template <MsgId TargetId>
    static consteval bool isValidMsgId() {
        return ((MsgDefs::id == TargetId) || ...);
    }

    static constexpr bool isValidPayloadSize(MsgId target_id, const uint8_t payload_size) {
        return (((MsgDefs::id == target_id) && (MsgDefs::payload_size == payload_size)) || ...);
    }

    template <typename TPayload>
    static consteval MsgId getPayloadMsgId() {
        static_assert(isValidPayload<TPayload>(),
                      "Requested Payload Type not present in MessageRegistry");

        MsgId target_id = static_cast<MsgId>(0);

        ((std::is_same_v<TPayload, typename MsgDefs::PayloadType> ? (target_id = MsgDefs::id, true)
                                                                  : false) ||
         ...);

        return target_id;
    }

    template <MsgId TargetId>
    static consteval const MessageHeader& getMessageHeader() {
        const MessageHeader* result = nullptr;
        ((MsgDefs::id == TargetId ? (result = &MsgDefs::header, true) : false) || ...);

        return *result;
    }

    static constexpr const MessageHeader* getMessageHeader(const MsgId target_id) {
        const MessageHeader* result = nullptr;
        ((MsgDefs::id == target_id ? (result = &MsgDefs::header, true) : false) || ...);

        return result;
    }

    template <MsgId TargetId>
    static constexpr std::span<const uint8_t> getHeaderBytes() {
        static_assert(std::is_same_v<uint8_t, unsigned char>);

        constexpr auto& header = getMessageHeader<TargetId>();
        return {reinterpret_cast<const uint8_t*>(&header), sizeof(header)};
    }
};

// #### Message registry concept ####

template <typename T>
struct is_message_registry : std::false_type {};

template <typename... MsgDefs>
struct is_message_registry<MessageRegistry<MsgDefs...>> : std::true_type {};

template <typename T>
concept MessageRegistryConcept = is_message_registry<std::remove_cvref_t<T>>::value;

#endif  // MESSAGE_TRAITS_HPP