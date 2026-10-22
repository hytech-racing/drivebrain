#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace comms::camera_sync {
constexpr uint32_t device_key = 0x48595443;
constexpr uint32_t group_key = 1;
constexpr uint32_t group_mask = 1;
constexpr uint64_t ns_per_second = 1000000000;
constexpr int64_t max_offset_ns = 100000; // 100 us per camera relative to master
constexpr uint64_t lead_ns = 200000000;

// GigE Vision ACTION_CMD: network byte order, scheduled-time and ACK flags.
inline std::array<uint8_t, 28> scheduled_action(uint16_t request, uint64_t time) {
    std::array<uint8_t, 28> packet{0x42, 0x81, 0x01, 0x00, 0x00, 0x14,
                                   uint8_t(request >> 8), uint8_t(request)};
    const uint32_t words[] = {device_key, group_key, group_mask,
                              uint32_t(time >> 32), uint32_t(time)};
    for (std::size_t i = 0; i < 5; ++i)
        for (std::size_t j = 0; j < 4; ++j)
            packet[8 + 4 * i + j] = uint8_t(words[i] >> (24 - 8 * j));
    return packet;
}

inline bool action_accepted(const uint8_t* ack, std::size_t size, uint16_t request) {
    return size == 8 && ack[0] == 0 && ack[1] == 0 &&
           ack[2] == 1 && ack[3] == 1 && ack[4] == 0 && ack[5] == 0 &&
           ack[6] == uint8_t(request >> 8) && ack[7] == uint8_t(request);
}
} // namespace comms::camera_sync
