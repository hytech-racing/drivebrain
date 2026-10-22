#include "CameraSync.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

int main() {
    using namespace comms::camera_sync;
    // Fixed wire-format fixture: preserves both halves of a 64-bit PTP time.
    const std::array<uint8_t, 28> expected{
        0x42, 0x81, 0x01, 0x00, 0x00, 0x14, 0x12, 0x34,
        0x48, 0x59, 0x54, 0x43, 0, 0, 0, 1, 0, 0, 0, 1,
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
    assert(scheduled_action(0x1234, 0x0123456789abcdefULL) == expected);
    std::array<uint8_t, 8> ack{0, 0, 1, 1, 0, 0, 0x12, 0x34};
    assert(action_accepted(ack.data(), ack.size(), 0x1234));
    assert(!action_accepted(ack.data(), 7, 0x1234));
    assert(!action_accepted(ack.data(), ack.size(), 0x1235));
    ack[0] = 0x80; ack[1] = 0x16; // late action / protocol error
    assert(!action_accepted(ack.data(), ack.size(), 0x1234));
    ack[0] = 0; ack[1] = 0; ack[3] = 0x89; // unrelated ACK
    assert(!action_accepted(ack.data(), ack.size(), 0x1234));
    std::cout << "Camera scheduled-action protocol tests passed\n";
}
