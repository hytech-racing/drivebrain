#include "UARTComms.hpp"

#include <iostream>
#include <stdexcept>

namespace comms {
struct UARTCodecTestAccess {
    static auto encode(const UARTComms& uart, const UARTComms::Message& message) {
        return uart._encode_message(message);
    }
    static auto decode(const UARTComms& uart, const std::vector<uint8_t>& frame) {
        return uart._decode_message(frame);
    }
    static auto crc(const std::vector<uint8_t>& bytes) {
        return UARTComms::_crc16(bytes, bytes.size());
    }
};
}

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() try {
    using Codec = comms::UARTCodecTestAccess;
    check(Codec::crc({'1', '2', '3', '4', '5', '6', '7', '8', '9'}) == 0x29b1,
          "CRC check failed");

    auto command = comms::make_drivetrain_uart_command({100, 200, -300, 400}, {1, 2, 3, 4}, 123);
    check(command.desired_rpms().rl() == -300 && command.torque_limit_nm().rr() == 4 &&
          command.prev_mcu_recv_millis() == 123, "command values changed");

    // Constructing the driver does not open a serial port; all checks run in memory.
    comms::UARTComms uart({}, command, [](auto) {});
    for (const auto& message : {command, comms::make_drivetrain_uart_command({}, {})}) {
        auto frame = Codec::encode(uart, message);
        check(frame.back() == 0, "missing frame delimiter");
        frame.pop_back();
        auto decoded = Codec::decode(uart, frame);
        check(decoded && decoded->SerializeAsString() == message.SerializeAsString(),
              "protobuf/COBS round trip failed");
        frame.back() ^= 1;
        check(!Codec::decode(uart, frame), "corrupted checksum accepted");
    }
    check(!Codec::decode(uart, {4, 1}), "truncated COBS frame accepted");
    check(!Codec::decode(uart, {0}), "invalid COBS code accepted");

    bool rejected = false;
    try {
        comms::make_drivetrain_uart_command({}, {-1, 0, 0, 0});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    check(rejected, "negative torque allowance accepted");
    std::cout << "UART unit tests passed\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
