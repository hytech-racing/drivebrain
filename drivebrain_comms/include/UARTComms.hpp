#pragma once

#include <google/protobuf/message.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace comms {

class UARTComms {
public:
    using Message = google::protobuf::Message;
    using ReceiveHandler = std::function<void(std::shared_ptr<const Message>)>;

    struct Config {
        std::string device_name;
        unsigned int baud_rate = 115200;
        std::size_t max_payload_bytes = 4096;
        std::size_t max_outstanding_tx_bytes = 64 * 1024;
        std::string receive_topic; // Empty generates uart/<lowercase message name>/rx.
    };

    // Stores an empty prototype of the receive type; does not open the port or start I/O.
    UARTComms(Config config, const Message& receive_prototype, ReceiveHandler handler);
    ~UARTComms();

    UARTComms(const UARTComms&) = delete;
    UARTComms& operator=(const UARTComms&) = delete;
    UARTComms(UARTComms&&) = delete;
    UARTComms& operator=(UARTComms&&) = delete;

private:
    friend struct UARTCodecTestAccess;

    // Throws invalid_argument for missing required fields, length_error for oversized
    // payloads, or runtime_error if protobuf serialization fails.
    std::vector<uint8_t> _encode_message(const Message& message) const;
    // Returns nullptr for an invalid frame or protobuf payload.
    std::shared_ptr<Message> _decode_message(const std::vector<uint8_t>& encoded) const;

    static uint16_t _crc16(const std::vector<uint8_t>& bytes, std::size_t size);
    // Wire format: COBS(protobuf payload | little-endian CRC-16) | 0x00.
    // Throws invalid_argument for an unsupported limit or length_error for oversized payloads.
    static std::vector<uint8_t> _encode_uart_frame(
        const std::vector<uint8_t>& payload, std::size_t max_payload_bytes);
    // The receive loop removes the trailing zero delimiter before decoding.
    static std::optional<std::vector<uint8_t>> _decode_uart_frame(
        const std::vector<uint8_t>& encoded, std::size_t max_payload_bytes);

    Config _config;
    std::unique_ptr<Message> _receive_prototype;
    ReceiveHandler _handler;
};

} // namespace comms
