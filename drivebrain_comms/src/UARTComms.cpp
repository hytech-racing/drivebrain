#include "UARTComms.hpp"

#include <boost/crc.hpp>

#include <limits>
#include <stdexcept>
#include <utility>

namespace comms {

UARTComms::UARTComms(Config config, const Message& receive_prototype, ReceiveHandler handler)
    : _config(std::move(config)),
      _receive_prototype(receive_prototype.New()),
      _handler(std::move(handler)) {
    if (!_handler) {
        throw std::invalid_argument("UARTComms requires a receive handler");
    }
    if (_config.max_payload_bytes > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("UART payload limit exceeds protobuf's array API limit");
    }
    if (_config.receive_topic.empty()) {
        std::string name = _receive_prototype->GetDescriptor()->name();
        for (char& character : name) {
            if (character >= 'A' && character <= 'Z') {
                character = static_cast<char>(character - 'A' + 'a');
            }
        }
        _config.receive_topic = "uart/" + name + "/rx";
    }
}

UARTComms::~UARTComms() = default;

std::vector<uint8_t> UARTComms::_encode_message(const Message& message) const {
    if (!message.IsInitialized()) {
        throw std::invalid_argument("Cannot encode UART message " + message.GetTypeName() +
                                    ": missing required fields: " + message.InitializationErrorString());
    }
    const std::size_t size = message.ByteSizeLong();
    if (size > _config.max_payload_bytes) {
        throw std::length_error("Cannot encode UART message " + message.GetTypeName() +
                                ": payload is " + std::to_string(size) + " bytes; maximum is " +
                                std::to_string(_config.max_payload_bytes));
    }

    std::vector<uint8_t> payload(size);
    if (!message.SerializeToArray(payload.data(), static_cast<int>(size))) {
        throw std::runtime_error("Cannot encode UART message " + message.GetTypeName() +
                                 ": protobuf serialization failed");
    }
    return _encode_uart_frame(payload, _config.max_payload_bytes);
}

std::shared_ptr<UARTComms::Message> UARTComms::_decode_message(
    const std::vector<uint8_t>& encoded) const {
    auto payload = _decode_uart_frame(encoded, _config.max_payload_bytes);
    if (!payload) {
        return nullptr;
    }

    auto message = std::shared_ptr<Message>(_receive_prototype->New());
    if (!message->ParseFromArray(payload->data(), static_cast<int>(payload->size()))) {
        return nullptr;
    }
    return message;
}

uint16_t UARTComms::_crc16(const std::vector<uint8_t>& bytes, std::size_t size) {
    boost::crc_optimal<16, 0x1021, 0xFFFF, 0, false, false> crc;
    crc.process_bytes(bytes.data(), size);
    return crc.checksum();
}

std::vector<uint8_t> UARTComms::_encode_uart_frame(const std::vector<uint8_t>& payload, std::size_t max_payload_bytes) {
    // Protobuf's array APIs take an int byte count. Also bounds size arithmetic.
    if (max_payload_bytes > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("Cannot encode UART frame: payload limit exceeds protobuf's array API limit");
    }
    if (payload.size() > max_payload_bytes) {
        throw std::length_error("Cannot encode UART frame: payload is " +
                                std::to_string(payload.size()) + " bytes; maximum is " +
                                std::to_string(max_payload_bytes));
    }

    auto bytes = payload;
    const uint16_t checksum = _crc16(bytes, bytes.size());
    bytes.push_back(static_cast<uint8_t>(checksum & 0xFF));
    bytes.push_back(static_cast<uint8_t>(checksum >> 8));

    std::vector<uint8_t> encoded{0}; // Placeholder for the first COBS code.
    encoded.reserve(bytes.size() + bytes.size() / 254 + 2);
    std::size_t code_index = 0;
    uint8_t code = 1;
    for (uint8_t byte : bytes) {
        if (byte == 0) {
            encoded[code_index] = code;
            code_index = encoded.size();
            encoded.push_back(0);
            code = 1;
        } else {
            encoded.push_back(byte);
            if (++code == 0xFF) {
                encoded[code_index] = code;
                code_index = encoded.size();
                encoded.push_back(0);
                code = 1;
            }
        }
    }
    encoded[code_index] = code;
    encoded.push_back(0); // Frame delimiter; all preceding bytes are nonzero.
    return encoded;
}

std::optional<std::vector<uint8_t>> UARTComms::_decode_uart_frame(const std::vector<uint8_t>& encoded, std::size_t max_payload_bytes) {
    if (max_payload_bytes > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        return std::nullopt;
    }
    const std::size_t decoded_limit = max_payload_bytes + 2; // Payload plus CRC.
    if (encoded.empty() || encoded.size() > decoded_limit + decoded_limit / 254 + 1) {
        return std::nullopt;
    }

    std::vector<uint8_t> decoded;
    decoded.reserve(encoded.size() < decoded_limit ? encoded.size() : decoded_limit);
    for (std::size_t position = 0; position < encoded.size();) {
        const uint8_t code = encoded[position++];
        if (code == 0 || code - 1u > encoded.size() - position) {
            return std::nullopt;
        }
        for (unsigned int index = 1; index < code; ++index) {
            const uint8_t byte = encoded[position++];
            if (byte == 0 || decoded.size() == decoded_limit) {
                return std::nullopt;
            }
            decoded.push_back(byte);
        }
        if (code != 0xFF && position < encoded.size()) {
            if (decoded.size() == decoded_limit) {
                return std::nullopt;
            }
            decoded.push_back(0);
        }
    }

    if (decoded.size() < 2) {
        return std::nullopt;
    }
    const std::size_t payload_size = decoded.size() - 2;
    const uint16_t checksum = static_cast<uint16_t>(decoded[payload_size]) |
                              (static_cast<uint16_t>(decoded[payload_size + 1]) << 8);
    if (_crc16(decoded, payload_size) != checksum) {
        return std::nullopt;
    }
    decoded.resize(payload_size);
    return decoded;
}

} // namespace comms
