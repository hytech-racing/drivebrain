#pragma once

#include <google/protobuf/message.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace comms {

template <typename ReceiveMessage, typename SendMessage>
class UARTComms {
    static_assert(std::is_base_of_v<google::protobuf::Message, ReceiveMessage>,
                  "ReceiveMessage must be a protobuf message");
    static_assert(std::is_base_of_v<google::protobuf::Message, SendMessage>,
                  "SendMessage must be a protobuf message");

public:
    using ReceiveHandler = std::function<void(std::shared_ptr<ReceiveMessage>)>;

    struct Config {
        std::string device_name;
        unsigned int baud_rate = 115200;
        std::size_t max_payload_bytes = 4096;
        std::size_t max_outstanding_tx_bytes = 64 * 1024;
    };

    // Stores configuration only; construction does not open the port or start I/O.
    UARTComms(Config config, ReceiveHandler handler)
        : _config(std::move(config)), _handler(std::move(handler)) {
        if (!_handler) {
            throw std::invalid_argument("UARTComms requires a receive handler");
        }
    }

    ~UARTComms() = default;

    // The driver will own a worker and asynchronous handlers referring to this object.
    UARTComms(const UARTComms&) = delete;
    UARTComms& operator=(const UARTComms&) = delete;
    UARTComms(UARTComms&&) = delete;
    UARTComms& operator=(UARTComms&&) = delete;

private:
    Config _config;
    ReceiveHandler _handler;
};

} // namespace comms
