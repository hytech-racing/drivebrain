#include "UARTComms.hpp"

#include <boost/crc.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/write.hpp>
#include <spdlog/spdlog.h>

#include <limits>
#include <stdexcept>
#include <utility>

namespace comms {

UARTComms::UARTComms(Config config, const Message& receive_prototype, ReceiveHandler handler)
    : _config(std::move(config)),
      _receive_prototype(receive_prototype.New()),
      _handler(std::move(handler)),
      _serial(_io_context) {
    if (!_handler) {
        throw std::invalid_argument("UARTComms requires a receive handler");
    }
    if (_config.max_payload_bytes > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("UART payload limit exceeds protobuf's array API limit");
    }
    if (_config.receive_topic.empty()) {
        std::string name(_receive_prototype->GetDescriptor()->name());
        for (char& character : name) {
            if (character >= 'A' && character <= 'Z') {
                character = static_cast<char>(character - 'A' + 'a');
            }
        }
        _config.receive_topic = "uart/" + name + "/rx";
    }
}

UARTComms::~UARTComms() {
    stop();
}

void UARTComms::start() {
    if (_running.load()) {
        return;
    }
    if (_worker.joinable()) {
        throw std::logic_error("Call UARTComms::stop() before restarting after an I/O failure");
    }
    if (_config.device_name.empty() || _config.baud_rate == 0) {
        throw std::invalid_argument("UART start requires a device name and nonzero baud rate");
    }

    _io_context.restart();
    _encoded_frame.clear();
    _discarding_frame = false;
    try {
        _serial.open(_config.device_name);
        using Serial = boost::asio::serial_port_base;
        _serial.set_option(Serial::baud_rate(_config.baud_rate));
        _serial.set_option(Serial::character_size(8));
        _serial.set_option(Serial::parity(Serial::parity::none));
        _serial.set_option(Serial::stop_bits(Serial::stop_bits::one));
        _serial.set_option(Serial::flow_control(Serial::flow_control::none));

        _work_guard.emplace(boost::asio::make_work_guard(_io_context));
        _running.store(true);
        boost::asio::post(_io_context, [this]() { _start_receive(); });
        // A failure on the worker must not race enabling send admission.
        std::lock_guard lock(_submission_mutex);
        _worker = std::thread([this]() { _io_context.run(); });
        _accepting_sends = true;
    } catch (...) {
        // No worker was launched if opening, configuring, or thread creation failed.
        _running.store(false);
        _work_guard.reset();
        boost::system::error_code ignored;
        _serial.close(ignored);
        // Drain the initial receive post if worker creation failed.
        _io_context.run();
        throw;
    }
}

void UARTComms::stop() {
    {
        std::lock_guard lock(_submission_mutex);
        _accepting_sends = false;
        _running.store(false);
        _transmit_idle.notify_all();
    }
    if (!_worker.joinable()) {
        return;
    }

    // Closing on the I/O thread serializes shutdown with read/write handlers.
    boost::asio::post(_io_context, [this]() { _halt_io(); });
    // run() drains cancellation handlers before returning; do not call io_context::stop().
    _worker.join();
    // A read failure may have ended run() before it could execute the shutdown post.
    // Drain that post now, while stopped, so it cannot affect the next start().
    _io_context.restart();
    _io_context.run();
    _encoded_frame.clear();
    _discarding_frame = false;
}

bool UARTComms::is_running() const noexcept {
    return _running.load();
}

bool UARTComms::send(const Message& message) {
    auto frame = _encode_message(message);
    const std::size_t size = frame.size();

    std::lock_guard lock(_submission_mutex);
    if (!_accepting_sends || size > _config.max_outstanding_tx_bytes - _outstanding_tx_bytes) {
        return false;
    }
    _outstanding_tx_bytes += size;
    try {
        boost::asio::post(_io_context, [this, frame = std::move(frame)]() mutable {
            const std::size_t size = frame.size();
            if (!_running.load()) {
                _release_tx_bytes(size);
                return;
            }

            const bool idle = _write_queue.empty();
            try {
                _write_queue.push_back(std::move(frame));
            } catch (const std::exception& error) {
                _release_tx_bytes(size);
                spdlog::error("UART {} could not queue a write: {}", _config.device_name, error.what());
                _halt_io();
                return;
            }
            if (idle) {
                _start_write();
            }
        });
    } catch (...) {
        _outstanding_tx_bytes -= size; // Posting failed; the frame was never submitted.
        throw;
    }
    return true;
}

bool UARTComms::wait_for_transmit_idle(std::chrono::milliseconds timeout) {
    std::unique_lock lock(_submission_mutex);
    return _transmit_idle.wait_for(lock, timeout, [this] {
        return !_running.load() || _outstanding_tx_bytes == 0;
    }) && _running.load();
}

void UARTComms::_release_tx_bytes(std::size_t size) {
    std::lock_guard lock(_submission_mutex);
    _outstanding_tx_bytes -= size;
    _transmit_idle.notify_all();
}

void UARTComms::_halt_io() {
    {
        std::lock_guard lock(_submission_mutex);
        _accepting_sends = false;
        _running.store(false);
        _transmit_idle.notify_all();
    }
    boost::system::error_code ignored;
    _serial.cancel(ignored);
    _serial.close(ignored);
    _work_guard.reset();
    _encoded_frame.clear();
    _discarding_frame = false;

    // Keep the active buffer until its completion handler runs after cancellation.
    const std::size_t keep = _write_in_progress ? 1 : 0;
    while (_write_queue.size() > keep) {
        _release_tx_bytes(_write_queue.back().size());
        _write_queue.pop_back();
    }
}

void UARTComms::_start_write() {
    if (!_running.load() || _write_in_progress || _write_queue.empty()) {
        return;
    }
    _write_in_progress = true;
    try {
        boost::asio::async_write(_serial, boost::asio::buffer(_write_queue.front()),
            [this](const boost::system::error_code& error, std::size_t size) {
                _handle_write(error, size);
            });
    } catch (const std::exception& error) {
        _write_in_progress = false;
        spdlog::error("UART {} could not start writing: {}", _config.device_name, error.what());
        _halt_io();
    }
}

void UARTComms::_handle_write(const boost::system::error_code& error, std::size_t size) {
    const std::size_t expected = _write_queue.front().size();
    _write_in_progress = false;
    if (_running.load() && (error || size != expected)) {
        // Publish failure before releasing bytes, so an idle waiter cannot
        // mistake a failed final write for successful completion.
        {
            std::lock_guard lock(_submission_mutex);
            _accepting_sends = false;
            _running.store(false);
        }
        spdlog::error("UART {} write failed after {} of {} bytes: {}", _config.device_name,
                      size, expected, error ? error.message() : "incomplete write");
    }
    _release_tx_bytes(expected);
    _write_queue.pop_front();

    if (!_running.load()) {
        _halt_io();
        return;
    }
    _start_write();
}

void UARTComms::_start_receive() {
    if (!_running.load()) {
        return;
    }
    try {
        _serial.async_read_some(boost::asio::buffer(_receive_buffer),
            [this](const boost::system::error_code& error, std::size_t size) {
                _handle_receive(error, size);
            });
    } catch (const std::exception& error) {
        spdlog::error("UART {} could not start reading: {}", _config.device_name, error.what());
        _halt_io();
    }
}

void UARTComms::_handle_receive(const boost::system::error_code& error, std::size_t size) {
    if (!_running.load()) {
        return; // Includes the cancellation completion during normal shutdown.
    }
    if (error) {
        spdlog::error("UART {} read failed: {}", _config.device_name, error.message());
        _halt_io();
        return;
    }

    const std::size_t decoded_limit = _config.max_payload_bytes + 2; // Include CRC.
    const std::size_t encoded_limit = decoded_limit + decoded_limit / 254 + 1;
    for (std::size_t index = 0; index < size && _running.load(); ++index) {
        const uint8_t byte = _receive_buffer[index];
        if (byte != 0) {
            if (!_discarding_frame) {
                if (_encoded_frame.size() == encoded_limit) {
                    _encoded_frame.clear();
                    _discarding_frame = true;
                } else {
                    _encoded_frame.push_back(byte);
                }
            }
            continue;
        }

        // A delimiter completes one frame and resets collection for the next.
        std::shared_ptr<Message> message;
        if (!_discarding_frame && !_encoded_frame.empty()) {
            message = _decode_message(_encoded_frame);
        }
        _encoded_frame.clear();
        _discarding_frame = false;
        if (!message) {
            continue;
        }

        // Logging failures must not prevent application delivery.
        try {
            if (_config.logger) _config.logger(message, _config.receive_topic);
        } catch (const std::exception& exception) {
            spdlog::error("UART RX logging failed: {}", exception.what());
        } catch (...) {
            spdlog::error("UART RX logging failed with an unknown exception");
        }
        if (!_running.load()) {
            break;
        }
        try {
            _handler(message);
        } catch (const std::exception& exception) {
            spdlog::error("UART receive handler failed: {}", exception.what());
        } catch (...) {
            spdlog::error("UART receive handler failed with an unknown exception");
        }
    }
    _start_receive();
}

std::vector<uint8_t> UARTComms::_encode_message(const Message& message) const {
    if (!message.IsInitialized()) {
        throw std::invalid_argument("Cannot encode UART message " + std::string(message.GetTypeName()) +
                                    ": missing required fields: " + message.InitializationErrorString());
    }
    const std::size_t size = message.ByteSizeLong();
    if (size > _config.max_payload_bytes) {
        throw std::length_error("Cannot encode UART message " + std::string(message.GetTypeName()) +
                                ": payload is " + std::to_string(size) + " bytes; maximum is " +
                                std::to_string(_config.max_payload_bytes));
    }

    std::vector<uint8_t> payload(size);
    if (!message.SerializeToArray(payload.data(), static_cast<int>(size))) {
        throw std::runtime_error("Cannot encode UART message " + std::string(message.GetTypeName()) +
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
