#pragma once

#include <google/protobuf/message.h>
#include "hytech_msgs.pb.h"

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace comms {

// FL, FR, RL, RR. RPMs are signed; torque allowances are magnitudes in N*m.
// Physical vehicle limits belong to the controller, not this transport helper.
inline hytech_msgs::MCUCommandData make_drivetrain_uart_command(
    const std::array<float, 4>& desired_rpms,
    const std::array<float, 4>& torque_limits_nm,
    int64_t previous_mcu_receive_millis = 0) {
    for (std::size_t i = 0; i < desired_rpms.size(); ++i) {
        if (!std::isfinite(desired_rpms[i]) || !std::isfinite(torque_limits_nm[i]) || torque_limits_nm[i] < 0) {
            throw std::invalid_argument("UART commands require finite RPMs and nonnegative finite torque allowances");
        }
    }
    const auto assign = [](hytech_msgs::veh_vec_float* out, const std::array<float, 4>& values) {
        out->set_fl(values[0]);
        out->set_fr(values[1]);
        out->set_rl(values[2]);
        out->set_rr(values[3]);
    };
    hytech_msgs::MCUCommandData command;
    assign(command.mutable_desired_rpms(), desired_rpms);
    assign(command.mutable_torque_limit_nm(), torque_limits_nm);
    command.set_prev_mcu_recv_millis(previous_mcu_receive_millis);
    return command;
}

class UARTComms {
public:
    using Message = google::protobuf::Message;
    using ReceiveHandler = std::function<void(std::shared_ptr<const Message>)>;

    using LogHandler = std::function<void(std::shared_ptr<const Message>, const std::string&)>;

    struct Config {
        std::string device_name;
        unsigned int baud_rate = 115200;
        std::size_t max_payload_bytes = 4096;
        std::size_t max_outstanding_tx_bytes = 64 * 1024;
        std::string receive_topic; // Empty generates uart/<lowercase message name>/rx.
        LogHandler logger; // Optional; runs on the UART worker, must not block or call start/stop.
    };

    // Cumulative since construction. Snapshot counters may advance independently.
    struct Stats {
        uint64_t tx_accepted_frames = 0;
        uint64_t tx_completed_frames = 0;
        uint64_t tx_bytes = 0;
        uint64_t send_rejections = 0;
        uint64_t rx_bytes = 0;
        uint64_t rx_frames = 0;
        uint64_t cobs_errors = 0;
        uint64_t crc_errors = 0;
        uint64_t oversize_errors = 0;
        uint64_t protobuf_errors = 0;
        uint64_t callback_errors = 0;
        uint64_t logger_errors = 0;
        uint64_t io_errors = 0;
        std::size_t outstanding_tx_bytes = 0;
        std::size_t tx_high_water_bytes = 0;
    };

    Stats stats() const;

    // Stores an empty prototype of the receive type; does not open the port or start I/O.
    UARTComms(Config config, const Message& receive_prototype, ReceiveHandler handler);
    ~UARTComms();

    UARTComms(const UARTComms&) = delete;
    UARTComms& operator=(const UARTComms&) = delete;
    UARTComms(UARTComms&&) = delete;
    UARTComms& operator=(UARTComms&&) = delete;

    // Call from the owning application thread, outside receive callbacks.
    // Repeated calls are safe. After an I/O failure, call stop() before restarting.
    // start() throws if opening or configuring fails.
    void start();
    void stop(); // Cancels I/O and joins the worker before returning.
    bool is_running() const noexcept;

    // Encodes on the calling thread; encoding/resource errors propagate as exceptions.
    // Thread-safe. false means stopped or TX capacity full; true means accepted,
    // not transmitted. The caller may modify or destroy the message after return.
    bool send(const Message& message);

private:
    enum class DecodeError { None, COBS, CRC, Oversize };

    friend struct UARTCodecTestAccess;
    friend struct UARTBenchAccess; // Hardware-test injector; shares the bounded TX queue.
    friend struct UARTLifecycleTestAccess;

    bool _submit_frame(std::vector<uint8_t> frame);
    void _start_receive();
    void _handle_receive(const boost::system::error_code& error, std::size_t size);
    void _start_write();
    void _handle_write(const boost::system::error_code& error, std::size_t size);
    void _release_tx_bytes(std::size_t size);
    void _halt_io(); // Cancels/closes without joining; runs on the I/O thread or after it exits.

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
        const std::vector<uint8_t>& encoded, std::size_t max_payload_bytes,
        DecodeError* error = nullptr);

    Config _config;
    std::unique_ptr<Message> _receive_prototype;
    ReceiveHandler _handler;

    using WorkGuard = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;
    boost::asio::io_context _io_context;
    boost::asio::serial_port _serial;
    std::optional<WorkGuard> _work_guard; // Keeps run() alive between operations.
    std::thread _worker;
    std::atomic<bool> _running{false};

    std::array<uint8_t, 4096> _receive_buffer{};
    std::vector<uint8_t> _encoded_frame;
    bool _discarding_frame = false;

    // I/O handlers own this queue; its front owns the active write buffer.
    std::deque<std::vector<uint8_t>> _write_queue;
    bool _write_in_progress = false;

    // Includes posted frames, queued frames, and the active write.
    mutable std::mutex _submission_mutex;
    bool _accepting_sends = false;
    std::size_t _outstanding_tx_bytes = 0;
    std::size_t _tx_high_water_bytes = 0;

    struct AtomicStats {
        std::atomic<uint64_t> tx_accepted_frames{0};
        std::atomic<uint64_t> tx_completed_frames{0};
        std::atomic<uint64_t> tx_bytes{0};
        std::atomic<uint64_t> send_rejections{0};
        std::atomic<uint64_t> rx_bytes{0};
        std::atomic<uint64_t> rx_frames{0};
        std::atomic<uint64_t> cobs_errors{0};
        std::atomic<uint64_t> crc_errors{0};
        std::atomic<uint64_t> oversize_errors{0};
        std::atomic<uint64_t> protobuf_errors{0};
        std::atomic<uint64_t> callback_errors{0};
        std::atomic<uint64_t> logger_errors{0};
        std::atomic<uint64_t> io_errors{0};
    };
    mutable AtomicStats _stats;
};

} // namespace comms
