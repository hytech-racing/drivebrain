#include "UARTComms.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif

namespace comms {
struct UARTCodecTestAccess {
    static auto encode(const google::protobuf::Message& message) {
        UARTComms driver({}, message, [](auto) {});
        return driver._encode_message(message);
    }
    static auto encode_payload(const std::vector<uint8_t>& payload) {
        return UARTComms::_encode_uart_frame(payload, 4096);
    }
    static auto crc(const std::vector<uint8_t>& bytes) {
        return UARTComms::_crc16(bytes, bytes.size());
    }
};
}

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template<class F> void eventually(F condition) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!condition()) {
        require(std::chrono::steady_clock::now() < deadline, "asynchronous operation timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

struct Pty {
    int master = -1;
    int slave = -1;
    char path[256]{};
    Pty() {
        require(openpty(&master, &slave, path, nullptr, nullptr) == 0, "openpty failed");
        termios options{};
        require(tcgetattr(slave, &options) == 0, "tcgetattr failed");
        cfmakeraw(&options);
        require(tcsetattr(slave, TCSANOW, &options) == 0, "tcsetattr failed");
    }
    ~Pty() { if (master >= 0) close(master); if (slave >= 0) close(slave); }
    void write_frame(const std::vector<uint8_t>& bytes) {
        std::size_t offset = 0;
        while (offset < bytes.size()) {
            const auto n = write(master, bytes.data() + offset, bytes.size() - offset);
            require(n > 0, "PTY write failed");
            offset += static_cast<std::size_t>(n);
        }
    }
    std::vector<uint8_t> read_bytes(std::size_t size) {
        std::vector<uint8_t> bytes(size);
        std::size_t offset = 0;
        while (offset < size) {
            pollfd fd{master, POLLIN, 0};
            require(poll(&fd, 1, 2000) > 0, "PTY read timed out");
            const auto n = read(master, bytes.data() + offset, size - offset);
            require(n > 0, "PTY read failed");
            offset += static_cast<std::size_t>(n);
        }
        return bytes;
    }
};

int main(int argc, char** argv) try {
    using Codec = comms::UARTCodecTestAccess;
    auto command = comms::make_drivetrain_uart_command({100, 200, -300, 400}, {1, 2, 3, 4}, 123456);
    require(command.desired_rpms().rl() == -300 && command.torque_limit_nm().rr() == 4,
            "wheel values/order changed");
    require(command.prev_mcu_recv_millis() == 123456, "legacy timestamp changed");
    auto zero = comms::make_drivetrain_uart_command({}, {});
    require(zero.has_desired_rpms() && zero.has_torque_limit_nm(), "zero vectors lost message presence");
    for (int test = 0; test < 3; ++test) {
        bool rejected = false;
        try {
            comms::make_drivetrain_uart_command(
                {test == 0 ? std::numeric_limits<float>::quiet_NaN() : 0, 0, 0, 0},
                {test == 1 ? -1.0f : (test == 2 ? std::numeric_limits<float>::infinity() : 0), 0, 0, 0});
        } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "invalid command accepted");
    }
    require(Codec::crc({'1','2','3','4','5','6','7','8','9'}) == 0x29b1, "CRC check vector mismatch");
    const auto frame = Codec::encode(command);
    if (argc == 3 && std::string(argv[1]) == "--fixtures") {
        std::filesystem::create_directories(argv[2]);
        const auto save = [&](const char* name, const std::vector<uint8_t>& bytes) {
            std::ofstream file(std::filesystem::path(argv[2]) / name, std::ios::binary);
            file.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
            require(static_cast<bool>(file), "fixture write failed");
        };
        save("command.frame", frame);
        save("zero.frame", Codec::encode(zero));
        for (int size : {0, 1, 253, 254, 255, 256, 512, 1024}) {
            std::vector<uint8_t> payload(size);
            for (int i = 0; i < size; ++i) payload[i] = static_cast<uint8_t>(i);
            const auto name = "pattern-" + std::to_string(size) + ".frame";
            save(name.c_str(), Codec::encode_payload(payload));
        }
        return 0;
    }

    Pty pty;
    std::mutex mutex;
    std::vector<std::string> received;
    comms::UARTComms::Config config;
    config.device_name = pty.path;
    config.max_payload_bytes = 128;
    config.logger = [](auto, const auto&) { throw std::runtime_error("intentional logger failure"); };
    comms::UARTComms driver(config, command, [&](auto message) {
        std::lock_guard lock(mutex);
        received.push_back(message->SerializeAsString());
        if (received.size() == 2) throw std::runtime_error("intentional callback failure");
    });
    require(!driver.send(command), "send accepted while stopped");
    driver.start();
    driver.start();
    // Fragment a real frame, then combine invalid and valid frames in the stream.
    for (auto byte : frame) pty.write_frame({byte});
    auto bad_crc = frame;
    bad_crc[2] ^= 1; // This is a payload byte, not a COBS code in this fixture.
    pty.write_frame(bad_crc);
    pty.write_frame({4, 1, 0}); // COBS code promises bytes that are absent.
    pty.write_frame(Codec::encode_payload({0x80})); // CRC-valid invalid protobuf.
    pty.write_frame(std::vector<uint8_t>(140, 1));
    pty.write_frame({0, 0});
    pty.write_frame(Codec::encode(zero));
    eventually([&] { return driver.stats().rx_frames == 2; });
    eventually([&] { return driver.stats().callback_errors == 1; });
    const auto stats = driver.stats();
    require(stats.crc_errors == 1 && stats.cobs_errors == 1 && stats.protobuf_errors == 1 &&
            stats.oversize_errors == 1 && stats.logger_errors == 2, "RX rejection counters incorrect");
    {
        std::lock_guard lock(mutex);
        require(received.size() == 2 && received[0] == command.SerializeAsString() &&
                received[1] == zero.SerializeAsString(), "decoded messages changed or invalid frame delivered");
    }
    require(driver.send(command), "running send rejected");
    require(pty.read_bytes(frame.size()) == frame, "transmitted frame mismatch");
    eventually([&] { return driver.stats().tx_completed_frames == 1; });
    driver.stop();
    driver.stop();
    require(driver.stats().outstanding_tx_bytes == 0, "TX reservation leaked");
    driver.start();
    require(driver.send(zero), "send rejected after restart");
    const auto zero_frame = Codec::encode(zero);
    require(pty.read_bytes(zero_frame.size()) == zero_frame, "restart transmission failed");
    driver.stop();

    config.logger = {};
    config.max_outstanding_tx_bytes = 1;
    comms::UARTComms bounded(config, command, [](auto) {});
    bounded.start();
    require(!bounded.send(command) && bounded.stats().send_rejections == 1, "TX capacity not enforced");
    pty.write_frame(zero_frame);
    eventually([&] { return bounded.stats().rx_frames == 1; });
    require(bounded.stats().logger_errors == 0, "empty logger was not optional");
    bounded.stop();
    std::cout << "UART command, codec, diagnostics, and PTY lifecycle checks passed\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
