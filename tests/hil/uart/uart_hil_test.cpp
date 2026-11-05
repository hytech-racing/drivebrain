#include "UARTComms.hpp"
#include "DrivetrainUARTCommand.hpp"
#include "uart_bench.pb.h"
#include <google/protobuf/util/message_differencer.h>
#include <openssl/rand.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace {
struct Options {
    std::string device, role = "pi";
    unsigned count = 1000, baud = 115200, pattern_bytes = 256, max_payload = 512;
    unsigned timeout_ms = 5000, interval_ms = 10;
    std::array<float, 4> rpms{100, 200, -300, 400}, torques{1, 2, 3, 4};
    bool throwing_logger = false;
};
unsigned number(const std::string& value) {
    std::size_t used;
    const auto n = std::stoul(value, &used);
    if (used != value.size() || value.empty() || value[0] == '-' || n > 100000000) throw std::invalid_argument("Invalid numeric argument");
    return static_cast<unsigned>(n);
}
std::array<float, 4> wheels(const std::string& value) {
    std::array<float, 4> out;
    std::istringstream input(value);
    std::string part;
    for (auto& wheel : out) {
        if (!std::getline(input, part, ',')) throw std::invalid_argument("Expected FL,FR,RL,RR");
        std::size_t used;
        wheel = std::stof(part, &used);
        if (used != part.size()) throw std::invalid_argument("Invalid wheel value");
    }
    if (std::getline(input, part, ',') || (!value.empty() && value.back() == ',')) throw std::invalid_argument("Expected exactly four wheel values");
    return out;
}
Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--throw-logger") { o.throwing_logger = true; continue; }
        if (arg == "--help") {
            std::cout << "uart_hil_test --device PORT [--role pi|peer] [--count N] [--baud N]\n"
                      << "  [--rpms FL,FR,RL,RR] [--torque-limits FL,FR,RL,RR] [--pattern-bytes 0..256]\n"
                      << "  [--max-payload N] [--timeout-ms N] [--interval-ms N]\n";
            std::exit(0);
        }
        if (++i == argc) throw std::invalid_argument("Missing value for " + arg);
        const std::string value = argv[i];
        if (arg == "--device") o.device = value;
        else if (arg == "--role") o.role = value;
        else if (arg == "--count") o.count = number(value);
        else if (arg == "--baud") o.baud = number(value);
        else if (arg == "--pattern-bytes") o.pattern_bytes = number(value);
        else if (arg == "--max-payload") o.max_payload = number(value);
        else if (arg == "--timeout-ms") o.timeout_ms = number(value);
        else if (arg == "--interval-ms") o.interval_ms = number(value);
        else if (arg == "--rpms") o.rpms = wheels(value);
        else if (arg == "--torque-limits") o.torques = wheels(value);
        else throw std::invalid_argument("Unknown option " + arg);
    }
    if (o.role != "pi" && o.role != "peer") throw std::invalid_argument("Role must be pi or peer");
    if ((o.device.empty() || !o.count || !o.baud || !o.max_payload || !o.timeout_ms || o.pattern_bytes > 256)) {
        throw std::invalid_argument("Device and positive count/baud/limit/timeout required; pattern <= 256 bytes");
    }
    return o;
}
struct Inbox {
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<std::shared_ptr<const comms::UARTComms::Message>> messages;
    void push(std::shared_ptr<const comms::UARTComms::Message> message) {
        std::lock_guard lock(mutex);
        if (messages.size() >= 1024) throw std::runtime_error("Bench receive queue overflow");
        messages.push_back(std::move(message));
        ready.notify_one();
    }
    auto next(std::chrono::milliseconds timeout) {
        std::unique_lock lock(mutex);
        if (!ready.wait_for(lock, timeout, [&] { return !messages.empty(); })) throw std::runtime_error("Reply/command deadline expired");
        auto message = messages.front(); messages.pop_front(); return message;
    }
};
void send(comms::UARTComms& uart, const google::protobuf::Message& message) {
    if (!uart.send(message)) throw std::runtime_error("UART stopped or transmit queue full");
}
}

int main(int argc, char** argv) {
    try {
        const auto options = parse(argc, argv);
        const auto wheel_command = comms::make_drivetrain_uart_command(options.rpms, options.torques);
        const auto timeout = std::chrono::milliseconds(options.timeout_ms);
        Inbox inbox;
        uart_bench::Command command_type;
        uart_bench::Reply reply_type;
        comms::UARTComms::Config config;
        config.device_name = options.device; config.baud_rate = options.baud; config.max_payload_bytes = options.max_payload;
        if (options.throwing_logger) {
            config.logger = [](auto, const auto&) { throw std::runtime_error("Injected logger failure"); };
        }
        const bool pi = options.role == "pi";
        // Inbox outlives uart: stop joins the worker before callback state is destroyed.
        comms::UARTComms uart(config, pi ? static_cast<const comms::UARTComms::Message&>(reply_type) : command_type,
                             [&](auto message) { inbox.push(std::move(message)); });
        uart.start();
        uint64_t run_id = 0;
        if (RAND_bytes(reinterpret_cast<unsigned char*>(&run_id), sizeof(run_id)) != 1 || !run_id) throw std::runtime_error("Run ID generation failed");
        std::vector<double> rtts;
        for (unsigned i = 1; i <= options.count; ++i) {
            if (pi) {
                uart_bench::Command command;
                command.set_run_id(run_id); command.set_sequence(i);
                *command.mutable_command() = wheel_command;
                std::string pattern(options.pattern_bytes, '\0');
                for (unsigned j = 0; j < options.pattern_bytes; ++j) pattern[j] = static_cast<char>((j + i) & 0xff);
                command.set_pattern(pattern);
                const auto before = Clock::now();
                send(uart, command);
                const auto message = inbox.next(timeout);
                const auto& reply = static_cast<const uart_bench::Reply&>(*message);
                if (!reply.has_decoded() || !google::protobuf::util::MessageDifferencer::Equals(command, reply.decoded())) {
                    throw std::runtime_error("Decoded reply mismatch (run, sequence, setpoints, limits or pattern)");
                }
                rtts.push_back(std::chrono::duration<double, std::milli>(Clock::now() - before).count());
                if (i != options.count) std::this_thread::sleep_for(std::chrono::milliseconds(options.interval_ms));
            } else {
                const auto message = inbox.next(timeout);
                const auto& command = static_cast<const uart_bench::Command&>(*message);
                if (!command.has_command() || !command.command().has_desired_rpms() || !command.command().has_torque_limit_nm() ||
                    !command.run_id() || command.sequence() != i || command.pattern().size() > 256 || (i > 1 && command.run_id() != run_id)) {
                    throw std::runtime_error("Invalid/out-of-order bench command");
                }
                run_id = command.run_id();
                const auto& rpm = command.command().desired_rpms();
                const auto& torque = command.command().torque_limit_nm();
                comms::make_drivetrain_uart_command({rpm.fl(), rpm.fr(), rpm.rl(), rpm.rr()}, {torque.fl(), torque.fr(), torque.rl(), torque.rr()});
                uart_bench::Reply reply;
                *reply.mutable_decoded() = command; // Decoded echo only: never drive actuators.
                send(uart, reply);
            }
        }
        if (!uart.wait_for_transmit_idle(timeout)) throw std::runtime_error("Local write deadline expired");
        if (!uart.is_running()) throw std::runtime_error("UART I/O stopped during the test");
        uart.stop();
        if (!rtts.empty()) {
            std::sort(rtts.begin(), rtts.end());
            std::cout << "RTT_ms p50=" << rtts[rtts.size()/2] << " p99=" << rtts[(rtts.size()-1)*99/100] << " max=" << rtts.back() << '\n';
        }
        std::cout << "PASS role=" << options.role << " count=" << options.count << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
