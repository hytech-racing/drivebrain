#include "UARTComms.hpp"
#include "uart_bench.pb.h"
#include "uart_bench_schema.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <deque>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_set>

using Clock = std::chrono::steady_clock;
static std::atomic<bool> running{true};

namespace comms {
// Bench-only access: malformed frames still use the driver's one bounded writer.
struct UARTBenchAccess {
    static bool submit(UARTComms& uart, std::vector<uint8_t> bytes) { return uart._submit_frame(std::move(bytes)); }
    static auto frame(const std::vector<uint8_t>& bytes) { return UARTComms::_encode_uart_frame(bytes, 4096); }
};
}

std::string quoted(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c) << std::dec;
        else out << c;
    }
    return out.str() + '"';
}

struct Options {
    std::string device, kind = "command", phase = "standalone";
    double seconds = 10, rate = 250;
    uint64_t run_id = 1, seed = 1979;
    unsigned baud = 921600, pattern_bytes = 0, cpu_workers = 0;
    bool identity = false, describe = false;
};

Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--identity") { o.identity = true; continue; }
        if (key == "--describe") { o.describe = true; continue; }
        if (i + 1 == argc) throw std::invalid_argument("missing value for " + key);
        const std::string v = argv[++i];
        if (key == "--device") o.device = v;
        else if (key == "--kind") o.kind = v;
        else if (key == "--phase") o.phase = v;
        else if (key == "--seconds") o.seconds = std::stod(v);
        else if (key == "--rate") o.rate = std::stod(v);
        else if (key == "--run-id") o.run_id = std::stoull(v);
        else if (key == "--seed") o.seed = std::stoull(v);
        else if (key == "--baud") o.baud = std::stoul(v);
        else if (key == "--pattern-bytes") o.pattern_bytes = std::stoul(v);
        else if (key == "--cpu-workers") o.cpu_workers = std::stoul(v);
        else throw std::invalid_argument("unknown argument: " + key);
    }
    if (o.identity) return o;
    if ((!o.describe && o.device.empty()) || !o.run_id || !o.baud || !std::isfinite(o.seconds) || o.seconds <= 0 || o.seconds > 86400 ||
        !std::isfinite(o.rate) || o.rate <= 0 || o.rate > 100000 || o.pattern_bytes > 1024 || o.cpu_workers > 4)
        throw std::invalid_argument("invalid device/run/baud/duration/rate/pattern/CPU configuration");
    if (o.kind != "command" && o.kind != "pattern" && o.kind != "raw" && o.kind != "faults" && o.kind != "overload")
        throw std::invalid_argument("unknown phase kind");
    return o;
}

hytech_msgs::MCUCommandData command_for(uint64_t sequence, uint64_t seed) {
    const float n = static_cast<float>((sequence + seed) % 18000);
    return comms::make_drivetrain_uart_command(
        {n, n + 1, n + 2, n + 3}, {1.0f, 2.125f, 3.25f, static_cast<float>(sequence % 100) / 8.0f},
        static_cast<int64_t>(sequence));
}

uart_bench::Request request_for(const Options& o, uint64_t sequence) {
    uart_bench::Request request;
    request.set_version(1);
    request.set_run_id(o.run_id);
    request.set_sequence(sequence);
    if (o.kind == "pattern") {
        std::string pattern(o.pattern_bytes, '\0');
        for (std::size_t i = 0; i < pattern.size(); ++i) pattern[i] = static_cast<char>((i * 31 + sequence * 17 + o.seed) & 255);
        request.set_pattern(pattern);
    } else *request.mutable_command() = command_for(sequence, o.seed);
    return request;
}

struct Pending { Clock::time_point sent; std::string body; };
struct State {
    std::mutex mutex;
    std::map<uint64_t, Pending> pending;
    std::unordered_set<uint64_t> completed;
    std::deque<uint64_t> recent;
    std::vector<double> rtts;
    std::vector<std::string> examples;
    uint64_t offered = 0, accepted = 0, verified = 0, missing = 0, mismatch = 0, duplicate = 0,
        reordered = 0, unexpected = 0, rejected = 0, missed_slots = 0, injected = 0, last_sequence = 0;
    std::size_t pending_high_water = 0;
    double max_lag_ms = 0;
    void example(const std::string& text) { if (examples.size() < 20) examples.push_back(text); }
};

double percentile(std::vector<double> values, double quantile) {
    if (values.empty()) return 0;
    const auto index = static_cast<std::size_t>((values.size() - 1) * quantile);
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

void receive(State& s, const Options& o, std::shared_ptr<const google::protobuf::Message> message) {
    const auto now = Clock::now();
    std::lock_guard lock(s.mutex);
    uint64_t sequence;
    std::string actual;
    if (o.kind == "raw" || o.kind == "command" || o.kind == "overload") {
        const auto& reply = static_cast<const hytech_msgs::MCUCommandData&>(*message);
        sequence = static_cast<uint64_t>(reply.prev_mcu_recv_millis());
        actual = reply.SerializeAsString();
    } else {
        const auto& reply = static_cast<const uart_bench::Reply&>(*message);
        if (reply.version() != 1 || reply.run_id() != o.run_id) {
            ++s.mismatch; s.example("wrong reply version/run ID"); return;
        }
        sequence = reply.sequence();
        actual = reply.has_command() ? reply.command().SerializeAsString() : reply.pattern();
        if ((o.kind == "pattern") != reply.has_pattern()) {
            ++s.mismatch; s.example("wrong reply body type"); return;
        }
    }
    const auto found = s.pending.find(sequence);
    if (found == s.pending.end()) {
        if (s.completed.count(sequence)) ++s.duplicate;
        else ++s.unexpected;
        s.example("duplicate, late, or unknown reply: " + std::to_string(sequence));
        return;
    }
    if (actual != found->second.body) { ++s.mismatch; s.example("value mismatch: " + std::to_string(sequence)); }
    else ++s.verified;
    if (sequence < s.last_sequence) ++s.reordered;
    s.last_sequence = std::max(s.last_sequence, sequence);
    if (s.rtts.size() < 1000000) s.rtts.push_back(std::chrono::duration<double, std::milli>(now - found->second.sent).count());
    s.pending.erase(found);
    s.completed.insert(sequence);
    s.recent.push_back(sequence);
    if (s.recent.size() > 4096) { s.completed.erase(s.recent.front()); s.recent.pop_front(); }
}

void expire(State& s, Clock::time_point now) {
    while (!s.pending.empty() && now - s.pending.begin()->second.sent >= std::chrono::seconds(1)) {
        ++s.missing;
        s.example("reply deadline: " + std::to_string(s.pending.begin()->first));
        s.pending.erase(s.pending.begin());
    }
}

std::vector<uint8_t> fault(unsigned index, const Options& o) {
    using Access = comms::UARTBenchAccess;
    switch (index % 6) {
        case 0: { auto bytes = Access::frame({0x42}); bytes[1] ^= 1; return bytes; }
        case 1: return {4, 1, 0};
        case 2: return Access::frame({0x80});
        case 3: { std::vector<uint8_t> bytes(2200, 1); bytes.push_back(0); return bytes; }
        case 4: { auto r = request_for(o, UINT64_MAX); r.mutable_command()->mutable_torque_limit_nm()->set_fl(-1);
                  const auto p = r.SerializeAsString(); return Access::frame({p.begin(), p.end()}); }
        default: return {5, 1, 2, 0}; // Truncated block followed by a deliberate delimiter.
    }
}

int main(int argc, char** argv) try {
    const auto o = parse(argc, argv);
    if (o.identity) {
        std::cout << "{\"event\":\"identity\",\"version\":1,\"schema_sha256\":\"" UART_BENCH_SCHEMA_SHA256 "\"}\n";
        return 0;
    }
    if (o.describe) {
        const auto request = request_for(o, UINT64_MAX);
        uart_bench::Reply response;
        response.set_version(1); response.set_run_id(o.run_id); response.set_sequence(UINT64_MAX);
        if (request.has_pattern()) response.set_pattern(request.pattern());
        else *response.mutable_command() = request.command();
        response.set_processing_us(UINT32_MAX);
        const auto wire_bound = [](std::size_t payload) { return payload + 2 + (payload + 2) / 254 + 2; };
        std::cout << "{\"request_wire_max\":" << wire_bound(request.ByteSizeLong())
                  << ",\"reply_wire_max\":" << wire_bound(response.ByteSizeLong()) << "}" << std::endl;
        return 0;
    }
    std::signal(SIGINT, [](int) { running.store(false); });
    std::signal(SIGTERM, [](int) { running.store(false); });
    State state;
    hytech_msgs::MCUCommandData raw;
    uart_bench::Reply reply;
    comms::UARTComms::Config config;
    config.device_name = o.device;
    config.baud_rate = o.baud;
    config.max_payload_bytes = 1536;
    config.max_outstanding_tx_bytes = 4096;
    comms::UARTComms uart(config, (o.kind == "raw" || o.kind == "command" || o.kind == "overload") ? static_cast<const google::protobuf::Message&>(raw) : reply,
        [&](auto message) { receive(state, o, message); });
    uart.start();
    const auto start = Clock::now();
    const auto finish = start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(o.seconds));
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / o.rate));
    auto next = start, report = start, next_fault = start;
    uint64_t sequence = 0;
    std::atomic<bool> load_running{true};
    std::vector<std::thread> load;
    // Explicit RAII cleanup also covers a UART/send exception while load is active.
    struct LoadCleanup {
        std::atomic<bool>& flag;
        std::vector<std::thread>& threads;
        ~LoadCleanup() { flag.store(false); for (auto& t : threads) if (t.joinable()) t.join(); }
    } cleanup{load_running, load};
    for (unsigned i = 0; i < o.cpu_workers; ++i) load.emplace_back([&] {
        volatile uint64_t value = 1;
        while (load_running.load()) {
            const auto end = Clock::now() + std::chrono::milliseconds(75);
            while (load_running.load() && Clock::now() < end) value = value * 1664525 + 1013904223;
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    });

    const auto emit = [&](const char* event) {
        std::lock_guard lock(state.mutex);
        const auto stats = uart.stats();
        const bool overload = o.kind == "overload";
        const auto errors = state.mismatch + state.duplicate + state.reordered + stats.io_errors +
            stats.cobs_errors + stats.crc_errors + stats.protobuf_errors + stats.oversize_errors +
            (overload ? 0 : state.missing + state.unexpected + state.rejected);
        std::cout << "{\"event\":" << quoted(event) << ",\"phase\":" << quoted(o.phase)
            << ",\"kind\":" << quoted(o.kind) << ",\"schema_sha256\":\"" UART_BENCH_SCHEMA_SHA256 "\""
            << ",\"elapsed\":" << std::chrono::duration<double>(Clock::now() - start).count()
            << ",\"requested_rate\":" << o.rate << ",\"offered\":" << state.offered
            << ",\"accepted\":" << state.accepted << ",\"verified\":" << state.verified
            << ",\"missing\":" << state.missing << ",\"mismatch\":" << state.mismatch
            << ",\"duplicate\":" << state.duplicate << ",\"reordered\":" << state.reordered
            << ",\"unexpected\":" << state.unexpected << ",\"queue_rejected\":" << state.rejected
            << ",\"injected\":" << state.injected << ",\"missed_slots\":" << state.missed_slots
            << ",\"max_lag_ms\":" << state.max_lag_ms << ",\"pending\":" << state.pending.size()
            << ",\"pending_high_water\":" << state.pending_high_water << ",\"tx_high_water_bytes\":" << stats.tx_high_water_bytes
            << ",\"tx_frames\":" << stats.tx_completed_frames << ",\"tx_bytes\":" << stats.tx_bytes << ",\"rx_bytes\":" << stats.rx_bytes
            << ",\"rx_crc_errors\":" << stats.crc_errors << ",\"rx_cobs_errors\":" << stats.cobs_errors
            << ",\"rx_protobuf_errors\":" << stats.protobuf_errors << ",\"io_errors\":" << stats.io_errors
            << ",\"rtt_p50_ms\":" << percentile(state.rtts, .5) << ",\"rtt_p95_ms\":" << percentile(state.rtts, .95)
            << ",\"rtt_p99_ms\":" << percentile(state.rtts, .99) << ",\"rtt_max_ms\":" << percentile(state.rtts, 1)
            << ",\"errors\":" << errors << ",\"interrupted\":" << (!running.load() ? "true" : "false")
            << ",\"error_examples\":[";
        for (std::size_t i = 0; i < state.examples.size(); ++i) std::cout << (i ? "," : "") << quoted(state.examples[i]);
        std::cout << "]}" << std::endl;
        return errors;
    };
    while (running.load() && Clock::now() < finish) {
        const auto now = Clock::now();
        if (!uart.is_running()) throw std::runtime_error("UART I/O stopped");
        if (now >= report) { emit("stats"); report = now + std::chrono::seconds(1); }
        if (o.kind == "faults" && now >= next_fault) {
            const auto bytes = fault(static_cast<unsigned>(state.injected), o);
            if (!comms::UARTBenchAccess::submit(uart, bytes)) throw std::runtime_error("fault injector queue rejected");
            ++state.injected;
            next_fault = now + std::chrono::milliseconds(200);
        }
        if (now < next) { std::this_thread::sleep_until(next); continue; }
        const auto lag = now - next;
        {
            std::lock_guard lock(state.mutex);
            expire(state, now);
            state.max_lag_ms = std::max(state.max_lag_ms, std::chrono::duration<double, std::milli>(lag).count());
            if (lag >= period) { const auto missed = lag / period; state.missed_slots += missed; next += period * missed; }
        }
        ++sequence;
        const auto command = command_for(sequence, o.seed);
        const auto request = request_for(o, sequence);
        const auto body = o.kind == "pattern" ? request.pattern() : command.SerializeAsString();
        bool admitted;
        {
            std::lock_guard lock(state.mutex);
            ++state.offered;
            if (state.pending.size() >= 4096) { ++state.rejected; next += period; continue; }
            state.pending.emplace(sequence, Pending{Clock::now(), body});
            admitted = (o.kind == "raw" || o.kind == "command" || o.kind == "overload") ? uart.send(command) : uart.send(request);
            if (admitted) { ++state.accepted; state.pending_high_water = std::max(state.pending_high_water, state.pending.size()); }
            else { state.pending.erase(sequence); ++state.rejected; }
        }
        next += period;
    }
    load_running.store(false);
    const auto drain_until = Clock::now() + std::chrono::seconds(2);
    while (Clock::now() < drain_until) {
        { std::lock_guard lock(state.mutex); expire(state, Clock::now()); if (state.pending.empty()) break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    uart.stop();
    const auto errors = emit("result");
    return errors || !running.load() ? 1 : 0;
} catch (const std::exception& error) {
    std::cout << "{\"event\":\"fatal\",\"message\":" << quoted(error.what()) << "}" << std::endl;
    return 2;
}
