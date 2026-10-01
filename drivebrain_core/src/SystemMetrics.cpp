#include "SystemMetrics.hpp"

#include <array>
#include <fstream>
#include <sstream>
#include <utility>
#include <unistd.h>
#include <spdlog/spdlog.h>

namespace core {
SystemMetricsSampler::SystemMetricsSampler(std::string proc_root, std::string thermal_root)
    : _proc_root(std::move(proc_root)), _thermal_root(std::move(thermal_root)) {}

nlohmann::json SystemMetricsSampler::schema() {
    const nlohmann::json number = {{"type", {"number", "null"}}};
    return {{"type", "object"}, {"properties", {
        {"sample_interval_s", number},
        {"cpu_usage_percent", number},
        {"cpu_usage_percent_per_core", {{"type", "object"}, {"additionalProperties", number}}},
        {"process_cpu_usage_percent", number},
        {"process_rss_bytes", number},
        {"memory_available_bytes", number},
        {"memory_total_bytes", number},
        {"thermal_zone0_temperature_c", number},
        {"thermal_zone0_type", {{"type", {"string", "null"}}}}
    }}};
}

nlohmann::json SystemMetricsSampler::sample() {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = _previous_time.time_since_epoch().count() == 0 ? 0.0 :
        std::chrono::duration<double>(now - _previous_time).count();
    nlohmann::json result = {
        {"sample_interval_s", elapsed > 0 ? nlohmann::json(elapsed) : nlohmann::json(nullptr)},
        {"cpu_usage_percent", nullptr},
        {"cpu_usage_percent_per_core", nlohmann::json::object()},
        {"process_cpu_usage_percent", nullptr},
        {"process_rss_bytes", nullptr}, {"memory_available_bytes", nullptr},
        {"memory_total_bytes", nullptr}, {"thermal_zone0_temperature_c", nullptr},
        {"thermal_zone0_type", nullptr}
    };

    std::ifstream stat(_proc_root + "/stat");
    std::string line;
    std::map<std::string, CpuTicks> current_cpu;
    while (std::getline(stat, line)) {
        std::istringstream fields(line);
        std::string name;
        fields >> name;
        if (name != "cpu" && (name.size() <= 3 || name.substr(0, 3) != "cpu" ||
            name.find_first_not_of("0123456789", 3) != std::string::npos)) continue;
        // Guest counters are already included in user/nice; do not count twice.
        std::array<uint64_t, 8> ticks{};
        bool valid = true;
        for (auto& tick : ticks) if (!(fields >> tick)) { valid = false; break; }
        if (!valid) continue;
        CpuTicks current{0, ticks[3] + ticks[4]};
        for (auto tick : ticks) current.total += tick;
        current_cpu.emplace(name, current);
        nlohmann::json usage = nullptr;
        const auto previous = _previous_cpu.find(name);
        if (previous != _previous_cpu.end() && current.total > previous->second.total &&
            current.idle >= previous->second.idle) {
            const auto total_delta = current.total - previous->second.total;
            const auto idle_delta = current.idle - previous->second.idle;
            if (idle_delta <= total_delta)
                usage = 100.0 * (total_delta - idle_delta) / total_delta;
        }
        if (name == "cpu") result["cpu_usage_percent"] = usage;
        else result["cpu_usage_percent_per_core"][name] = usage;
    }
    _previous_cpu = std::move(current_cpu);

    // comm (field 2) can contain spaces and parentheses. Start after its final ')'.
    std::ifstream process_stat(_proc_root + "/self/stat");
    std::optional<double> process_seconds;
    if (std::getline(process_stat, line)) {
        const auto end_name = line.rfind(')');
        if (end_name != std::string::npos) {
            std::istringstream fields(line.substr(end_name + 1));
            std::string ignored;
            for (int field = 3; field < 14; ++field) fields >> ignored;
            uint64_t user_ticks, system_ticks;
            const long ticks_per_second = sysconf(_SC_CLK_TCK);
            if ((fields >> user_ticks >> system_ticks) && ticks_per_second > 0)
                process_seconds = (static_cast<double>(user_ticks) + system_ticks) / ticks_per_second;
        }
    }
    if (process_seconds && _previous_process_seconds && elapsed > 0 &&
        *process_seconds >= *_previous_process_seconds)
        result["process_cpu_usage_percent"] =
            100.0 * (*process_seconds - *_previous_process_seconds) / elapsed;
    _previous_process_seconds = process_seconds;
    _previous_time = now;

    const auto read_memory = [&](const std::string& path, bool process) {
        std::ifstream file(path);
        while (std::getline(file, line)) {
            std::istringstream fields(line);
            std::string key, unit;
            uint64_t kib;
            if (!(fields >> key >> kib >> unit) || unit != "kB") continue;
            if (process && key == "VmRSS:") result["process_rss_bytes"] = kib * 1024;
            if (!process && key == "MemAvailable:") result["memory_available_bytes"] = kib * 1024;
            if (!process && key == "MemTotal:") result["memory_total_bytes"] = kib * 1024;
        }
    };
    read_memory(_proc_root + "/self/status", true);
    read_memory(_proc_root + "/meminfo", false);
    std::ifstream temperature(_thermal_root + "/temp");
    double millidegrees;
    if (temperature >> millidegrees) result["thermal_zone0_temperature_c"] = millidegrees / 1000.0;
    std::ifstream type(_thermal_root + "/type");
    if (std::getline(type, line)) result["thermal_zone0_type"] = line;
    return result;
}

SystemMetricsMonitor::SystemMetricsMonitor(std::function<void(const nlohmann::json&)> publish)
    : _thread([this, publish = std::move(publish)] {
        try {
            SystemMetricsSampler sampler;
            sampler.sample(); // Prime CPU deltas; first published sample follows in one second.
            std::unique_lock<std::mutex> lock(_mutex);
            while (!_cv.wait_for(lock, std::chrono::seconds(1), [this] { return _stop; })) {
                lock.unlock();
                publish(sampler.sample());
                lock.lock();
            }
        } catch (const std::exception& error) {
            spdlog::error("System metrics sampler stopped: {}", error.what());
        }
    }) {}

SystemMetricsMonitor::~SystemMetricsMonitor() {
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _stop = true;
    }
    _cv.notify_one();
    if (_thread.joinable()) _thread.join();
}
} // namespace core
