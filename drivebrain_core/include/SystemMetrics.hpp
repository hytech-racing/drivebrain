#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <nlohmann/json.hpp>

namespace core {

// Linux counters; alternate roots allow deterministic tests without real hardware.
class SystemMetricsSampler {
public:
    explicit SystemMetricsSampler(std::string proc_root = "/proc",
                                  std::string thermal_root = "/sys/class/thermal/thermal_zone0");
    nlohmann::json sample();
    static nlohmann::json schema();

private:
    struct CpuTicks { uint64_t total; uint64_t idle; };
    std::string _proc_root;
    std::string _thermal_root;
    std::map<std::string, CpuTicks> _previous_cpu;
    std::optional<double> _previous_process_seconds;
    std::chrono::steady_clock::time_point _previous_time{};
};

// Owns its thread and joins before destruction; callback never runs on control.
class SystemMetricsMonitor {
public:
    explicit SystemMetricsMonitor(std::function<void(const nlohmann::json&)> publish);
    ~SystemMetricsMonitor();
    SystemMetricsMonitor(const SystemMetricsMonitor&) = delete;
    SystemMetricsMonitor& operator=(const SystemMetricsMonitor&) = delete;
private:
    std::mutex _mutex;
    std::condition_variable _cv;
    bool _stop = false;
    std::thread _thread;
};

} // namespace core
