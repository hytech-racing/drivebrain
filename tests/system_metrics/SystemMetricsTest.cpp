#include "SystemMetrics.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

namespace fs = std::filesystem;
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void write(const fs::path& path, const std::string& contents) {
    std::ofstream(path) << contents;
}
int main() {
    const auto root = fs::temp_directory_path() / ("drivebrain-metrics-" + std::to_string(getpid()));
    fs::create_directories(root / "self");
    fs::create_directories(root / "thermal");
    try {
        core::SystemMetricsSampler sampler(root.string(), (root / "thermal").string());
        const auto missing = sampler.sample();
        require(missing["cpu_usage_percent"].is_null(), "missing CPU must be null");
        require(missing["process_rss_bytes"].is_null(), "missing RSS must be null");
        require(missing["thermal_zone0_temperature_c"].is_null(), "missing thermal must be null");
        write(root / "stat", "cpu 100 0 0 100 0 0 0 0 50 0\ncpu0 100 0 0 100 0 0 0 0 50 0\n");
        // Fields 3..13 precede user/system CPU ticks. Name deliberately contains ')'.
        write(root / "self/stat", "1 (drive ) brain) S 0 0 0 0 0 0 0 0 0 0 10 20\n");
        write(root / "self/status", "Name:\tdrivebrain\nVmRSS:\t2048 kB\n");
        write(root / "meminfo", "MemTotal: 8192 kB\nMemAvailable: 4096 kB\n");
        write(root / "thermal/temp", "42500\n");
        write(root / "thermal/type", "cpu-thermal\n");
        const auto baseline = sampler.sample();
        require(baseline["cpu_usage_percent"].is_null(), "first CPU reading needs baseline");
        require(baseline["process_cpu_usage_percent"].is_null(), "first process reading needs baseline");
        require(baseline["process_rss_bytes"] == 2097152, "RSS units");
        require(baseline["memory_available_bytes"] == 4194304, "available memory units");
        require(baseline["memory_total_bytes"] == 8388608, "total memory units");
        require(baseline["thermal_zone0_temperature_c"] == 42.5, "thermal units");
        write(root / "stat", "cpu 150 0 0 150 0 0 0 0 100 0\ncpu0 150 0 0 150 0 0 0 0 100 0\n");
        write(root / "self/stat", "1 (drive ) brain) S 0 0 0 0 0 0 0 0 0 0 20 20\n");
        const auto sample = sampler.sample();
        require(sample["cpu_usage_percent"] == 50.0, "CPU delta must exclude duplicate guest ticks");
        require(sample["cpu_usage_percent_per_core"]["cpu0"] == 50.0, "per-core CPU");
        const double expected = 100.0 * (10.0 / sysconf(_SC_CLK_TCK)) / sample["sample_interval_s"].get<double>();
        require(std::abs(sample["process_cpu_usage_percent"].get<double>() - expected) < expected * 1e-9, "process CPU uses real sample duration");
        write(root / "stat", "cpu 1 0 0 1 0 0 0 0\n");
        require(sampler.sample()["cpu_usage_percent"].is_null(), "counter reset must not underflow");
        write(root / "stat", "cpu broken\n");
        write(root / "self/stat", "malformed\n");
        const auto malformed = sampler.sample();
        require(malformed["cpu_usage_percent"].is_null(), "malformed CPU must be null");
        require(malformed["process_cpu_usage_percent"].is_null(), "malformed process must be null");

        std::mutex mutex;
        std::condition_variable cv;
        int samples = 0;
        auto start = std::chrono::steady_clock::now();
        {
            core::SystemMetricsMonitor monitor([&](const nlohmann::json& record) {
                require(record.contains("sample_interval_s"), "monitor record");
                std::lock_guard<std::mutex> lock(mutex);
                ++samples;
                cv.notify_one();
            });
            std::unique_lock<std::mutex> lock(mutex);
            require(cv.wait_for(lock, std::chrono::seconds(4), [&] { return samples > 0; }), "monitor publishes");
            require(std::chrono::steady_clock::now() - start >= std::chrono::seconds(1), "monitor must not publish before interval");
            start = std::chrono::steady_clock::now();
        }
        require(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500), "shutdown interrupts wait");
        fs::remove_all(root);
        std::cout << "System metrics tests passed\n";
    } catch (const std::exception& error) {
        fs::remove_all(root);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
