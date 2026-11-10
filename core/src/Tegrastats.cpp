#include "Tegrastats.hpp"
#include "Telemetry.hpp"

#include <cerrno>
#include <algorithm>
#include <cctype>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <regex>
#include <spawn.h>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

std::shared_ptr<hytech_msgs::TegraStats> core::parse_tegrastats(const std::string& line) {
    static const std::regex ram(R"(RAM (\d+)/(\d+)MB)");
    static const std::regex cpus(R"(CPU \[([^\]]+)\])");
    static const std::regex cpu(R"((\d+)%(@(\d+))?)");
    static const std::regex gpu(R"(GR3D_FREQ (\d+)%)");
    static const std::regex temperature(R"((\w+)@(-?\d+(?:\.\d+)?)C)");
    static const std::regex power(R"((\w+) (\d+(?:\.\d+)?)mW)");

    std::smatch match;
    if (!std::regex_search(line, match, ram)) return nullptr;
    const double used_mb = std::stod(match[1]);
    const double total_mb = std::stod(match[2]);
    if (total_mb <= 0) return nullptr;
    auto msg = std::make_shared<hytech_msgs::TegraStats>();
    msg->set_ram_used_mb(used_mb);
    msg->set_ram_total_mb(total_mb);
    msg->set_ram_used_percent(100.0 * used_mb / total_mb);

    // Core and sensor names correspond to flat fields in the shared schema.
    auto set_field = [&](const std::string& name, double value) {
        const auto* field = msg->GetDescriptor()->FindFieldByName(name);
        if (!field) return;
        if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_BOOL) {
            msg->GetReflection()->SetBool(msg.get(), field, value != 0);
        } else {
            msg->GetReflection()->SetFloat(msg.get(), field, value);
        }
    };

    if (std::regex_search(line, match, cpus)) {
        std::istringstream entries(match[1].str());
        std::string entry;
        int index = 0;
        while (std::getline(entries, entry, ',')) {
            const std::string prefix = "cpu" + std::to_string(index++);
            if (std::regex_search(entry, match, cpu)) {
                set_field(prefix + "online", true);
                set_field(prefix + "usage_percent", std::stod(match[1]));
                if (match[3].matched) set_field(prefix + "frequency_mhz", std::stod(match[3]));
            }
        }
    }
    if (std::regex_search(line, match, gpu)) msg->set_gpu_usage_percent(std::stod(match[1]));
    // The first slash-separated value is the instantaneous reading.
    for (std::sregex_iterator it(line.begin(), line.end(), temperature), end; it != end; ++it) {
        set_field((*it)[1].str() + "_temp_c", std::stod((*it)[2]));
    }
    for (std::sregex_iterator it(line.begin(), line.end(), power), end; it != end; ++it) {
        std::string rail = (*it)[1].str();
        std::transform(rail.begin(), rail.end(), rail.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        set_field(rail + "_mw", std::stod((*it)[2]));
    }
    return msg;
}

core::Tegrastats::Tegrastats() {
    int output[2];
    if (pipe(output) != 0) {
        spdlog::warn("Cannot open tegrastats pipe: {}", std::strerror(errno));
        return;
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, output[1], STDOUT_FILENO);
    posix_spawn_file_actions_addclose(&actions, output[0]);
    posix_spawn_file_actions_addclose(&actions, output[1]);
    char command[] = "tegrastats", interval[] = "--interval", milliseconds[] = "1000";
    char* args[] = {command, interval, milliseconds, nullptr};
    pid_t pid;
    int error = posix_spawnp(&pid, command, &actions, nullptr, args, environ);
    posix_spawn_file_actions_destroy(&actions);
    close(output[1]);
    if (error != 0) {
        close(output[0]);
        spdlog::warn("Cannot start tegrastats: {}", std::strerror(error));
        return;
    }
    _pid = pid;
    _output = fdopen(output[0], "r");
    if (!_output) {
        close(output[0]);
        kill(_pid, SIGTERM);
        while (waitpid(_pid, nullptr, 0) < 0 && errno == EINTR) {}
        _pid = -1;
        spdlog::warn("Cannot read tegrastats output");
        return;
    }
    _thread = std::thread([this] {
        char* line = nullptr;
        size_t capacity = 0;
        while (getline(&line, &capacity, _output) != -1) {
            try {
                if (auto msg = parse_tegrastats(line)) core::log(msg);
            } catch (const std::exception& e) {
                spdlog::warn("Cannot log tegrastats sample: {}", e.what());
            }
        }
        free(line);
        spdlog::warn("tegrastats output stopped");
    });
}

core::Tegrastats::~Tegrastats() {
    if (_pid < 0) return;
    kill(_pid, SIGTERM);
    if (_thread.joinable()) _thread.join();
    fclose(_output);
    while (waitpid(_pid, nullptr, 0) < 0 && errno == EINTR) {}
}
