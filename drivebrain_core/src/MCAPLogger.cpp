#define MCAP_IMPLEMENTATION
#include <atomic>
#include <mutex>
#include <fstream>
#include <cerrno>
#include <cstring>

#if defined(__linux__) && JETSON_ENABLED
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
#endif

#include <MCAPLogger.hpp>

#if JETSON_ENABLED
#define RECORDINGS_DIR "/mnt/ssd/recordings"
#else
#define RECORDINGS_DIR "/home/nixos/recordings"
#endif

/****************************************************************
 * HELPER METHODS
 ****************************************************************/
static std::string get_logfile_name() {
  std::string dir_path = RECORDINGS_DIR;
  int max_file_number = 0;
  std::string largest_file_name; 

  try {
    for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
      if (entry.is_regular_file()) {
        std::string filename = entry.path().filename().string();
        try {
          int current_number = std::stoi(filename);
          if (current_number > max_file_number) {
              max_file_number = current_number;
          }
        } catch (const std::invalid_argument& e) {
          spdlog::error("Skipping non-numeric file: {}", filename);
        } 
      }
    }
  }  catch (const std::filesystem::filesystem_error& e) {
    spdlog::error("Filesystem error");
    return "sdfsdf.mcap";
  } 

  return std::to_string(max_file_number + 1) + ".mcap";
}

static std::vector<const google::protobuf::FileDescriptor *> get_pb_descriptors(const std::vector<std::string> &filenames) {
    std::vector<const google::protobuf::FileDescriptor *> descriptors;

    for (const auto &name : filenames) {
        const google::protobuf::FileDescriptor *file_descriptor = google::protobuf::DescriptorPool::generated_pool()->FindFileByName(name);

        if (!file_descriptor) {
            spdlog::error("File descriptor not found: {}", name);
        }
        else {
            descriptors.push_back(file_descriptor);
        }
    }
    return descriptors;
}

static std::string serialize_fd_set(const google::protobuf::Descriptor *toplevelDescriptor) {
    google::protobuf::FileDescriptorSet fdSet;
    std::queue<const google::protobuf::FileDescriptor *> toAdd;
    toAdd.push(toplevelDescriptor->file());
    std::unordered_set<std::string> seenDependencies;
    while (!toAdd.empty())
    {
        const google::protobuf::FileDescriptor *next = toAdd.front();
        toAdd.pop();
        next->CopyTo(fdSet.add_file());
        for (int i = 0; i < next->dependency_count(); ++i)
        {
            const auto &dep = next->dependency(i);
            if (seenDependencies.find(dep->name()) == seenDependencies.end())
            {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
            }
        }
    }
    return fdSet.SerializeAsString();
}

std::tuple<std::string, bool> core::MCAPLogger::status() {
    {
        std::lock_guard lock(_input_buffer_mutex);
        return std::make_tuple(_log_name, _logging);
    }
}

static nlohmann::json generate_json_schema(const nlohmann::json& obj) {
    nlohmann::json schema;
    
    if (obj.is_object()) {
        schema["type"] = "object";
        schema["properties"] = nlohmann::json::object();
        for (auto& [key, value] : obj.items()) {
            schema["properties"][key] = generate_json_schema(value);
        }
    } else if (obj.is_array()) {
        schema["type"] = "array";
        if (!obj.empty()) {
            schema["items"] = generate_json_schema(obj[0]);
        }
    } else if (obj.is_boolean()) {
        schema["type"] = "boolean";
    } else if (obj.is_number_integer()) {
        schema["type"] = "integer";
    } else if (obj.is_number_float()) {
        schema["type"] = "number";
    } else if (obj.is_string()) {
        schema["type"] = "string";
    } else if (obj.is_null()) {
        schema["type"] = "null";
    }
    
    return schema;
}
 
/****************************************************************
 * PUBLIC CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
void core::MCAPLogger::create(const std::string &base_dir, const mcap::McapWriterOptions &options, const std::string &params_file) {
    MCAPLogger* expected = nullptr;
    MCAPLogger* local = new MCAPLogger(base_dir, options, params_file);
    if(!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        // Already initialized, delete local instance
        delete local;
    }
}

core::MCAPLogger& core::MCAPLogger::instance() {
    MCAPLogger* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "MCAPLogger has not been initialized");
    return *instance;
}

void core::MCAPLogger::destroy() {
    MCAPLogger* instance = _s_instance.exchange(nullptr, std::memory_order_acq_rel);
    if (instance) {
        delete instance;
    }
}

int core::MCAPLogger::open_new_mcap() {
    std::string mcap_name = get_logfile_name();
    spdlog::info("Attempting to open new MCAP file: {}", mcap_name);
#if HOOTL_ENABLED
    _log_name = "sim_data.mcap";
#else
    _log_name = std::string(RECORDINGS_DIR) + "/" + get_logfile_name(); 
#endif

    const auto res = _writer.open(_log_name, _options);
    if (!res.ok()) {
        spdlog::error("Failed to open {} for writing: {}", _log_name, res.message);
        return -1;
    }

    std::vector<std::string> proto_names = {"hytech_msgs.proto", "ht_can.proto", "foxglove/PointCloud.proto", "foxglove/CompressedImage.proto", "foxglove/RawImage.proto"};

    _name_to_id_map.clear();
    _type_to_schema_id_map.clear();

    auto descriptors = get_pb_descriptors(proto_names);

    for (const auto &file_descriptor : descriptors) {
        for (int i = 0; i < file_descriptor->message_type_count(); i++) {
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
            mcap::Schema schema(message_descriptor->full_name(), "protobuf", serialize_fd_set(message_descriptor));
            _writer.addSchema(schema);
            mcap::Channel channel(message_descriptor->name(), "protobuf", schema.id);
            _writer.addChannel(channel);
            _name_to_id_map[message_descriptor->name()] = channel.id;
            _type_to_schema_id_map[message_descriptor->name()] = schema.id;
        }
    }

    spdlog::info("Successfully added message descriptions to mcap");

    mcap::Schema config_schema("drivebrain_configuration", "jsonschema", _params_schema_json.dump());
    _writer.addSchema(config_schema);
    mcap::Channel config_channel("drivebrain_configuration", "json", config_schema.id);
    _writer.addChannel(config_channel);
    _name_to_id_map["drivebrain_configuration"] = config_channel.id;

    mcap::Schema service_log_schema("drivebrain_service_logs", "jsonschema",
        R"({"type":"object","properties":{"message":{"type":"string"}},"required":["message"]})");
    _writer.addSchema(service_log_schema);
    mcap::Channel service_log_channel("drivebrain_service_logs", "json", service_log_schema.id);
    _writer.addChannel(service_log_channel);
    _name_to_id_map["drivebrain_service_logs"] = service_log_channel.id;

    log_params(_initial_params);

    spdlog::info("Successfully added params schema");
    
    return 0;
}

int core::MCAPLogger::close_active_mcap() {
    stop_logging();
    spdlog::info("Closing mcap");
    _writer.close(); 
    _log_name = "NONE";

    return 0;
}

void core::MCAPLogger::init_logging() {
    {
        std::lock_guard lock(_input_buffer_mutex);
        if (_logging) return;
        _logging = true;
    }
    _msg_log_thread = std::thread([this]() { _handle_log_to_file(); });
#if defined(__linux__) && JETSON_ENABLED
    _service_logs_running = true;
    _service_log_thread = std::thread([this]() { _handle_service_logs(); });
#endif
    spdlog::info("Msg log thread spawned");
}

void core::MCAPLogger::stop_logging() {
    // Stop the producer before draining the writer's queue.
    _service_logs_running = false;
    if (_service_log_thread.joinable()) _service_log_thread.join();
    {
        std::unique_lock lock(_input_buffer_mutex);
        _logging = false;
    }

    _input_buffer_cv.notify_one(); 
    if (_msg_log_thread.joinable()) {
        _msg_log_thread.join();
    }
}

int core::MCAPLogger::log_msg(core::MsgType message, const std::string &topic) {
    mcap::Timestamp log_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    RawMessage_s new_message; 
    new_message.log_time = log_time;
    new_message.serialized_data = message->SerializeAsString(); 
    new_message.type_name = message->GetDescriptor()->name();
    new_message.message_name = topic.empty() ? new_message.type_name : topic;
    {
        std::unique_lock lock(_input_buffer_mutex);
        if (!_logging) return -1; // Logging is not active, return an error code
        _input_buffer.push_back(std::move(new_message)); 
        _input_buffer_cv.notify_one(); 
   }

    return 0;
}

/****************************************************************
 * PRIVATE CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
core::MCAPLogger::MCAPLogger(const std::string &base_dir, const mcap::McapWriterOptions &options, const std::string &params_file) : _options(options) {
    std::fstream raw_param_file(params_file);
    nlohmann::json params_config = nlohmann::json::parse(raw_param_file);
    _initial_params = params_config; // Used in open_new_mcap to log initial params
    _params_schema_json = generate_json_schema(params_config);
}

void core::MCAPLogger::_queue_service_log(const std::string &entry) {
    // JSON output keeps embedded newlines/quotes intact and supplies journal time.
    const auto record = nlohmann::json::parse(entry, nullptr, false);
    if (!record.is_object() || !record.contains("MESSAGE") ||
        !record.contains("__REALTIME_TIMESTAMP")) return;

    try {
        const auto &text = record.at("MESSAGE");
        // Binary or repeated journal fields arrive as arrays; retain them as text.
        const std::string message = text.is_string() ? text.get<std::string>() : text.dump();
        RawMessage_s msg;
        msg.log_time = std::stoull(record.at("__REALTIME_TIMESTAMP").get<std::string>()) * 1000;
        msg.message_name = "drivebrain_service_logs";
        msg.serialized_data = nlohmann::json{{"message", message}}.dump();
        std::lock_guard lock(_input_buffer_mutex);
        if (!_logging) return;
        _input_buffer.push_back(std::move(msg));
        _input_buffer_cv.notify_one();
    } catch (const std::exception &) {
        // Skip malformed entries without feeding errors back into the journal.
    }
}

void core::MCAPLogger::_handle_service_logs() {
#if defined(__linux__) && JETSON_ENABLED
    int output[2];
    if (pipe2(output, O_CLOEXEC) != 0) {
        spdlog::warn("Cannot capture service logs: {}", std::strerror(errno));
        return;
    }

    posix_spawn_file_actions_t actions;
    int error = posix_spawn_file_actions_init(&actions);
    if (error == 0) {
        error = posix_spawn_file_actions_adddup2(&actions, output[1], STDOUT_FILENO);
        if (error == 0) error = posix_spawn_file_actions_addclose(&actions, output[0]);
        if (error == 0 && output[1] != STDOUT_FILENO)
            error = posix_spawn_file_actions_addclose(&actions, output[1]);
        if (error == 0) {
            // No shell, no historical backlog, no JSON field size truncation.
            const char *args[] = {"journalctl", "--unit=drivebrain-service.service",
                "--follow", "--lines=0", "--output=json", "--all", "--no-pager", nullptr};
            pid_t child;
            error = posix_spawnp(&child, args[0], &actions, nullptr,
                                const_cast<char **>(args), environ);
            posix_spawn_file_actions_destroy(&actions);
            close(output[1]);
            if (error == 0) {
                std::string pending;
                char buffer[4096];
                while (_service_logs_running) {
                    pollfd fd{output[0], POLLIN, 0};
                    const int ready = poll(&fd, 1, 100);
                    if (ready < 0 && errno == EINTR) continue;
                    if (ready < 0 || (fd.revents & (POLLERR | POLLNVAL))) break;
                    if (ready == 0) continue;
                    const auto count = read(output[0], buffer, sizeof(buffer));
                    if (count < 0 && errno == EINTR) continue;
                    if (count <= 0) break;
                    pending.append(buffer, static_cast<size_t>(count));
                    size_t start = 0;
                    size_t end;
                    while ((end = pending.find('\n', start)) != std::string::npos) {
                        _queue_service_log(pending.substr(start, end - start));
                        start = end + 1;
                    }
                    pending.erase(0, start);
                }
                if (_service_logs_running)
                    spdlog::warn("Service log capture stopped; check journalctl access for drivebrain-service.service");
                close(output[0]);
                // Bounded cleanup even if journalctl stops responding.
                kill(child, SIGTERM);
                int status;
                for (int attempt = 0; attempt < 5; ++attempt) {
                    const auto result = waitpid(child, &status, WNOHANG);
                    if (result == child || (result < 0 && errno == ECHILD)) return;
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                kill(child, SIGKILL);
                while (waitpid(child, &status, 0) < 0 && errno == EINTR) {}
                return;
            }
        } else {
            posix_spawn_file_actions_destroy(&actions);
            close(output[1]);
        }
    } else {
        close(output[1]);
    }
    close(output[0]);
    spdlog::warn("Cannot start journalctl for service log capture: {}", std::strerror(error));
#endif
}

void core::MCAPLogger::_handle_log_to_file() {
    static std::deque<RawMessage_s> write_buffer; // The buffer to be copied over to
    
    while (true) {
        {
            std::unique_lock lock(_input_buffer_mutex);
            _input_buffer_cv.wait(lock, [this](){ return !_input_buffer.empty() || !_logging;});
            if (!_logging && _input_buffer.empty()) {
                break;
            }

            write_buffer = std::move(_input_buffer);
        }

        for (auto &msg : write_buffer) {
            mcap::Message msg_to_log;
            msg_to_log.data = reinterpret_cast<const std::byte *>(msg.serialized_data.data());
            msg_to_log.dataSize = msg.serialized_data.size();

            msg_to_log.logTime = msg.log_time;
            msg_to_log.publishTime = msg.log_time;
    
            auto channel = _name_to_id_map.find(msg.message_name);
            if (channel == _name_to_id_map.end()) {
                // first message on this topic, add a channel for it with its message type's schema
                mcap::Channel new_channel(msg.message_name, "protobuf", _type_to_schema_id_map[msg.type_name]);
                _writer.addChannel(new_channel);
                channel = _name_to_id_map.emplace(msg.message_name, new_channel.id).first;
            }
            msg_to_log.channelId = channel->second;
            auto write_res = _writer.write(msg_to_log);
        }
        write_buffer.clear();
    }
}   

int core::MCAPLogger::log_params(nlohmann::json params) {
    mcap::Timestamp log_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    RawMessage_s msg;
    msg.log_time = log_time;
    msg.serialized_data = params.dump();
    msg.message_name = "drivebrain_configuration";
    {
        std::unique_lock lock(_input_buffer_mutex);
        _input_buffer.push_back(std::move(msg));
        _input_buffer_cv.notify_one();
    }

    return 0;
}

