#define MCAP_IMPLEMENTATION
#include <atomic>
#include <mutex>
#include <fstream>

#include <MCAPLogger.hpp>

#include <MatlabModelProtoRegHelper.hpp>

/****************************************************************
 * HELPER METHODS
 ****************************************************************/
static std::string get_logfile_name(const std::string& dir_path) {
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
    return std::make_tuple(_log_name, _logging);
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
    std::string mcap_name = get_logfile_name(_base_dir);
    spdlog::info("Attempting to open new MCAP file: {}", mcap_name);
    _log_name = _base_dir + mcap_name;

    const auto res = _writer.open(_log_name, _options);
    if (!res.ok()) {
        spdlog::error("Failed to open {} for writing: {}", _log_name, res.message);
        return -1;
    }

    std::vector<std::string> proto_names = {"hytech_msgs.proto", "hytech.proto", "dv_msgs.proto", "foxglove/PointCloud.proto"};
    proto_names.insert(
        proto_names.end(),
        matlab_model_gen::matlab_model_gend_protos.begin(),
        matlab_model_gen::matlab_model_gend_protos.end());

    auto descriptors = get_pb_descriptors(proto_names);

    for (const auto &file_descriptor : descriptors) {
        for (int i = 0; i < file_descriptor->message_type_count(); i++) {
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
            mcap::Schema schema(message_descriptor->full_name(), "protobuf", serialize_fd_set(message_descriptor));
            _writer.addSchema(schema);
            mcap::Channel channel(message_descriptor->name(), "protobuf", schema.id);
            _writer.addChannel(channel);
            _name_to_id_map[message_descriptor->name()] = channel.id;
        }
    }

    spdlog::info("Successfully added message descriptions to mcap");

    mcap::Schema config_schema("drivebrain_configuration", "jsonschema", _params_schema_json.dump());
    _writer.addSchema(config_schema);
    mcap::Channel config_channel("drivebrain_configuration", "json", config_schema.id);
    _writer.addChannel(config_channel);
    _name_to_id_map["drivebrain_configuration"] = config_channel.id;

    log_params(_initial_params);

    spdlog::info("Successfully added params schema");
    
    return 0;
}

int core::MCAPLogger::close_active_mcap() {
    spdlog::info("Closing mcap");
    _writer.close(); 
    _log_name = "NONE";

    return 0;
}

void core::MCAPLogger::init_logging() {
    _msg_log_thread = std::thread([this]() { _handle_log_to_file(); });
    spdlog::info("Msg log thread spawned");
    _logging = true;
}

void core::MCAPLogger::stop_logging() {
    {
        std::unique_lock lock(_input_buffer_mutex);
        _running = false;
        _logging = false;
    }

    _input_buffer_cv.notify_one(); 
    if (_msg_log_thread.joinable()) {
        _msg_log_thread.join();
    }
}

int core::MCAPLogger::log_msg(core::MsgType message) {
    mcap::Timestamp log_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    RawMessage_s new_message; 
    new_message.log_time = log_time;
    new_message.serialized_data = message->SerializeAsString(); 
    new_message.message_name = message->GetDescriptor()->name();
    {
        std::unique_lock lock(_input_buffer_mutex);
        _input_buffer.push_back(new_message); 
        _input_buffer_cv.notify_one(); 
   }

    return 0;
}

/****************************************************************
 * PRIVATE CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
core::MCAPLogger::MCAPLogger(const std::string &base_dir, const mcap::McapWriterOptions &options, const std::string &params_file) : _options(options), _base_dir(base_dir) {
    std::fstream raw_param_file(params_file);
    nlohmann::json params_config = nlohmann::json::parse(raw_param_file);
    _initial_params = params_config; // Used in open_new_mcap to log initial params
    _params_schema_json = generate_json_schema(params_config);
}

void core::MCAPLogger::_handle_log_to_file() {
    static std::deque<RawMessage_s> write_buffer; // The buffer to be copied over to
    
    while (true) {
        {
            std::unique_lock lock(_input_buffer_mutex);
            _input_buffer_cv.wait(lock, [this](){ return !_input_buffer.empty() || !_running;});
            if (!_running && _input_buffer.empty()) {
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

            auto id_it = _name_to_id_map.find(msg.message_name);
            if (id_it == _name_to_id_map.end()) {
                continue; // no channel registered for this message type, skip
            }
            msg_to_log.channelId = id_it->second;
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

