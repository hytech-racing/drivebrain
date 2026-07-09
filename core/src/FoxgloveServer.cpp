#include <FoxgloveServer.hpp>
#include <foxglove/websocket/parameter.hpp>
#include <nlohmann/detail/value_t.hpp>
#include <fstream>
#include <spdlog/spdlog.h>

#include <MatlabModelProtoRegHelper.hpp>

static std::string to_lowercase(std::string s) {
   std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return static_cast<unsigned char>(std::tolower(c)); });
   return s;
}

static uint64_t nanosecondsSinceEpoch() {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count());
}

static std::vector<const google::protobuf::FileDescriptor *> get_pb_descriptors(const std::vector<std::string> &filenames) {
    std::vector<const google::protobuf::FileDescriptor *> descriptors;

    for (const auto &name : filenames) {
        const google::protobuf::FileDescriptor *file_descriptor = google::protobuf::DescriptorPool::generated_pool()->FindFileByName(name);

        if (!file_descriptor) {
            spdlog::error("File descriptor not found {}", name);
        }
        else {
            descriptors.push_back(file_descriptor);
        }
    }
    return descriptors;
}

static std::string SerializeFdSet(const google::protobuf::Descriptor *toplevelDescriptor) {
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

void core::FoxgloveServer::create(const std::string &parameters_file) {
    FoxgloveServer* expected = nullptr;
    FoxgloveServer* local = new FoxgloveServer(parameters_file);
    if(!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        // Already initialized, delete local instance
        delete local;
    }
}

core::FoxgloveServer& core::FoxgloveServer::instance() {
    FoxgloveServer* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "Foxglove server has not been initialized");
    return *instance;
}

void core::FoxgloveServer::destroy() {
    FoxgloveServer* instance = _s_instance.exchange(nullptr, std::memory_order_acq_rel);
    if (instance) {
        delete instance;
    }
}

void core::FoxgloveServer::_init_params(const nlohmann::json &json_obj, const std::string &prefix) {
    for (auto &[key, value] : json_obj.items()) {
        std::string param_name = prefix.empty() ? to_lowercase(key) : prefix + "/" + to_lowercase(key);
        
        if (value.type() == nlohmann::detail::value_t::object) {
            _init_params(value, param_name);
        } else {
            core::DBParam param_value;
            if (value.type() == nlohmann::detail::value_t::boolean) {
                param_value = value.get<bool>();
            } else if (value.type() == nlohmann::detail::value_t::number_float) {
                param_value = value.get<float>();
            } else if (value.type() == nlohmann::detail::value_t::number_integer || value.type() == nlohmann::detail::value_t::number_unsigned) {
                param_value = value.get<int>();
            } else if (value.type() == nlohmann::detail::value_t::string) {
                param_value = value.get<std::string>();
            } else {
                spdlog::error("Invalid parameter config type: {} for key: {}", value.type_name(), param_name);
                continue;
            }

            if (_foxglove_params_map.find(param_name) != _foxglove_params_map.end()) {
                spdlog::warn("Duplicate parameter detected: {}", param_name);
            } else {
                _foxglove_params_map[param_name] = param_value;
                spdlog::info("Added parameter: {}", param_name);
            }
        }
    }
}

core::FoxgloveServer::FoxgloveServer(std::string file_name) {

    // Read params data
    std::fstream params_file(file_name);
    nlohmann::json init_param_data = nlohmann::json::parse(params_file); 

    _init_params(init_param_data, "");

    // Instantiate handlers and create foxglove server 
    const auto logHandler = [](foxglove::WebSocketLogLevel, char const *msg) {
        spdlog::info("{}", msg);
    };

    _server_options.capabilities.push_back("parameters");
    _server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>("HTX_Foxglove", logHandler, _server_options);

    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;

    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        spdlog::info("Client {} subscribed to {}", clientStr, chanId);
    };

    hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        spdlog::info("Client {} unsubscribed from {}", clientStr, chanId);
    };

    hdlrs.parameterChangeHandler = [&](const std::vector<foxglove::Parameter> &params, const std::optional<std::string> &request_id, foxglove::ConnHandle clientHandle) 
    {
        std::unordered_map<std::string, DBParam> param_copy;
        {
            std::unique_lock lock(_parameter_mutex);
            for (const auto &incoming_param : params) {
                if (_foxglove_params_map.find(incoming_param.getName()) == _foxglove_params_map.end()) {
                    spdlog::warn("Couldn't find updated param in params map. Please get good.");
                    continue;
                }
                core::DBParam current_param = _foxglove_params_map[incoming_param.getName()];
                std::optional<foxglove::Parameter> converted_param = _convert_foxglove_parameter(current_param, incoming_param);

                spdlog::info("trying to change param {}", incoming_param.getName());
                if (converted_param) {
                    auto value = _get_db_param(converted_param.value());
                    _foxglove_params_map[incoming_param.getName()] = value;
                }
            }
            param_copy = _foxglove_params_map;
        }
        MCAPLogger::instance().log_params(get_all_params()); /* Needed for showing param updates in the outputted MCAP file */
        _param_update_signal(param_copy);
    };

    hdlrs.parameterRequestHandler = [this](const std::vector<std::string> &param_names, const std::optional<std::string> &request_id,
                                           foxglove::ConnHandle clientHandle) {
        std::vector<foxglove::Parameter> foxglove_params; 
        for (auto &[key, value] : _foxglove_params_map) {
            auto param = _get_foxglove_parameter(key, value);
            foxglove_params.push_back(param.value());
        }
        _server->publishParameterValues(clientHandle, foxglove_params, request_id);
    };

    std::vector<std::string> proto_names = {"hytech_msgs.proto", "hytech.proto", "sim_msgs.proto", "foxglove/PointCloud.proto", "foxglove/FrameTransform.proto"};
    proto_names.insert(
        proto_names.end(),
        matlab_model_gen::matlab_model_gend_protos.begin(),
        matlab_model_gen::matlab_model_gend_protos.end());

    auto descriptors = get_pb_descriptors(proto_names);
    std::vector<foxglove::ChannelWithoutId> channels;

    int running_index = 1;
    for (const auto &file_descriptor : descriptors) {

        for (int i = 0; i < file_descriptor->message_type_count(); ++i) {
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
            foxglove::ChannelWithoutId server_channel;
            server_channel.topic = message_descriptor->name();
            server_channel.encoding = "protobuf";
            server_channel.schemaName = message_descriptor->full_name();
            server_channel.schema = foxglove::base64Encode(SerializeFdSet(message_descriptor));
            _name_to_id_map[server_channel.topic] = running_index;
            channels.push_back(server_channel);
            running_index++;
        }
    }

    auto res_ids = _server->addChannels(channels);

    _server->setHandlers(std::move(hdlrs));
    _server->start("0.0.0.0", 5555);

    _send_running = true;
    _send_thread = std::thread([this]() { _broadcast_loop(); });

    params_file.close();
}

boost::signals2::connection core::FoxgloveServer::register_param_callback(std::function<void (const std::unordered_map<std::string, core::DBParam> &)> callback) {
    return _param_update_signal.connect(callback);
}

nlohmann::json core::FoxgloveServer::get_all_params() {
    std::unique_lock lock(_parameter_mutex);
    nlohmann::json params_json;
    params_json["type"] = "object";
    
    for (const auto& [name, param_value] : _foxglove_params_map) {
        if (std::holds_alternative<bool>(param_value)) {
            params_json[name] = std::get<bool>(param_value);
        } 
        else if (std::holds_alternative<float>(param_value)) {
            params_json[name] = std::get<float>(param_value);
        } 
        else if (std::holds_alternative<int>(param_value)) {
            params_json[name] = std::get<int>(param_value);
        } 
        else if (std::holds_alternative<std::string>(param_value)) {
            params_json[name] = std::get<std::string>(param_value);
        }
    }

    return params_json;
} 

std::optional<foxglove::Parameter> core::FoxgloveServer::_convert_foxglove_parameter(core::DBParam current_param, foxglove::Parameter incoming_param) {
    auto converted_current_param = _get_foxglove_parameter(incoming_param.getName(), current_param); 
    if (!converted_current_param) {
        spdlog::warn("Failed to converter db param --> foxglove param while invokking _convert_foxglove_parameter");
        return std::nullopt; 
    }
    if (converted_current_param.value().getType() == incoming_param.getType()) {
        return incoming_param; 
    } else if (converted_current_param.value().getType() == foxglove::ParameterType::PARAMETER_INTEGER && incoming_param.getType() == foxglove::ParameterType::PARAMETER_DOUBLE) {
        return foxglove::Parameter(converted_current_param.value().getName(), static_cast<int64_t>(incoming_param.getValue().getValue<double>()));
    } else if (converted_current_param.value().getType() == foxglove::ParameterType::PARAMETER_DOUBLE && incoming_param.getType() == foxglove::ParameterType::PARAMETER_INTEGER) {
        return foxglove::Parameter(converted_current_param.value().getName(), static_cast<double>(incoming_param.getValue().getValue<int64_t>()));
    } else {
        spdlog::warn("Invalid parameter type conversion!");
        return std::nullopt;
    }
}

std::optional<foxglove::Parameter> core::FoxgloveServer::_get_foxglove_parameter(std::string set_name, core::DBParam db_param) {
    if (std::holds_alternative<bool>(db_param)) {
        return foxglove::Parameter(set_name, std::get<bool>(db_param));
    } else if (std::holds_alternative<int>(db_param)) {
        return foxglove::Parameter(set_name, std::get<int>(db_param));
    } else if (std::holds_alternative<float>(db_param)) {
        return foxglove::Parameter(set_name, ((double)std::get<float>(db_param)));
    } else if (std::holds_alternative<double>(db_param)) {
        return foxglove::Parameter(set_name, std::get<double>(db_param));
    } else if (std::holds_alternative<std::string>(db_param)) {
        return foxglove::Parameter(set_name, std::get<std::string>(db_param));
    } else {
        spdlog::warn("{} unknown param variant type", set_name);
        return std::nullopt;
    }
}

core::DBParam core::FoxgloveServer::_get_db_param(foxglove::Parameter param) {
    if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_BOOL) {
        return param.getValue().getValue<bool>();
    } else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_INTEGER) {
        return ((int)param.getValue().getValue<int64_t>());
    } else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_DOUBLE) {
        return ((float)param.getValue().getValue<double>());
    } else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_STRING) {
        return param.getValue().getValue<std::string>();
    } else {
        spdlog::warn("unsupported param type");
        return std::monostate();
    }
}

void core::FoxgloveServer::send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg) {
    /* find() is a non-mutating lookup — safe for concurrent callers (main loop, eth,
       sim state/lidar threads). operator[] would insert-on-miss and race a rehash. */
    auto it = _name_to_id_map.find(msg->GetDescriptor()->name());
    if (it == _name_to_id_map.end()) {
        return;
    }

    /* Serialize on the calling thread (CPU only, never blocks), then hand off to the
       sender thread. The actual network send happens in _broadcast_loop so a slow
       client can never apply backpressure to a producer (e.g. the control loop). */
    OutgoingMsg out;
    out.channel_id = it->second;
    out.timestamp = nanosecondsSinceEpoch();
    out.data = msg->SerializeAsString();
    {
        std::unique_lock lock(_send_mutex);
        if (_send_queue.size() >= MAX_SEND_QUEUE) {
            _send_queue.pop_front(); // drop oldest — keep telemetry fresh under load
        }
        _send_queue.push_back(std::move(out));
    }
    _send_cv.notify_one();
}

void core::FoxgloveServer::_broadcast_loop() {
    std::deque<OutgoingMsg> batch;
    while (true) {
        {
            std::unique_lock lock(_send_mutex);
            _send_cv.wait(lock, [this]() { return !_send_queue.empty() || !_send_running; });
            if (!_send_running && _send_queue.empty()) {
                break;
            }
            batch.swap(_send_queue); 
        }
        for (const auto &m : batch) {
            _server->broadcastMessage(m.channel_id, m.timestamp,
                                      reinterpret_cast<const uint8_t *>(m.data.data()), m.data.size());
        }
        batch.clear();
    }
}



