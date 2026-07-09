#include "Telemetry.hpp"
#include <CANComms.hpp>
#include <stdexcept>
#include <iostream>

/****************************************************************
 * HELPER METHODS
 ****************************************************************/
static std::string string_to_lowercase(std::string input_string) {
    std::transform(input_string.begin(), input_string.end(), input_string.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    return input_string;
}

static std::shared_ptr<google::protobuf::Message> get_proto_message_from_name(const std::string &name) {
    std::shared_ptr<google::protobuf::Message> proto_message; 
    const google::protobuf::Descriptor *desc = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName("hytech." + string_to_lowercase(name));
    if (!desc) {
        return nullptr;
    }
    proto_message.reset(google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc)->New());
    if (!proto_message) {
        return nullptr;
    }
    return proto_message;
}


/****************************************************************
 * PUBLIC CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
comms::CANComms::CANComms(const std::string &device_name, const std::string &dbc_file_path) : _device_name(device_name) {
    if (_init( dbc_file_path) < 0) {
        throw std::runtime_error("Failed to initialize CAN communications");
    }
}

void comms::CANComms::send_message(std::shared_ptr<google::protobuf::Message> message) {
    can_frame frame{};
    int return_code = _encode_can_frame(message, &frame);
    if (return_code < 0) {
        spdlog::error("[{}] Failed to encode CAN frame", _device_name);
        return;
    }

    ssize_t nbytes = write(_socket, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            uint64_t drops = _tx_drop_count.fetch_add(1, std::memory_order_relaxed) + 1;
            if ((drops & 0xFF) == 0) {  
                spdlog::warn("[{}] CAN TX queue full, total drops: {}", _device_name, drops);
            }
        } else {
            _tx_error_count.fetch_add(1, std::memory_order_relaxed);
            spdlog::error("[{}] CAN write failed: {}", _device_name, strerror(errno));
        }
    } else if (static_cast<size_t>(nbytes) != sizeof(struct can_frame)) {
        spdlog::warn("[{}] Partial CAN write: {} of {} bytes", _device_name, nbytes, sizeof(struct can_frame));
    }
}

/****************************************************************
 * PRIVATE CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
int comms::CANComms::_init( const std::string &dbc_file_path) {
    // Initialize the CAN socket
    spdlog::info("Beginning CAN initialization for device {}", _device_name);
    struct sockaddr_can addr;
    struct ifreq ifr;

    _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (_socket < 0) {
        std::cerr << "Failed to create CAN socket: " << strerror(errno) << std::endl;
        return -1;
    }

    strcpy(ifr.ifr_name, _device_name.c_str());
    if (ioctl(_socket, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get interface index for " << _device_name << ": " << strerror(errno) << std::endl;
        close(_socket);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind CAN socket: " << strerror(errno) << std::endl;
        close(_socket);
        return -1;
    }

    // Make socket non-blocking
    int flags = fcntl(_socket, F_GETFL, 0);
    if (flags < 0 || fcntl(_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::cerr << "Failed to set CAN socket non-blocking: " << strerror(errno) << std::endl;
        close(_socket);
        return -1;
    }

    // Set up DBC parsing
    {
        std::ifstream idbc(dbc_file_path.c_str());
        if (!idbc.is_open()) {
            std::cerr << "Failed to open DBC file: " << dbc_file_path << std::endl;
            close(_socket);
            return -1;
        }
        _net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    }

    if (!_net) {
        std::cerr << "Failed to parse DBC file: " << dbc_file_path << std::endl;
        close(_socket);
        return -1;
    }

    for (const dbcppp::IMessage& msg : _net->Messages()) {
        _messages.insert(std::make_pair(msg.Id(), &msg));
        _names_to_can_id.insert(std::make_pair(string_to_lowercase(msg.Name()), msg.Id()));
    }

    // Spawn reader thread
    _reader_thread = std::thread(&comms::CANComms::_receive_handler, this);

    spdlog::info("Successfully initialized CAN driver {}", _device_name);
    return 0;
}

void comms::CANComms::_receive_handler() {
    int nbytes;

    while(true) {
        nbytes = read(_socket, &_frame, sizeof(can_frame));

        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            std::cout << "Error reading from can socket." << std::endl;
            continue;
        }

        if (nbytes < (ssize_t)sizeof(struct can_frame)) {
            std::cout << "Error reading from can socket. Incomplete CAN frame." << std::endl;
            continue;
        }

        auto dmsg = _decode_can_frame(_frame);
        if(dmsg.has_value()) {
            core::log(dmsg.value());
            core::StateTracker::instance().handle_receive_protobuf_message(dmsg.value());
        }
        else spdlog::error("Did not receive valid protobuf message. Skipping...");
    }
}

std::optional<std::shared_ptr<google::protobuf::Message>> comms::CANComms::_decode_can_frame(struct can_frame &frame) {

    const dbcppp::IMessage* dbc_msg = _messages[frame.can_id];
    if (dbc_msg == nullptr) {
        spdlog::error("DBC message does not exist");
        return std::nullopt; 
    }

    std::shared_ptr<google::protobuf::Message> proto_message = get_proto_message_from_name(dbc_msg->Name());
    if (proto_message == nullptr) {
        spdlog::error("Protobuf message not found");
        return std::nullopt;
    }

    const google::protobuf::Descriptor *descriptor = proto_message->GetDescriptor();
    const google::protobuf::Reflection *reflection = proto_message->GetReflection(); 

    for (const dbcppp::ISignal &sig : dbc_msg->Signals()) {
        const dbcppp::ISignal *mux_sig = dbc_msg->MuxSignal();

        if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue || (mux_sig && mux_sig->Decode(frame.data) == sig.MultiplexerSwitchValue())) {
            dbcppp::ISignal::raw_t raw_value = sig.Decode(frame.data);
            double value = sig.RawToPhys(raw_value);

            const google::protobuf::FieldDescriptor *field = descriptor->FindFieldByName(sig.Name());
            switch (field->type()) {
                case google::protobuf::FieldDescriptor::TYPE_ENUM: 
                    reflection->SetEnumValue(proto_message.get(), field, (int)value);
                    break;
                case google::protobuf::FieldDescriptor::TYPE_FLOAT:
                    reflection->SetFloat(proto_message.get(), field, (float)value);
                    break;
                case google::protobuf::FieldDescriptor::TYPE_BOOL: 
                    reflection->SetBool(proto_message.get(), field, (bool)value);
                    break;
                case google::protobuf::FieldDescriptor::TYPE_INT32: 
                    reflection->SetInt32(proto_message.get(), field, (int32_t)value);
                    break;
                default: 
                    break;
            }
        }
    }

    return proto_message; 

}

int comms::CANComms::_encode_can_frame(std::shared_ptr<google::protobuf::Message> proto_message, can_frame* frame) {

    std::string type_url = proto_message->GetTypeName();
    std::string messageTypeName = string_to_lowercase(type_url.substr(type_url.find_last_of('.') + 1));

    const google::protobuf::Descriptor *descriptor = proto_message->GetDescriptor();
    const google::protobuf::Reflection *reflection = proto_message->GetReflection(); 

    if (_names_to_can_id.find(messageTypeName) == _names_to_can_id.end()) {
        std::cout << "Could not find message in known CAN description. Ensure that the message is in the DBC file." << std::endl;
        return -EINVAL;
    }
        
    uint64_t id = _names_to_can_id[messageTypeName]; 
    auto dbc_message = _messages[id]->Clone();   
    frame->can_id = id;
    frame->len = dbc_message->MessageSize();

    for (const auto &sig : dbc_message->Signals()) {
        const google::protobuf::FieldDescriptor *field = descriptor->FindFieldByName(sig.Name());

        switch(field->cpp_type()) {
            case google::protobuf::FieldDescriptor::CPPTYPE_INT32: {
                int32_t value = reflection->GetInt32(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_INT64: {
                int64_t value = reflection->GetInt64(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_UINT32: {
                uint32_t value = reflection->GetUInt32(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_UINT64: {
                uint64_t value = reflection->GetUInt64(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE: {
                double value = reflection->GetDouble(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT: {
                double value = reflection->GetFloat(*proto_message, field);
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_BOOL: {
                bool value = reflection->GetBool(*proto_message, field);
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } default: 
                break;       
        }        
    }

    return 0;

}
