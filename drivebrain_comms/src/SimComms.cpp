#include <SimComms.hpp>

namespace comms {

/****************************************************************
 * STATIC HELPER METHODS
 ****************************************************************/
static std::shared_ptr<google::protobuf::Message> parse_by_name(const std::string& type_name, const void* data, size_t len) 
{
    std::shared_ptr<google::protobuf::Message> msg;

    const google::protobuf::Descriptor *desc = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(type_name);
    if (!desc) return nullptr; 

    msg.reset(google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc)->New());

    if (!msg) return nullptr; 

    if (!msg->ParseFromArray(data, static_cast<int>(len))) return nullptr;
    return msg;
}

static std::string endpoint(uint16_t port) {
    return "ipc:///tmp/drivebrain_sim_" + std::to_string(port);
}

/****************************************************************
 * STATIC HELPER METHODS
 ****************************************************************/
void SimComms::create() {
    SimComms* expected = nullptr;
    SimComms* local = new SimComms();
    if(!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        // Already initialized, delete local instance
        delete local;
    }
}

SimComms& SimComms::instance() {
    SimComms* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "SimComms has not been initialized");
    return *instance;
}

void SimComms::destroy() {
    SimComms* instance = _s_instance.exchange(nullptr, std::memory_order_acq_rel);
    if (instance) {
        delete instance;
    }
}

bool SimComms::start() {
    // TODO setup the other sockets
    _veh_recv_thread = std::thread(&SimComms::_veh_recv_loop, this);

    return true;
}

bool SimComms::close() {
    if (!_running.exchange(false)) return false; 
    if (_veh_recv_thread.joinable()) _veh_recv_thread.join();
    _veh_data_recv_socket.close();
    return true;
}

bool SimComms::send_message(std::shared_ptr<google::protobuf::Message> message) {
    // TODO send over the veh data send socket
    return true;
}

/****************************************************************
 * PRIVATE CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
SimComms::SimComms() {
    _running = true;
    if (!_setup_recv_socket(_veh_data_recv_socket, 6767)) {
        _running = false;
        throw std::runtime_error("SimComms: failed to bind recv socket");
    }
}

bool SimComms::_setup_recv_socket(zmq::socket_t& s, uint16_t port) {
  try {
    s = zmq::socket_t(_ctx, zmq::socket_type::pull);
    s.set(zmq::sockopt::rcvhwm, 10000);
    s.set(zmq::sockopt::rcvtimeo, 100);
    s.bind(endpoint(port));
  } catch (const zmq::error_t& e) {
    spdlog::error("replay bind failed on port {}: {}", port, e.what());
    _running = false; 
    return false;
  }
  return true;
}

void SimComms::_veh_recv_loop() {
    while (_running) {
        zmq::multipart_t m;
        try {
            if (!m.recv(_veh_data_recv_socket)) continue;  
        } catch (const zmq::error_t& e) {
        if (e.num() == ETERM) break;   
            spdlog::error("SimComms recv error: {}", e.what());
            continue;
        }
        if (m.size() < 2) { spdlog::error("SimComms: short frame, {} parts", m.size()); continue; }
        auto type_name = m.popstr(); 
        auto body = m.pop();

        std::shared_ptr<google::protobuf::Message> msg(parse_by_name(type_name, body.data(), body.size()));

        spdlog::info("{}", type_name);

        if (msg) {
            core::log(msg); 
            core::StateTracker::instance().handle_receive_protobuf_message(msg); 
        }
    }
}

}