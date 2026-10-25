#include <SimComms.hpp>
#include <foxglove/PointCloud.pb.h>
#include <foxglove/FrameTransform.pb.h>
#include <foxglove/ImageAnnotations.pb.h>
#include <foxglove/RawImage.pb.h>
#include <algorithm>
#include <cstdio>
#include <memory>
#include "StateTracker.hpp"
#include "dv_msgs.pb.h"
#include <cstdlib>

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
    const char* host = std::getenv("SIM_HOST");
    if (!host) host = "127.0.0.1";
    return "tcp://" + std::string(host) + ":" + std::to_string(port);
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
    _veh_recv_thread = std::thread(&SimComms::_veh_recv_loop, this);
    _lidar_recv_thread = std::thread(&SimComms::_lidar_recv_loop, this);
    return true;
}

bool SimComms::close() {
    if (!_running.exchange(false)) return false;
    if (_veh_recv_thread.joinable()) _veh_recv_thread.join();
    if (_lidar_recv_thread.joinable()) _lidar_recv_thread.join();
    _veh_data_recv_socket.close();
    _lidar_socket.close();
    return true;
}

bool SimComms::send_message(std::shared_ptr<google::protobuf::Message> message) {
    zmq::multipart_t m;
    m.addstr(message->GetDescriptor()->full_name());
    std::string body = message->SerializeAsString();
    m.addmem(body.data(), body.size());

    try {
        std::unique_lock lk(_send_mutex);
        return m.send(_veh_data_send_socket, static_cast<int>(zmq::send_flags::dontwait));
    } catch (const zmq::error_t& e) {
        if (e.num() != EAGAIN && e.num() != ETERM) {
            spdlog::error("SimComms send error: {}", e.what());
        }
        return false;
    }
}

/****************************************************************
 * PRIVATE CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
SimComms::SimComms() {
    _running = true;
    if (!_setup_recv_socket(_veh_data_recv_socket, _recv_socket_port)) {
        _running = false;
        throw std::runtime_error("SimComms: failed to bind recv socket");
    }
    if (!_setup_send_socket(_veh_data_send_socket, _send_socket_port)) {
        _running = false;
        throw std::runtime_error("SimComms: failed to connect send socket");
    }
    if (!_setup_lidar_socket(_lidar_socket, _lidar_socket_port)) {
        _running = false;
        throw std::runtime_error("SimComms: failed to connect lidar socket");
    }
}

bool SimComms::_setup_recv_socket(zmq::socket_t& s, uint16_t port) {
  try {
    s = zmq::socket_t(_ctx, zmq::socket_type::pull);
    s.set(zmq::sockopt::rcvhwm, 10000);
    s.set(zmq::sockopt::rcvtimeo, 100);
    s.connect(endpoint(port));
  } catch (const zmq::error_t& e) {
    spdlog::error("SimComms: connect failed on port {}: {}", port, e.what());
    _running = false;
    return false;
  }
  return true;
}

bool SimComms::_setup_send_socket(zmq::socket_t& s, uint16_t port) {
    try {
        s = zmq::socket_t(_ctx, zmq::socket_type::push);
        s.set(zmq::sockopt::sndhwm, 10);
        s.connect(endpoint(port));
    } catch (const zmq::error_t& e) {
        spdlog::error("SimComms: failed to connect send socket: {}", e.what());
        _running = false;
        return false;
    }
    return true;
}

bool SimComms::_setup_lidar_socket(zmq::socket_t& s, uint16_t port) {
    try {
        s = zmq::socket_t(_ctx, zmq::socket_type::pull);
        s.set(zmq::sockopt::rcvhwm, 2);
        s.set(zmq::sockopt::rcvtimeo, 100);
        s.connect(endpoint(port));
    } catch (const zmq::error_t& e) {
        spdlog::error("SimComms: failed to connect lidar socket: {}", e.what());
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
        // MCAP replay also sends the topic, other sims don't
        const std::string topic = m.empty() ? "" : m.popstr();

        std::shared_ptr<google::protobuf::Message> msg(parse_by_name(type_name, body.data(), body.size()));

        // spdlog::info("{}", type_name);
        if (!msg) {
            spdlog::error("SimComms: unknown message type '{}'", type_name);
            continue;
        }

        /* Sim message parsing */
        auto desc = msg->GetDescriptor();
        if (desc == dv_msgs::Cones::descriptor()) {
            // core::render_cones(std::static_pointer_cast<dv_msgs::Cones>(msg), "ground_truth_cones");
            // Right now for testing purposes, we assume that all cones of the track are visible
            core::StateTracker::instance().set_cone_observations(std::static_pointer_cast<dv_msgs::Cones>(msg));
        } else if (desc == hytech_msgs::pose::descriptor()) {
            core::render_pose(std::static_pointer_cast<hytech_msgs::pose>(msg), "ground_truth_pose");
        } else if (desc == foxglove::FrameTransform::descriptor()) {
            core::log(msg);
        } else if (desc == foxglove::RawImage::descriptor()) {
            auto img = std::static_pointer_cast<foxglove::RawImage>(msg);
            if (img->encoding() == "bayer_rggb8" && img->step() == img->width()) {
                std::reverse(img->mutable_data()->begin(), img->mutable_data()->end());
                img->set_encoding("bayer_bggr8");
            }
            // per-camera topic, otherwise every camera lands on "RawImage"
            core::log(img, img->frame_id() + "/raw");
        } else if (desc == foxglove::ImageAnnotations::descriptor() && topic.ends_with("/detections")) {
            // Recorded YOLO boxes: LINE_LOOP corners in raw (upside-down) full-res pixels, labelled "<color> <score>"
            constexpr float W = 1280, H = 1024;
            const auto& ann = static_cast<const foxglove::ImageAnnotations&>(*msg);
            auto detections = std::make_shared<std::vector<core::ConeDetection>>();
            for (int i = 0; i < ann.points_size() && i < ann.texts_size(); ++i) {
                const auto& p = ann.points(i).points();
                if (p.size() != 4) continue;
                char name[16] = {};
                float score = 0;
                std::sscanf(ann.texts(i).text().c_str(), "%15s %f", name, &score);
                detections->push_back({W - p[2].x(), H - p[2].y(), W - p[0].x(), H - p[0].y(), core::cone_color_from_name(name), score});
            }
            core::StateTracker::instance().set_camera_detections(topic.substr(0, topic.find('/')), detections);
        } else {
            core::log(msg);
            core::StateTracker::instance().handle_receive_protobuf_message(msg);
        }
    }
}

void SimComms::_lidar_recv_loop() {
    while (_running) {
        zmq::message_t frame;
        try {
            auto res = _lidar_socket.recv(frame);
            std::cout << "Received lidar frame of size: " << frame.size() << std::endl;
            if (!res) continue;
        } catch (const zmq::error_t& e) {
            if (e.num() == ETERM) break;
            spdlog::error("SimComms lidar recv error: {}", e.what());
            continue;
        }

        auto pc = std::make_shared<foxglove::PointCloud>();
        if (!pc->ParseFromArray(frame.data(), static_cast<int>(frame.size()))) {
            spdlog::error("SimComms: failed to parse foxglove::PointCloud");
            continue;
        }

        core::StateTracker::instance().handle_receive_protobuf_message(pc);
        core::log(pc);
    }
}

}
