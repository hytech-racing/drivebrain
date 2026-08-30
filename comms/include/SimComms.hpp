#pragma once

#include <cstdint>
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <StateTracker.hpp> 
#include <spdlog/spdlog.h>
#include "hytech.pb.h"
#include "hytech_msgs.pb.h"
#include "Telemetry.hpp"

namespace comms {

class SimComms {
    public:
        /**
         * Starts a new instance of SimComms.
        */
        bool start();

        bool close();
        
        bool send_message(std::shared_ptr<google::protobuf::Message> message);

        static void create();

        static SimComms& instance();

        static void destroy();

    private: 
        SimComms();

        SimComms(const SimComms&) = delete;
        SimComms& operator=(const SimComms&) = delete;
        inline static std::atomic<SimComms*> _s_instance;

        /**
         * Initializes the ZMQ SimComms receive socket
         * @param s The ZMQ socket to initialize
         * @param port The endpoint the ZMQ socket should be listening on
         * @return Whether the setup suceeded
        */
        bool _setup_recv_socket(zmq::socket_t& s, uint16_t port);

        void _veh_recv_loop();
        void _lidar_recv_loop();

        static constexpr char _endpoint_prefix[] = "ipc:///tmp/drivebrain_sim_";
        static constexpr uint16_t _recv_socket_port = 6767;
        static constexpr uint16_t _send_socket_port = 5940;
        static constexpr uint16_t _lidar_socket_port = 1155;

        zmq::context_t _ctx;
        zmq::socket_t _lidar_socket;
        zmq::socket_t _veh_data_send_socket;
        zmq::socket_t _veh_data_recv_socket;

        std::mutex _send_mutex;

        std::thread _veh_recv_thread;
        std::thread _lidar_recv_thread;
        std::atomic<bool> _running{false};
};
}
