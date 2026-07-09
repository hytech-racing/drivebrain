#pragma once

#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <StateTracker.hpp> 
#include "hytech.pb.h"
#include "hytech_msgs.pb.h"
#include "Telemetry.hpp"
#include <spdlog/spdlog.h>

namespace comms {

    class SimComms {

        public:


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

            bool _setup_recv_socket(zmq::socket_t& s, uint16_t port);

            void _veh_recv_loop(); 

            zmq::context_t _ctx;
            zmq::socket_t _lidar_socket; 
            zmq::socket_t _camera_socket;
            zmq::socket_t _veh_data_send_socket; 
            zmq::socket_t _veh_data_recv_socket;

            std::thread _veh_recv_thread; 
            std::atomic<bool> _running{false};

    };

}