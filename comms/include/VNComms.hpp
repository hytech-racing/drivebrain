#pragma once

#include <StateTracker.hpp>

// protobuf
#include "FoxgloveServer.hpp"
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>
#include "hytech_msgs.pb.h"

// boost
#include <boost/asio.hpp>
#include <boost/array.hpp>

// c++ stl includes
#include <memory>
#include <deque>
#include <variant>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <optional>

#include <unistd.h>
#include <cstring>

#include "libvncxx/vntime.h"
#include "libvncxx/packetfinder.h"
#include "libvncxx/packet.h"
#include <spdlog/spdlog.h>

using namespace vn::xplat;
using namespace vn::protocol::uart;
using SerialPort = boost::asio::serial_port;

namespace comms {
    class VNDriver
    {
        public:
            VNDriver(boost::asio::io_context &io_context, bool &init_successful); 
            ~VNDriver() {
                spdlog::info("destructed %s");
            }
            bool init();
            struct config {
                int baud_rate;
                int freq_divisor;
            };

        private: 
            vn::protocol::uart::PacketFinder _processor;
            boost::array<std::uint8_t, 512> _output_buff;
            boost::array<std::uint8_t, 512> _input_buff;
            SerialPort _serial;
            config _config;
            bool _initial_heading_set = false;
            float _local_declination;

        public: 
            // Public methods
            void log_proto_message(std::shared_ptr<google::protobuf::Message> msg);  
            
        private:
            // Private methods
            static void _handle_recieve(void *userData, vn::protocol::uart::Packet &packet, size_t runningIndexOfPacketStart, TimeStamp ts);
            void _configure_INS();
            void _configure_binary_outputs();
            void _start_recieve();
            void _set_initial_heading(float initial_heading);
            void _try_initialize_heading(float mag_heading, uint8_t ins_mode);

    };
}
