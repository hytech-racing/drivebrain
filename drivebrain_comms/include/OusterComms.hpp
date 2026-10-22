#pragma once

//#include "StateTracker.hpp"
#include "Telemetry.hpp"
#include "hytech_msgs.pb.h"
#include "dv_msgs.pb.h"

#include <ouster/core/sensor_info.h>
#include <ouster/sensor/client.h> 
#include <ouster/core/chanfield.h>
#include <ouster/core/xyzlut.h>
#include <ouster/sensor/sensor_frame_set_source.h>
#include <ouster/sensor/sensor_http.h>
#include <ouster/sensor/sensor_packet_source.h>

namespace comms {

    class OusterComms {

        public: 

            /**
             * Initializes a new Ouster communications interface. 
             */
            OusterComms();

            ~OusterComms() {
                _running = false;
                if (_thread.joinable()) {
                    _thread.join();
                }
                spdlog::warn("destructed ouster comms");
            }
            
        
        private: 

            /**
             * Initializes the OS1 interface, invoked by constructor. 
             * 
             * @return 0 if successful, negative error code on failure
             */
            int _init();
            
            /**
             * Runs the main loop for receiving and processing Ouster data
             */
            void _loop();

            // Represents a physical ouster lidar in a vector so you can have multiple sensors/lidars
            std::vector<ouster::sdk::sensor::Sensor> _sensors;
            
            // SensorFrameSetSource allows you to receive data in lidar frames instead of udp packets
            std::unique_ptr<ouster::sdk::sensor::SensorFrameSetSource> _source;

            // gets all udp packets -- gives access to imu data
            std::unique_ptr<ouster::sdk::sensor::SensorPacketSource> _packet; 

            // Use the lookup table to convert range measurements into xyz coordinates
            std::vector<ouster::sdk::core::XYZLutT<float>> _lut;

            std::thread _thread;
            std::atomic<bool> _running{true};

    };

}
