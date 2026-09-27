#pragma once

//#include "StateTracker.hpp"
#include "Telemetry.hpp"
#include <ouster/sensor/client.h> 
#include <ouster/core/chanfield.h>
#include <ouster/core/xyzlut.h>
#include <ouster/sensor/sensor_frame_set_source.h>
#include <ouster/sensor/sensor_packet_source.h>
#include <ouster/mapping/slam_engine.h>



namespace comms {

    class OusterComms {

        public: 

            /**
             * Initializes a new Ouster communications interface with the specified sensor hostname. 
             * 
             * @param sensor_hostname the hostname of the ouster sensor trying to be initialized
             */
            OusterComms(const std::string &sensor_hostname, bool successful);

            ~OusterComms() {
                _running = false;
                spdlog::warn("destructed ouster comms");
            }
            
        
        private: 

            /**
             * Initializes the OS1 interface, invoked by constructor. 
             * 
             * @return 0 if successful, negative error code on failure
             */
            int _init(const std::string &sensor_hostname);   
            
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
            
            // SLAM parameters and settings that the ouster SDK's system will use
            ouster::sdk::mapping::LIOSlamConfig _slam_config;

            // Use the lookup table to convert range measurements into xyz coordinates
            std::vector<ouster::sdk::core::XYZLutT<float>> _lut;

            std::thread _thread;
            std::atomic<bool> _running{true};

    };

}
