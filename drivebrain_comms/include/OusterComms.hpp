#pragma once

#include "StateTracker.hpp"
#include <ouster/sensor/client.h> 
#include <ouster/core/chanfield.h>
#include <ouster/core/xyzlut.h>
#include <ouster/mapping/slam_engine.h>
#include <ouster/sensor/sensor_frame_set_source.h>


namespace comms {

    class OusterComms {

        public: 

            /**
             * Initializes a new Ouster communications interface with the specified sensor hostname. 
             * 
             * @param sensor_hostname the hostname of the ouster sensor trying to be initialized
             */
            OusterComms(const std::string &sensor_hostname);

            ~OusterComms() {
                // TODO cleanup
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
            ouster::sdk::sensor::SensorFrameSetSource _source;
            
            // SLAM parameters and settings that the ouster SDK's system will use
            ouster::sdk::mapping::SlamConfig _slam_config;

            // Use the lookup table to convert range measurements into xyz coordinates
            std::vector<ouster::sdk::core::XYZLut> _lut;

            std::thread _recv_thread;
            std::atomic<bool> _running{true};

    };

}

/*
main things you need for lidar driver is get data, deskew, and send as a protobuf message. 
can use core::log to visualize the point cloud in foxglove
*/