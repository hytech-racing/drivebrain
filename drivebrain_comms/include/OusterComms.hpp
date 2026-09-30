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
             * Initializes a new Ouster communications interface with the specified sensor hostname. 
             * 
             * @param sensor_hostname the hostname of the ouster sensor trying to be initialized
             */
            OusterComms(const std::string &sensor_hostname, bool successful);

            ~OusterComms() {
                _running = false;
                spdlog::warn("destructed ouster comms");
            }

            struct deskew_data {
                Eigen::Vector3d gyro_bias =
                Eigen::Vector3d::Zero();

            Eigen::Vector3d accel_bias =
                Eigen::Vector3d::Zero();

            // Current orientation
            Eigen::Matrix3d rotation =
                Eigen::Matrix3d::Identity();

            // Current velocity
            Eigen::Vector3d velocity =
                Eigen::Vector3d::Zero();

            // Current position
            Eigen::Vector3d position =
                Eigen::Vector3d::Zero();

            // Last IMU timestamp
            double last_timestamp = -1.0;
            };

            deskew_data _deskew_data;
            
        
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

            void _deskew(ouster::sdk::core::LidarFrame &frame, int index);

            // Represents a physical ouster lidar in a vector so you can have multiple sensors/lidars
            std::vector<ouster::sdk::sensor::Sensor> _sensors;
            
            // SensorFrameSetSource allows you to receive data in lidar frames instead of udp packets
            std::unique_ptr<ouster::sdk::sensor::SensorFrameSetSource> _source;

            // gets all udp packets -- gives access to imu data
            std::unique_ptr<ouster::sdk::sensor::SensorPacketSource> _packet; 

            ouster::sdk::core::SensorInfo _sensor_info;
            

            // Use the lookup table to convert range measurements into xyz coordinates
            std::vector<ouster::sdk::core::XYZLutT<float>> _lut;

            Eigen::Array<uint16_t, Eigen::Dynamic, 1> imu_status;
            Eigen::Array<uint64_t, Eigen::Dynamic, 1> imu_timestamp;
            Eigen::Array<float, Eigen::Dynamic, 3> imu_accel;
            Eigen::Array<float, Eigen::Dynamic, 3> imu_gyro;

            std::thread _thread;
            std::atomic<bool> _running{true};

    };

}
