#pragma once

#include <ouster/client.h> 
#include <ouster/chanfield.h>
#include <ouster/slam_engine.h>
#include <ouster/slam_engine.h>

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

            std::vector<sensor::Sensor> _sensors; 
            sensor::SensorScanSource _source;
            mapping::SlamConfig _slam_config;
            mapping::SlamEngine _slam_engine;
            core::XYZLut _lut;
            std::thread _recv_thread;
            std::atomic<bool> _running{true};

    }

}