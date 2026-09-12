#pragma once

#include <arv.h>
#include <spdlog/spdlog.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <foxglove/CompressedImage.pb.h>
#include "Telemetry.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

namespace comms {

class BlackflyComms {

    public: 

        /**
         * Initializes the Aravis communication interface.
         * 
         */
        BlackflyComms(); 

        /**
         * Cleans up the Aravis communication interface.
         * 
         */
        ~BlackflyComms(); 

        /**
         * Opens the camera for image acquisition and starts the logging thread
         * 
         * @return true if the camera was successfully opened, false otherwise.
         * 
         */
        bool start(const std::string& id, const std::string& pixel_format, double fps);


    private: 

        void _aravis_receive_loop(); 

        std::thread _veh_recv_thread; 
        std::atomic<bool> _running{false};

        ArvCamera *_camera{nullptr};
        ArvStream *_stream{nullptr};
        GError *_error{nullptr};

        std::thread _blackfly_receive_thread; 

};

} // namespace comms