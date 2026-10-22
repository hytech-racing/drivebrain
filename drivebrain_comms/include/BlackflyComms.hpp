#pragma once

#include <arv.h>
#include <spdlog/spdlog.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <foxglove/CompressedImage.pb.h>
#include <foxglove/RawImage.pb.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "Telemetry.hpp"
#if JETSON_ENABLED
#include "Detector.hpp"
#endif

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
         * Arms a camera for scheduled Action0 triggers and starts its receive
         * thread. Call synchronize() once the whole group has been armed.
         * 
         * @return true if the camera was successfully opened, false otherwise.
         * 
         */
        bool start(const std::string& id, const std::string& name, const std::string& pixel_format, double fps);

        // Call after every camera has been armed by start(). The first camera owns
        // the scheduler; stop it before destroying any other member of the group.
        static bool synchronize(const std::vector<std::unique_ptr<BlackflyComms>>& cameras);
        void stop_synchronization();

    private: 

        void _aravis_receive_loop(); 
        bool _check_error(const char* operation);
        bool _ptp_locked(int64_t& parent);
        uint64_t _camera_time();
        bool _schedule_frame(uint64_t timestamp);
        void _synchronize_loop(std::vector<BlackflyComms*> cameras);

        std::atomic<bool> _running{false};

        ArvCamera *_camera{nullptr};
        ArvStream *_stream{nullptr};
        GError *_error{nullptr};
        std::string _name; // frame id and topic prefix

        std::thread _blackfly_receive_thread; 
        std::thread _sync_thread;
        std::atomic<bool> _sync_running{false};
        GSocket* _action_socket{nullptr};
        uint16_t _action_request{0};
        double _fps{0};

#if JETSON_ENABLED
        std::unique_ptr<Detector> _detector;
#endif      

};

} // namespace comms
