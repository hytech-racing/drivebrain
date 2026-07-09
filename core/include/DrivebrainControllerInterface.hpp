
#pragma once

#include <spdlog/spdlog.h>
#include <FoxgloveServer.hpp>

/**
The purpose of this class is to create an interface that can be used 
to switch between different kinds of mode 4 controllers, start logging to mcaps, 
stop logging to mcaps, and more. Last year this was done via GRPC but this year we're 
just doing everything in foxglove
*/

namespace core {
class DrivebrainControllerInterface {

    public: 

        static void create();

        static DrivebrainControllerInterface& instance(); 

        static void destroy(); 

        ~DrivebrainControllerInterface() {
            spdlog::info("Destructing drivebrain controller interface");
            _s_instance.store(nullptr, std::memory_order_release);
            spdlog::info("Drivebrain controller interface singleton instance released");
        }

        void _handle_parameter_updates(const std::unordered_map<std::string, core::DBParam> &new_params);

        void _request_start_logging();

        void _request_stop_logging();

        void _request_current_logger_status();

        void _request_controller_change(int controller_index);

    private: 

        DrivebrainControllerInterface();

        DrivebrainControllerInterface(const DrivebrainControllerInterface&) = delete;
        DrivebrainControllerInterface& operator=(const DrivebrainControllerInterface&) = delete;
        
        inline static std::atomic<DrivebrainControllerInterface*> _s_instance;
        std::string _response; 

};
}
