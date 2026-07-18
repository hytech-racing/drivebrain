#pragma once

#include <exception>
#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/parameter.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/parameter.hpp>
#include <boost/signals2/connection.hpp>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>
#include <boost/signals2.hpp>
#include <MCAPLogger.hpp>
#include <string>
#include <atomic>
#include <cassert>
#include <optional>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <spdlog/spdlog.h>

#include "hytech_msgs.pb.h"

namespace core {

    using DBParam = std::variant<bool, int, float, double, std::string, std::monostate>; 

    class FoxgloveServer {
        public: 
            /**
             *  Initialzes a new Foxglove server singleton instance 
             * 
             * @param parameters_file the json containing parameters for putting on foxglove
             */
            static void create(const std::string &parameters_file);

            /**
             * Fetches Foxglove server singleton instance
             *
             * @retun FoxgloveServer instance
             */
            static FoxgloveServer& instance();

            /**
             * Destroys the Foxglove server singleton instance, stopping the server and freeing resources
             */
            static void destroy();
            
            /**
             * Destructs the foxglove server instance by stopping the broadcast
             * thread (before any producer can enqueue into a dead server) and
             * then stopping the websocket server.
             */
            ~FoxgloveServer() {
                {
                    std::unique_lock lock(_send_mutex);
                    _send_running = false;
                }
                _send_cv.notify_all();
                if (_send_thread.joinable()) _send_thread.join();

                _server->stop();
                spdlog::info("Destructed and stopped foxglove websocket server");
            }

            /**
             * Queues a protobuf for broadcast to Foxglove
             *
             * @param msg the message to be sent
             */
            void send_live_telem_msg(std::shared_ptr<const google::protobuf::Message> msg);

            /**
             * Registers a callback function to be run whenever a parameter is updated in Foxglove.
             * @param The function to be reigstered, which takes the form of (const string, param) -> void
             * @return The newly created boost connection
            */
            boost::signals2::connection register_param_callback(std::function<void(const std::unordered_map<std::string, DBParam>&)> callback);

            /**
             * Returns all the current parameter values in a JSON format. Mostly used to log the current parameters in an MCAP file
            */
            nlohmann::json get_all_params();

            /**
             * Thread-safe method to get a foxglove param
             * 
             * @param param_name name of the parameter the user wants to get
             * @return the parameter value
             */
            template <typename param_type> 
            std::optional<param_type> get_param(std::string param_name) {
                std::unique_lock lock(_parameter_mutex); 
                std::transform(param_name.begin(), param_name.end(), param_name.begin(),
                    [](unsigned char c){ return static_cast<unsigned char>(std::tolower(c)); });

                if (_foxglove_params_map.find(param_name) == _foxglove_params_map.end()) {
                    spdlog::warn("The following parameter was not found in the params json: " + param_name);
                    return std::nullopt;
                }

                if (auto pval = std::get_if<param_type>(&_foxglove_params_map[param_name])) {
                    return *pval;
                } else {
                    spdlog::warn("Param has incorrect type maybe: " + param_name);
                    return std::nullopt;
                }
            }

        private:
            FoxgloveServer(std::string parameters_file);

            /* Registers JSON params on init. Recursively called to support multi-level JSON */
            void _init_params(const nlohmann::json &json_obj, const std::string &prefix);

            /* Drains the broadcast queue and sends to clients. */
            void _broadcast_loop();

            /* Singleton move semantics */
            FoxgloveServer(const FoxgloveServer&) = delete;
            FoxgloveServer& operator=(const FoxgloveServer&) = delete;

            /* Singleton instance */
            inline static std::atomic<FoxgloveServer*> _s_instance;

            /* Boost signal for parameter updates */
            boost::signals2::signal<void(const std::unordered_map<std::string, DBParam>&)> _param_update_signal;

            /* Method to handle converting foxglove params to avoid type conflicts */
            std::optional<foxglove::Parameter> _convert_foxglove_parameter(DBParam current_param, foxglove::Parameter incoming_param);

            /* Method to go from db param to foxglove param */
            std::optional<foxglove::Parameter> _get_foxglove_parameter(std::string set_name, DBParam db_param); 

            /* Method to go from foxglove param to db param */
            DBParam _get_db_param(foxglove::Parameter foxglove_parameter);
            
            std::unordered_map<std::string, DBParam> _foxglove_params_map; 
            std::unordered_map<std::string, uint32_t> _name_to_id_map;
            
            std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> _server;
            foxglove::ServerOptions _server_options;

            std::mutex _parameter_mutex;

            /* Async broadcast pipeline — decouples producers from the network. */
            struct OutgoingMsg {
                uint32_t channel_id;
                uint64_t timestamp;
                std::string data;
            };
            static constexpr size_t MAX_SEND_QUEUE = 256; // drop-oldest beyond this
            std::deque<OutgoingMsg> _send_queue;
            std::mutex _send_mutex;
            std::condition_variable _send_cv;
            std::thread _send_thread;
            bool _send_running = false;
    };
}

