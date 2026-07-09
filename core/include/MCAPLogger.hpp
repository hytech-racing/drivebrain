#pragma once 

#include <deque> 
#include <thread> 
#include <mutex> 
#include <condition_variable>
#include <atomic>
#include <cassert>
#include <queue> 
#include <chrono>
#include <mcap/writer.hpp>
#include <google/protobuf/descriptor.pb.h>
#include <foxglove/websocket/base64.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace core {

    struct RawMessage_s {
        std::string serialized_data;
        std::string message_name;
        mcap::Timestamp log_time;
    };

    using MsgType = std::shared_ptr<google::protobuf::Message>;

    class MCAPLogger {
        public: 

            /**
             * Constructor for initializing a new MCAPLogger singleton instance 
             * 
             * @param base_dir the directory in which the log file should be created
             * @param options options to create the mcap with 
             */
            static void create(const std::string &base_dir, const mcap::McapWriterOptions &options, const std::string &params_file);

            /**
             * Fetches MCAPLogger singleton instance
             *
             * @retun MCAPLogger instance
             */
            static MCAPLogger& instance();

            /**
             * Destroys the MCAPLogger singleton instance, stopping logging and freeing resources
             */
            static void destroy();
            
            /**
             * Destructs MCAPLogger instance, frees singleton instance and joins all running log threads
             */
            ~MCAPLogger() {
              spdlog::info("Destructing mcap logger");
              _logging = false;
              _running = false;

              _s_instance.store(nullptr, std::memory_order_release);
              spdlog::info("Msg logger singleton instance released");

              _param_cv.notify_all();
              _input_buffer_cv.notify_all();
              if(_param_log_thread.joinable()) _param_log_thread.join();
              if(_msg_log_thread.joinable()) _msg_log_thread.join();
            }
            
            /**
             * Opens a new mcap file by adding options and all protobuf schema
             * 
             * @return 0 on success, negative err code on failure
             */
            int open_new_mcap();

            /**
             * Closes the current mcap file. Only runs if there is a file open. 
             * @return 0 on success, negative err code on failure
             */
            int close_active_mcap();

            /**
             * Adds params schema to allow for parameter logging
             * @return 0 on success, negative err code on failure
             */
            int add_params_schema();

            /**
             * Fetches the current logger status
             * @return A tuple containing the name of the file being logged to, and whether or not the logging process is active
            */
            std::tuple<std::string, bool> status();

            /**
             * Kicks off the message logging & param logging thread and begins writing to a log file.
            */
            void init_logging();

            /**
             * Stops logging to the current file.
             * To not be confused with halting, which stops logging and running the MCAPLogger entirely.
            */
            void stop_logging();

            /**
             * Logs a protobuf message to the current MCAP file
             * 
             * @param message the message to be logged
             * @return 0 on success, negative err code on failure
             */
            int log_msg(MsgType message); 

            /**
             * Logs the json params to the current MCAP file
             *
             * @param The parameters to be logged, in JSON format
             * @return 0 on success, negative err code on failure
             */
            int log_params(nlohmann::json params);

        private: 
          
            /* Private constructor to be called by init method */
            MCAPLogger(const std::string &base_dir, const mcap::McapWriterOptions &options, const std::string &params_file);

            /**
             * Spawned by thread, loops until end of program life or error occurs. 
             * Logs all queued messages to a file. 
             */
            void _handle_log_to_file();     

            /* Singleton move semantics */
            MCAPLogger(const MCAPLogger&) = delete;
            MCAPLogger& operator=(const MCAPLogger&) = delete;

            /* Singleton instance */
            inline static std::atomic<MCAPLogger*> _s_instance;
        
            /* Message log management */
            std::deque<RawMessage_s> _input_buffer; 
            std::mutex _input_buffer_mutex;
            std::condition_variable _input_buffer_cv; 
            std::thread _msg_log_thread;
  
            /* Param log management */
            std::mutex _param_mutex;
            std::condition_variable _param_cv;
            std::thread _param_log_thread;

            /* MCAP utilities */
            mcap::McapWriter _writer;   
            mcap::McapWriterOptions _options;
            
            /* State */
            nlohmann::json _params_schema_json;
            nlohmann::json _initial_params;
            std::unordered_map<std::string, uint32_t> _name_to_id_map;
            std::string _base_dir;
            std::string _log_name = "NONE";
            bool _logging = false;
            bool _running = true;

    };

}
