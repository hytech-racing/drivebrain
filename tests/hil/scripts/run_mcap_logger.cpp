// /**
//  * Script for testing MCAP logging functionality.
//  * Spawns two mock threads sending protobuf messages at different rates and logs to MCAP.
// */

// #include <chrono>
// #include <csignal>
// #include <google/protobuf/message.h>
// #include <iostream>
// #include <atomic> 
// #include <hytech_msgs.pb.h>
// #include <hytech.pb.h>
// #include <FoxgloveServer.hpp>
// #include <MCAPLogger.hpp>
// #include <mcap/writer.hpp>
// #include <memory>
// #include <thread>

// std::atomic<bool> running = true;

// void sig_handler(int signal) {
//     if(signal == SIGINT) {
//         spdlog::info("halting");
//         running = false;
//     }
// }

// void get_param_task(int wait_time, core::MsgType msg) {
//     while(running) {
//         core::MCAPLogger::instance().log_msg(static_cast<core::MsgType>(msg));
//         std::this_thread::sleep_for((std::chrono::milliseconds(wait_time)));
//     }
// }

// int main(int argc, char* argv[]) {

//     core::FoxgloveServer::create(argv[1]);
//     core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), argv[1]);
//     core::MCAPLogger::instance().open_new_mcap("test_1.mcap");
//     core::MCAPLogger::instance().init_logging();

//     std::signal(SIGINT, sig_handler);
    
//     auto vel_msg = std::make_shared<hytech::velocities>();
//     vel_msg->set_velocity_x(1000);
//     vel_msg->set_velocity_y(10000);

//     auto acu_data = std::make_shared<hytech_msgs::ACUAllData>();
//     acu_data->set_max_cell_temp_id(676767);

//     std::thread t1(get_param_task, 20, vel_msg);
//     std::thread t2(get_param_task, 40, acu_data);

//     if(t1.joinable()) t1.join();
//     if(t2.joinable()) t2.join();
//     core::MCAPLogger::instance().close_active_mcap();

//     core::MCAPLogger::destroy();
//     core::FoxgloveServer::destroy();

// }
