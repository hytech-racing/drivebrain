// /**
//  * Script for testing ethernet driver functionality.
//  * Spawns a mock thread sending protobuf messages over "ethernet" using the ETHSend class.
// */

// #include "ETHSendComms.hpp"
// #include "ETHRecvComms.hpp"
// #include <boost/asio/io_context.hpp>
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
//         std::cout << "halting\n";
//         running = false;
//     }
// }

// int main(int argc, char* argv[]) {
//     // Singleton Creation
//     core::FoxgloveServer::create(argv[1]);
//     core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), argv[1]);
//     core::MCAPLogger::instance().open_new_mcap("test_1.mcap");
//     core::MCAPLogger::instance().init_logging();

//     std::signal(SIGINT, sig_handler);
    
//     boost::asio::io_context io_context;
//     comms::ETHSendComms eth_sender{io_context, 2222, "127.0.0.1"};
//     comms::ETHRecvComms<hytech_msgs::ACUAllData> eth_recver{io_context, 2222};

//     std::thread io_thread([&]() {
//         io_context.run();
//     });

//     std::thread t1([&]() {
//         while(running) {
//             auto msg = std::make_shared<hytech_msgs::ACUAllData>();
//             msg->set_max_cell_temp_id(6767);
//             std::this_thread::sleep_for((std::chrono::seconds(1)));
//             eth_sender.enqueue_msg_send(msg);
//         }
//     });

//     if(t1.joinable()) t1.join();
//     io_context.stop();
//     if(io_thread.joinable()) io_thread.join();
    
//     core::MCAPLogger::instance().close_active_mcap();
//     core::MCAPLogger::destroy();
//     core::FoxgloveServer::destroy();
// }
