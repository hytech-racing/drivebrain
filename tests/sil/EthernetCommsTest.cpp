// #include "base_msgs.pb.h"
// #include "hytech_msgs.pb.h"
// #include <boost/asio/io_context.hpp>
// #include <cassert>
// #include <chrono>
// #include <gtest/gtest.h>
// #include <EthernetComms.hpp>
// #include <thread>

// class EthernetSendTest : public testing::Test {

// protected:
//   boost::asio::io_context io_context;
//   std::thread io_thread;
//   comms::ETHDriver<> eth_sender{io_context, 1155, "127.0.0.1"};

//   void SetUp() override {
//     io_thread = std::thread([this]() {
//       io_context.run();
//     });             
//   }

//   void TearDown() override {
//     io_context.stop();
//     if (io_thread.joinable()) {
//       io_thread.join();
//     }
//   }

// };

// class EthernetDuplexTest : public EthernetSendTest {
// protected:
//   comms::ETHDriver<hytech_msgs::VCRStatus> eth_transceiver{io_context, 1155};
// };

// TEST_F(EthernetSendTest, SendSingleMsg) {
//   auto msg = std::make_shared<hytech_msgs::VCRStatus>();
//   msg->set_drivebrain_controller_timing_failure(true);

//   ASSERT_NO_THROW(eth_sender.enqueue_msg_send(msg));
// }

// TEST_F(EthernetSendTest, SendMultipleMsgs) {
//   auto msg = std::make_shared<hytech_msgs::ACUAllData>();
//   for (int i = 0; i < 10; i++) {
//         msg->set_cell_voltages(0, i);
//         eth_sender.enqueue_msg_send(msg);
//   }
  
//   SUCCEED();
// }

// TEST_F(EthernetDuplexTest, SendRecvMsgs) {
  
// }


