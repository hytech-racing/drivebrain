#pragma once

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/bind.hpp>
#include <boost/system/detail/error_code.hpp>

#include <cstdint>
#include <memory>
#include <google/protobuf/message.h>

#include "StateTracker.hpp"
#include "Telemetry.hpp"

namespace comms {

  /**
  * @brief Ethernet driver class for  receiving Protobuf messages
  * @tparam MsgType The protobuf message type the initialized Ethernet class will receive.
  */
  template<typename MsgType>
  class ETHRecvComms {
  public:
    ETHRecvComms() = delete;
    ~ETHRecvComms();

    /**
     * Constructor for creating a new instance of the Ethernet receiving driver
     *
     * @param io_context The reference to the initialized io_context
     * @param port The port to listen/receive messages from
     * @param handler The handler function to be called on the received message. By default the message is logged to an MCAP and sent to the State Tracker.
    */
    ETHRecvComms(boost::asio::io_context& io_context, uint16_t port,
              std::function<void(std::shared_ptr<MsgType>)> handler = [](std::shared_ptr<MsgType> msg) {
                core::log(msg); core::StateTracker::instance().handle_receive_protobuf_message(msg);
              });
    
    private:
      /* Deserializes the received ethernet message and logs to the MessagerLogger and state estimator */
      void _handle_receive(const boost::system::error_code &error, std::size_t size);

      /* Begins message receiving loop */
      void _start_receive();    

    private:
      std::array<uint8_t, 4096> _recv_buffer;
      boost::asio::ip::udp::socket _socket;
      boost::asio::ip::udp::endpoint _remote_endpoint;
      std::shared_ptr<MsgType> _received_msg;
      bool _running = false;
      std::thread _output_thread;

      std::function<void(std::shared_ptr<MsgType>)> _handler;
  };
}

#include "ETHRecvComms.tpp"
