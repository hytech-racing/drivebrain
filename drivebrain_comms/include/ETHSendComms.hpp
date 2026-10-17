#pragma once

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/bind.hpp>
#include <boost/system/detail/error_code.hpp>

#include <cstdint>
#include <memory>
#include <google/protobuf/message.h>

#include "DriverBus.hpp"

namespace comms {
  class ETHSendComms {
  public:
    using deqtype = core::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;

    ETHSendComms() = delete;
    ~ETHSendComms();

    /**
     * Constructor for creating a new instance of the Ethernet driver (full-duplex mode)
     *
     * @param io_context The reference to the initialized io_context
     * @param port The port to listen/receive messages from
     * @param send_ip The IP address to send messages to
    */
    ETHSendComms(boost::asio::io_context& io_context, uint16_t port, std::string send_ip);
    
    /**
     * Method for adding a new message to the send queue
     *
     * @param send_msg The message to send
    */
    void enqueue_msg_send(std::shared_ptr<google::protobuf::Message> send_msg);

  private:
    /* Logs and sends all messages in the message queue */
    void _handle_send_msg_from_queue();

    /* Sends one message from the send buffer */
    void _send_message(std::shared_ptr<google::protobuf::Message> msg_out);

  private:
      boost::asio::ip::udp::endpoint _send_endp;
      boost::asio::ip::udp::socket _socket;
      boost::asio::ip::udp::endpoint _remote_endpoint;
      bool _running = false;
      std::thread _output_thread;
      std::array<uint8_t, 4096> _send_buffer;
      deqtype _send_msg_queue;
    
  };
}
