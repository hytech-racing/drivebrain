#include <ETHRecvComms.hpp>
#include <spdlog/spdlog.h>

using boost::asio::ip::udp;
using namespace comms;  

template <typename MsgType>
ETHRecvComms<MsgType>::ETHRecvComms(boost::asio::io_context& io_context, uint16_t port, std::function<void(std::shared_ptr<MsgType>)> handler)
  : _socket(io_context, udp::v4()),
    _handler(std::move(handler))
{
  static_assert(!std::is_same_v<MsgType, void>, "MsgType cannot be void, receive requires a message type");

  _socket.bind(udp::endpoint(udp::v4(), port));
  _received_msg = std::make_shared<MsgType>();
  _running = true;
  _start_receive();
  spdlog::info("Ethernet receive-only mode, port {}", port);
}

template <typename ETHMsgType>
ETHRecvComms<ETHMsgType>::~ETHRecvComms() {
  _running = false;
  _socket.close();
  spdlog::warn("Destructed Ethernet receive comms");
}

template <typename MsgType>
void ETHRecvComms<MsgType>::_handle_receive(const boost::system::error_code &error, std::size_t size) {
  if (_running && !error) {
      _received_msg->ParseFromArray(_recv_buffer.data(), size);
      _handler(_received_msg);
      _start_receive();
  }
}

template<typename MsgType>
void ETHRecvComms<MsgType>::_start_receive() {
  using namespace boost::placeholders;
  _socket.async_receive_from(
      boost::asio::buffer(_recv_buffer), _remote_endpoint,
      boost::bind(&ETHRecvComms::_handle_receive, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}
