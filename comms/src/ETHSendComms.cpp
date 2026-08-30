#include <ETHSendComms.hpp>
#include <spdlog/spdlog.h>

using boost::asio::ip::udp;
using namespace comms;

ETHSendComms::ETHSendComms(boost::asio::io_context& io_context, uint16_t port, std::string send_ip)
    : _socket(io_context, udp::v4()),
      _send_endp(udp::endpoint(boost::asio::ip::make_address(send_ip), port))
{

    _running = true;
    _output_thread = std::thread(&ETHSendComms::_handle_send_msg_from_queue, this);
    spdlog::info("Ethernet send-only mode, port {}", port);
}

ETHSendComms::~ETHSendComms() {
    _running = false;
    _send_msg_queue.cv.notify_all();
    _socket.close();
    if(_output_thread.joinable()) {
        _output_thread.join();
    }
    spdlog::warn("Destructed Ethernet send comms");
}


void ETHSendComms::enqueue_msg_send(std::shared_ptr<google::protobuf::Message> send_msg) {
  spdlog::debug("Enqueing msg");
  std::unique_lock lk(_send_msg_queue.mtx);
  _send_msg_queue.deque.push_back(send_msg);
  _send_msg_queue.cv.notify_all();
}

void ETHSendComms::_handle_send_msg_from_queue() {
  std::deque<std::shared_ptr<google::protobuf::Message>> temp_msg_queue;
  while(_running) {
      /* transfer messages to temp queue for duration of lock */
      {
          std::unique_lock lk(_send_msg_queue.mtx);
          _send_msg_queue.cv.wait(lk, [this]() { return !_send_msg_queue.deque.empty() || !_running; });

          if(!_running) {
              break;
          }

          _send_msg_queue.deque.swap(temp_msg_queue);
      }

      for (const auto &msg : temp_msg_queue) {
          _send_message(msg);
      }

      temp_msg_queue.clear();
  }
}

void ETHSendComms::_send_message(std::shared_ptr<google::protobuf::Message> msg_out) {
  spdlog::info("sending ethernet message");
  msg_out->SerializeToArray(_send_buffer.data(), msg_out->ByteSizeLong());
  _socket.async_send_to(boost::asio::buffer(_send_buffer, msg_out->ByteSizeLong()),
                          _send_endp,
                          [](const boost::system::error_code &error, std::size_t bytes_transferred)
  {
      if (error) {
          spdlog::error("async send failed: {}", error.message());
      } else {
          spdlog::debug("async send completed: {} bytes.", bytes_transferred);
      }                        
  });
}
