#include "OMSComms.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

#include <spdlog/spdlog.h>

#include "hytech_msgs.pb.h"
#include "Telemetry.hpp"

using boost::asio::ip::tcp;
using namespace comms;

OMSComms::OMSComms(boost::asio::io_context& io_context, const std::string& host, uint16_t port)
  : _socket(io_context),
    _reconnect_timer(io_context),
    _endpoint(boost::asio::ip::make_address(host), port)
{
  _running = true;
  _connect();
  spdlog::info("OMS comms connecting to {}:{}", host, port);
}

OMSComms::~OMSComms() {
  _running = false;
  _reconnect_timer.cancel();
  boost::system::error_code ec;
  _socket.close(ec);
  spdlog::warn("Destructed OMS comms");
}

void OMSComms::_connect() {
  _socket.async_connect(_endpoint, [this](const boost::system::error_code& error) {
    if (!_running) return;
    if (error) {
      // the sensor is off whenever the car is; only nag about it now and then
      if (++_connect_failures % 30 == 1) {
        spdlog::warn("OMS connect to {}:{} failed: {}", _endpoint.address().to_string(), _endpoint.port(), error.message());
      }
      _reconnect_later();
      return;
    }
    boost::system::error_code ec;
    // the sensor skips samples instead of blocking when the client falls behind
    _socket.set_option(boost::asio::socket_base::receive_buffer_size(4 << 20), ec);
    _connect_failures = 0;
    _stream.clear();
    _last_sample_time_s = -1.0f;
    spdlog::info("OMS connected to {}:{}", _endpoint.address().to_string(), _endpoint.port());
    _start_receive();
  });
}

void OMSComms::_reconnect_later() {
  boost::system::error_code ec;
  _socket.close(ec);
  _reconnect_timer.expires_after(std::chrono::seconds(1));
  _reconnect_timer.async_wait([this](const boost::system::error_code& error) {
    if (!error && _running) _connect();
  });
}

void OMSComms::_start_receive() {
  _socket.async_read_some(boost::asio::buffer(_recv_buffer),
                          [this](const boost::system::error_code& error, std::size_t size) {
                            _handle_receive(error, size);
                          });
}

void OMSComms::_handle_receive(const boost::system::error_code& error, std::size_t size) {
  if (!_running) return;
  if (error) {
    spdlog::warn("OMS receive error: {} (reconnecting)", error.message());
    _reconnect_later();
    return;
  }
  _stream.insert(_stream.end(), _recv_buffer.begin(), _recv_buffer.begin() + size);
  _parse_packets();
  _start_receive();
}

/*
 * Packet layout (little-endian):
 *   u8[4] magic "eS\x01\x00" | u32 packet seq | u32 len (bytes after this field)
 *   | u16 type | u16 frame size | frames[(len - 4) / frame size]
 * Frames per packet depend on the sensor's "Direct Send" setting, so it is taken from len.
 */
void OMSComms::_parse_packets() {
  std::size_t pos = 0;
  while (_stream.size() - pos >= kHeaderBytes) {
    auto it = std::search(_stream.begin() + pos, _stream.end(), kMagic.begin(), kMagic.end());
    if (it == _stream.end()) {
      pos = _stream.size() - (kMagic.size() - 1);  // the tail may hold the start of a magic
      break;
    }
    pos = static_cast<std::size_t>(it - _stream.begin());
    if (_stream.size() - pos < kHeaderBytes) break;

    uint32_t seq, len;
    uint16_t frame_size;
    std::memcpy(&seq, &_stream[pos + 4], sizeof(seq));
    std::memcpy(&len, &_stream[pos + 8], sizeof(len));
    std::memcpy(&frame_size, &_stream[pos + 14], sizeof(frame_size));
    if (frame_size != kFrameBytes || len < 4 + kFrameBytes || len > _recv_buffer.size()) {
      pos += kMagic.size();  // not a real header, keep searching
      continue;
    }
    const std::size_t total = 12 + len;
    if (_stream.size() - pos < total) break;

    const std::size_t n_frames = (len - 4) / frame_size;
    for (std::size_t i = 0; i < n_frames; i++) {
      _log_frame(&_stream[pos + kHeaderBytes + i * frame_size], seq);
    }
    pos += total;
  }
  _stream.erase(_stream.begin(), _stream.begin() + pos);
}

void OMSComms::_log_frame(const uint8_t* frame, uint32_t packet_seq) {
  std::array<uint32_t, kWordsPerFrame> words;
  std::memcpy(words.data(), frame, kFrameBytes);
  auto as_float = [&words](std::size_t i) {
    float v;
    std::memcpy(&v, &words[i], sizeof(v));
    return v;
  };
  auto set_xyz = [&as_float](hytech_msgs::xyz_vector* v, std::size_t i) {
    v->set_x(as_float(i));
    v->set_y(as_float(i + 1));
    v->set_z(as_float(i + 2));
  };

  auto msg = std::make_shared<hytech_msgs::OMSData>();
  msg->set_sample_time_s(as_float(0));
  msg->set_roll_deg(as_float(8));
  msg->set_pitch_deg(as_float(9));
  set_xyz(msg->mutable_acc_hor_m_ss(), 10);
  set_xyz(msg->mutable_rate_hor_deg_s(), 13);
  set_xyz(msg->mutable_acc_m_ss(), 23);
  set_xyz(msg->mutable_rate_deg_s(), 26);
  msg->set_temp_sensor_c(as_float(34));
  msg->set_status_word(words[35]);
  msg->set_packet_seq(packet_seq);
  msg->mutable_raw_words()->Reserve(kWordsPerFrame);
  for (uint32_t w : words) {
    msg->add_raw_words(w);
  }

  // SampleTime steps 1 ms per frame; a bigger step means the sensor skipped samples
  const float t = as_float(0);
  if (_last_sample_time_s >= 0.0f) {
    const long gap = std::lround((t - _last_sample_time_s) * 1000.0f) - 1;
    if (gap > 0) {
      _dropped_frames += gap;
      const auto now = std::chrono::steady_clock::now();
      if (now - _last_drop_warning > std::chrono::seconds(5)) {
        spdlog::warn("OMS skipped {} frames ({} total)", gap, _dropped_frames);
        _last_drop_warning = now;
      }
    }
  }
  _last_sample_time_s = t;

  core::log_mcap_only(msg);
  if (++_frame_count % kFoxgloveDecimation == 0) {
    core::log_foxglove_only(msg);
  }
}
