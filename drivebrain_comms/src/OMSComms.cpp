#include "OMSComms.hpp"

#include <cstddef>
#include <cstring>
#include <memory>

#include <spdlog/spdlog.h>

#include "hytech_msgs.pb.h"
#include "Telemetry.hpp"

using boost::asio::ip::tcp;
using namespace comms;

namespace {
  constexpr uint8_t packet_magic[4] = {'e', 'S', 0x01, 0x00};
}

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
  if (!_parse_packets()) {
    spdlog::error("OMS received a malformed packet header (reconnecting)");
    _reconnect_later();
    return;
  }
  _start_receive();
}

/* Packet: header | frames[(len - 4) / frame_size] */
bool OMSComms::_parse_packets() {
  static_assert(sizeof(PacketHeader) == 16);
  static_assert(sizeof(OMSFrame) == 36 * 4);

  std::size_t pos = 0;
  while (_stream.size() - pos >= sizeof(PacketHeader)) {
    PacketHeader header;
    std::memcpy(&header, &_stream[pos], sizeof(header));
    if (std::memcmp(header.magic, packet_magic, sizeof(packet_magic)) != 0 ||
        header.frame_size != sizeof(OMSFrame) ||
        header.len < 4 + sizeof(OMSFrame) ||
        header.len > _recv_buffer.size()) {
      return false;
    }
    const std::size_t packet_size = offsetof(PacketHeader, type) + header.len;
    if (_stream.size() - pos < packet_size) break;

    const std::size_t n_frames = (header.len - 4) / sizeof(OMSFrame);
    for (std::size_t i = 0; i < n_frames; i++) {
      OMSFrame frame;
      std::memcpy(&frame, &_stream[pos + sizeof(PacketHeader) + i * sizeof(OMSFrame)], sizeof(frame));
      _log_frame(frame);
    }
    pos += packet_size;
  }
  _stream.erase(_stream.begin(), _stream.begin() + pos);
  return true;
}

void OMSComms::_log_frame(const OMSFrame& frame) {
  auto set_xyz = [](hytech_msgs::xyz_vector* v, const float (&xyz)[3]) {
    v->set_x(xyz[0]);
    v->set_y(xyz[1]);
    v->set_z(xyz[2]);
  };
  auto set_motion = [](hytech_msgs::OMSMotion* m, const MotionFrame& motion) {
    m->set_vel_x_kmh(motion.vel_x_kmh);
    m->set_vel_y_kmh(motion.vel_y_kmh);
    m->set_vel_a_kmh(motion.vel_a_kmh);
    m->set_ang_s_deg(motion.ang_s_deg);
    m->set_dist_a_mm(motion.dist_a_mm);
    m->set_radius_m(motion.radius_m);
    m->set_acc_c_m_ss(motion.acc_c_m_ss);
  };

  auto msg = std::make_shared<hytech_msgs::OMSData>();
  msg->set_sample_time_s(frame.sample_time_s);
  set_motion(msg->mutable_motion_poi(), frame.motion_poi);
  msg->set_roll_deg(frame.roll_deg);
  msg->set_pitch_deg(frame.pitch_deg);
  set_xyz(msg->mutable_acc_hor_m_ss(), frame.acc_hor_m_ss);
  set_xyz(msg->mutable_rate_hor_deg_s(), frame.rate_hor_deg_s);
  set_motion(msg->mutable_motion(), frame.motion);
  set_xyz(msg->mutable_acc_m_ss(), frame.acc_m_ss);
  set_xyz(msg->mutable_rate_deg_s(), frame.rate_deg_s);
  auto* sp = msg->mutable_motion_sp();
  sp->set_vel_a_kmh(frame.sp_vel_a_kmh);
  sp->set_vel_y_kmh(frame.sp_vel_y_kmh);
  sp->set_vel_x_kmh(frame.sp_vel_x_kmh);
  sp->set_ang_s_deg(frame.sp_ang_s_deg);
  sp->set_dist_a_mm(frame.sp_dist_a_mm);
  msg->set_temp_sensor_c(frame.temp_sensor_c);

  auto bit = [&frame](unsigned b) { return ((frame.status >> b) & 1u) != 0; };
  auto* st = msg->mutable_status();
  st->set_standstill(bit(0));
  st->set_direction_neg(bit(1));
  st->set_rate_adjust_active(bit(2));
  st->set_trigger(bit(4));
  st->set_brake_acc_status((frame.status >> 5) & 0x7u);
  st->set_illu_on(bit(8));
  st->set_illu_ok(bit(9));
  st->set_sensor_connected(bit(10));
  st->set_adjustment_active(bit(12));
  st->set_can_ang_adj_failed(bit(13));
  st->set_status_ok(bit(15));
  st->set_info_bits(frame.status >> 16);

  core::log(msg);
}
