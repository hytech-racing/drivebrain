#pragma once

#include <boost/asio.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace comms {

  /**
  * @brief TCP client for the Sensoric OMS Race ground speed sensor
  */
  class OMSComms {
  public:
    OMSComms() = delete;
    ~OMSComms();

    /**
     * @param io_context The reference to the initialized io_context
     * @param host The sensor's IP address
     * @param port The sensor's TCP data port
    */
    OMSComms(boost::asio::io_context& io_context, const std::string& host, uint16_t port);

  private:
    /* Little-endian wire layout of one packet header */
    struct PacketHeader {
      uint8_t magic[4];
      uint32_t seq;
      uint32_t len;          // bytes after this field
      uint16_t type;
      uint16_t frame_size;
    };

    struct MotionFrame {
      float vel_x_kmh;
      float vel_y_kmh;
      float vel_a_kmh;
      float ang_s_deg;
      int32_t dist_a_mm;
      float radius_m;
      float acc_c_m_ss;
    };

    /* Little-endian wire layout of one 1 kHz sample */
    struct OMSFrame {
      float sample_time_s;
      MotionFrame motion_poi;
      float roll_deg;
      float pitch_deg;
      float acc_hor_m_ss[3];
      float rate_hor_deg_s[3];
      MotionFrame motion;
      float acc_m_ss[3];
      float rate_deg_s[3];
      float sp_vel_a_kmh;
      float sp_vel_y_kmh;
      float sp_vel_x_kmh;
      float sp_ang_s_deg;
      float sp_dist_a_mm;
      float temp_sensor_c;
      uint32_t status;
    };

    void _connect();
    void _reconnect_later();
    void _start_receive();
    void _handle_receive(const boost::system::error_code& error, std::size_t size);

    /* Logs every complete packet in the stream and keeps any partial tail. Returns false on a malformed header */
    bool _parse_packets();
    void _log_frame(const OMSFrame& frame);

  private:
    boost::asio::ip::tcp::socket _socket;
    boost::asio::steady_timer _reconnect_timer;
    boost::asio::ip::tcp::endpoint _endpoint;
    std::array<uint8_t, 65536> _recv_buffer;
    std::vector<uint8_t> _stream;
    bool _running = false;
    uint32_t _connect_failures = 0;
  };
}
