#pragma once

#include <boost/asio.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace comms {

  /**
  * @brief TCP client for the Sensoric OMS Race optical speed sensor.
  *
  * The sensor streams packets of 36-word frames at 1 kHz as soon as a client connects.
  * Each frame is logged as a hytech_msgs::OMSData with the identified words decoded and
  * all raw words kept bit-exact. Reconnects on its own if the sensor drops the link.
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
    void _connect();
    void _reconnect_later();
    void _start_receive();
    void _handle_receive(const boost::system::error_code& error, std::size_t size);

    /* Splits the buffered stream into packets, logs the complete frames and keeps any partial tail */
    void _parse_packets();
    void _log_frame(const uint8_t* frame, uint32_t packet_seq);

  private:
    static constexpr std::array<uint8_t, 4> kMagic = {'e', 'S', 0x01, 0x00};
    static constexpr std::size_t kHeaderBytes = 16;
    static constexpr std::size_t kWordsPerFrame = 36;
    static constexpr std::size_t kFrameBytes = kWordsPerFrame * 4;
    static constexpr uint32_t kFoxgloveDecimation = 10;

    boost::asio::ip::tcp::socket _socket;
    boost::asio::steady_timer _reconnect_timer;
    boost::asio::ip::tcp::endpoint _endpoint;
    std::array<uint8_t, 65536> _recv_buffer;
    std::vector<uint8_t> _stream;
    bool _running = false;
    uint32_t _connect_failures = 0;
    uint64_t _frame_count = 0;
    uint64_t _dropped_frames = 0;
    float _last_sample_time_s = -1.0f;
    std::chrono::steady_clock::time_point _last_drop_warning;
  };
}
