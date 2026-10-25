#include "OusterComms.hpp"
#include "Telemetry.hpp"
#include "RangeCompression.hpp"
#include "StateTracker.hpp"
#include "dv_msgs.pb.h"
#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <cstring>
#include <string>

#include <foxglove/PointCloud.pb.h>
#include <foxglove/PackedElementField.pb.h>
#include <foxglove/RawImage.pb.h>
#include <ouster/core/frame_set.h>
#include <ouster/sensor/sensor_packet_source.h>

/****************************************************************
 * PUBLIC METHODS
 ****************************************************************/
comms::OusterComms::OusterComms() {
    // Initialize the Ouster interface
    _thread = std::thread([this]() {
          while (_running && _init() < 0) {
              spdlog::debug("Retrying Ouster initialization");
              std::this_thread::sleep_for(std::chrono::seconds(2));
          }
          if (_running) _loop();
    });
}

/****************************************************************
 * PRIVATE METHODS
 ****************************************************************/
int comms::OusterComms::_init() {

    _sensors.clear();
    _lut.clear();

    // Establish communications with the Ouster
    ouster::sdk::core::SensorConfig config; 
    config.udp_dest = "169.254.0.1"; // TODO validate that this works reliably
    config.timestamp_mode = ouster::sdk::core::TimestampMode::TIME_FROM_PTP_1588; // use jetson's PTP clock
    spdlog::info("Entering Ouster _init: Attempting to open sensor at 169.254.77.2");

    try {
        _sensors.emplace_back("169.254.77.2", config);
        _source = std::make_unique<ouster::sdk::sensor::SensorFrameSetSource>(_sensors); // creating the client that will configure the sensors 
        spdlog::info("Initialized Ouster communications interface");
    }
    catch (...) {
        spdlog::error("failed in ouster init");
        return -1;
    }

    // LUT setup
    if (_source->sensor_info().empty()) {
        spdlog::error("No sensor info available for LUT initialization");
        return -1;
    }

    try {
        const std::string metadata_json = _source->sensor_info()[0]->to_json_string();
        if (metadata_json.empty()) {
            spdlog::error("Ouster sensor metadata is empty");
            return -1;
        }
        _lut.emplace_back(*_source->sensor_info()[0], true);
        auto calibration = std::make_shared<foxglove::RawImage>();
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        const auto secs = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000;
        calibration->mutable_timestamp()->set_seconds(secs);
        calibration->mutable_timestamp()->set_nanos(nanos);
        calibration->set_encoding("ouster_sensor_metadata_json");
        calibration->set_data(metadata_json.data(), metadata_json.size());
        _calibration = calibration;
    } catch (const std::exception& error) {
        spdlog::error("Failed to prepare Ouster calibration: {}", error.what());
        return -1;
    }
    spdlog::info("initialized Ouster LUT");

    return 0;
}


void comms::OusterComms::_loop() {
    while (_running) {

        /* Lidar Data */
        std::pair<int, std::unique_ptr<ouster::sdk::core::LidarFrame>> result = _source->get_frame(0.5); 
        int index = result.first;
        if (!result.second) continue; // check that you actually received a lidar frame before dereferencing it
        auto& frame = *result.second;

        const auto [mcap_name, mcap_logging] = core::MCAPLogger::instance().status();
        if (mcap_logging && mcap_name != _calibration_mcap && core::MCAPLogger::instance().log_msg(_calibration, "ouster/calibration") == 0) {
            _calibration_mcap = mcap_name;
        }
 
        /* IMU Data */
        // auto packet_event = _packet->get_packet(1.0); // timeout is 1 second (wait up to 1 second for a packet)
        // if (packet_event.packet().type() == ouster::sdk::core::PacketType::Imu) {
        //     spdlog::info("recieved an IMU packet");

        //     const auto& imu = packet_event.packet().as<ouster::sdk::core::ImuPacket>();

        //     imu_status = imu.status();
        //     imu_timestamp = imu.timestamp();
        //     imu_accel = imu.accel();
        //     imu_gyro  = imu.gyro();

        //     std::shared_ptr<dv_msgs::LidarIMU> imu_data_out = std::make_shared<dv_msgs::LidarIMU>();

        //     for (int i = 0; i < imu_accel.rows(); i++) {
        //         imu_data_out->set_accelerometer_x(imu_accel(i, 0));
        //         imu_data_out->set_accelerometer_y(imu_accel(i, 1));
        //         imu_data_out->set_accelerometer_z(imu_accel(i, 2));

        //         imu_data_out->set_gyro_x(imu_gyro(i, 2)); 
        //         imu_data_out->set_gyro_y(imu_gyro(i, 2));
        //         imu_data_out->set_gyro_z(imu_gyro(i, 2));
        //     }

        //     core::log(imu_data_out);
        //     spdlog::info("logged lidar imu data");

        // }

        // range image (row-major, columns = scan time) and the xyz lookup table for it
        const auto& lut = _lut[index];
        const Eigen::Index n = lut.direction.rows();
        const auto range = frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
        const Eigen::Index w = range.cols();
        const uint32_t* rng = range.data();
        const float* dir = lut.direction.data();
        const float* ofs = lut.offset.data();

        // per-column sensor timestamps; points are stamped relative to the last valid column (scan end)
        const auto column_ts = frame.timestamp();
        uint64_t scan_end_ts = 0;
        for (Eigen::Index u = 0; u < w; ++u) scan_end_ts = std::max<uint64_t>(scan_end_ts, column_ts(u));

        auto pc = std::make_shared<foxglove::PointCloud>();
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        pc->mutable_timestamp()->set_seconds(std::chrono::duration_cast<std::chrono::seconds>(now).count());
        pc->mutable_timestamp()->set_nanos(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000);
        pc->set_frame_id("os_sensor");
        pc->mutable_pose()->mutable_orientation()->set_w(1);
        pc->set_point_stride(4 * sizeof(float));
        const char* names[] = {"x", "y", "z", "t"};
        for (int i = 0; i < 4; i++) {
            auto* f = pc->add_fields();
            f->set_name(names[i]);
            f->set_offset(i * sizeof(float));
            f->set_type(foxglove::PackedElementField::FLOAT32);
        }

        auto live_pc = std::make_shared<foxglove::PointCloud>(*pc);

        // xyz = range * direction + offset for every return (range 0 = no return), written straight into the cloud
        std::string* data = pc->mutable_data();
        data->reserve(static_cast<size_t>(n) * 4 * sizeof(float));
        for (Eigen::Index i = 0; i < n; ++i) {
            const float r = static_cast<float>(rng[i]);
            if (r == 0.0f) continue;
            const uint64_t ts = column_ts(i % w);
            const float point[4] = {r * dir[3 * i] + ofs[3 * i], r * dir[3 * i + 1] + ofs[3 * i + 1], r * dir[3 * i + 2] + ofs[3 * i + 2],
                                    ts == 0 ? 0.0f : static_cast<float>(static_cast<double>(static_cast<int64_t>(ts) - static_cast<int64_t>(scan_end_ts)) * 1e-9)};
            data->append(reinterpret_cast<const char*>(point), sizeof(point));
        }

        std::size_t compressed_size = 0;
        auto compressed_ranges = compressRangeImage(
            range.data(), static_cast<std::size_t>(range.cols()),
            static_cast<std::size_t>(range.rows()), compressed_size);

        if (compressed_ranges) {
            // raw per-column sensor timestamps, stamped like the range image so the decompressor can pair them
            auto recorded_column_ts = std::make_shared<foxglove::RawImage>();
            *recorded_column_ts->mutable_timestamp() = pc->timestamp();
            recorded_column_ts->set_frame_id("os_sensor");
            recorded_column_ts->set_width(static_cast<uint32_t>(w));
            recorded_column_ts->set_height(1);
            recorded_column_ts->set_encoding("uint64_ns");
            recorded_column_ts->set_step(static_cast<uint32_t>(w * sizeof(uint64_t)));
            recorded_column_ts->set_data(column_ts.data(), static_cast<size_t>(w) * sizeof(uint64_t));
            core::log_mcap_only(recorded_column_ts, "ouster/column_timestamps");

            auto recorded_range = std::make_shared<foxglove::RawImage>();
            *recorded_range->mutable_timestamp() = pc->timestamp();
            recorded_range->set_frame_id("os_sensor");
            recorded_range->set_width(static_cast<uint32_t>(range.cols()));
            recorded_range->set_height(static_cast<uint32_t>(range.rows()));

            recorded_range->set_data(compressed_ranges.get(), compressed_size);
            core::log_mcap_only(recorded_range, "ouster/range");
        } else {
            spdlog::error("Failed to compress Ouster range frame");
        }

        core::log_mcap_only(pc, "PointCloud");
        core::StateTracker::instance().handle_receive_protobuf_message(pc);

        // stream just every Nth point
        constexpr int stream_stride = 16;
        const size_t point_size = 4 * sizeof(float);
        const size_t m = data->size() / point_size;
        std::string* live_data = live_pc->mutable_data();
        live_data->reserve(data->size() / stream_stride + point_size);
        for (size_t i = 0; i < m; i += stream_stride) {
            live_data->append(data->data() + i * point_size, point_size);
        }
        core::log_foxglove_only(live_pc);
        
    }
}
