#include "OusterComms.hpp"
#include "Telemetry.hpp"
#include "RangeCompression.hpp"
#include "dv_msgs.pb.h"
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
              spdlog::warn("Retrying Ouster initialization");
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
        core::log_mcap_only(calibration, "ouster/calibration");
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

        // write xyz directly into the foxglove pointcloud buffer
        const auto& lut = _lut[index];
        const Eigen::Index n = lut.direction.rows();

        auto pc = std::make_shared<foxglove::PointCloud>();
        pc->set_frame_id("os_sensor");
        pc->mutable_pose()->mutable_orientation()->set_w(1);
        pc->set_point_stride(3 * sizeof(float));
        const char* names[] = {"x", "y", "z"};
        for (int i = 0; i < 3; i++) {
            auto* f = pc->add_fields();
            f->set_name(names[i]);
            f->set_offset(i * sizeof(float));
            f->set_type(foxglove::PackedElementField::FLOAT32);
        }

        auto live_pc = std::make_shared<foxglove::PointCloud>(*pc);

        std::string* data = pc->mutable_data();
        data->resize(n * 3 * sizeof(float));
        Eigen::Map<ouster::sdk::core::PointCloudXYZf> points(reinterpret_cast<float*>(data->data()), n, 3);
        const auto range = frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);

        std::size_t compressed_size = 0;
        auto compressed_ranges = compressRangeImage(
            range.data(), static_cast<std::size_t>(range.cols()),
            static_cast<std::size_t>(range.rows()), compressed_size);

        if (compressed_ranges) {
            auto recorded_range = std::make_shared<foxglove::RawImage>();
            const auto now = std::chrono::system_clock::now().time_since_epoch();
            const auto secs = std::chrono::duration_cast<std::chrono::seconds>(now).count();
            const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000;

            recorded_range->mutable_timestamp()->set_seconds(secs);
            recorded_range->mutable_timestamp()->set_nanos(nanos);
            recorded_range->set_frame_id("os_sensor");
            recorded_range->set_width(static_cast<uint32_t>(range.cols()));
            recorded_range->set_height(static_cast<uint32_t>(range.rows()));

            recorded_range->set_data(compressed_ranges.get(), compressed_size);
            core::log_mcap_only(recorded_range, "ouster/range");
        } else {
            spdlog::error("Failed to compress Ouster range frame");
        }

        ouster::sdk::core::impl::cartesianT<float>(points, range, lut.direction, lut.offset);

        // stream just every Nth point that has a return (range 0 = no return)
        constexpr int stream_stride = 16;
        const size_t point_size = 3 * sizeof(float);
        std::string* live_data = live_pc->mutable_data();
        live_data->resize(data->size());
        size_t m = 0;
        int valid = 0;
        for (Eigen::Index i = 0; i < n; ++i) {
            if (range.data()[i] == 0 || valid++ % stream_stride != 0) continue;
            std::memcpy(&(*live_data)[m * point_size], &(*data)[i * point_size], point_size);
            ++m;
        }
        live_data->resize(m * point_size);
        core::log_foxglove_only(live_pc);
        
    }
}
