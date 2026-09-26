#include "OusterComms.hpp"
#include "Telemetry.hpp"
#include <memory>
#include <cstring>

#include <foxglove/PointCloud.pb.h>
#include <foxglove/PackedElementField.pb.h>

/****************************************************************
 * PUBLIC METHODS
 ****************************************************************/
comms::OusterComms::OusterComms(const std::string &device_name, bool successful) {
    // Initialize the Ouster interface
    int rc = _init(device_name);
    if (rc < 0) {
        throw std::runtime_error("Failed to initialize Ouster communications interface");
    }
}

/****************************************************************
 * PRIVATE METHODS
 ****************************************************************/
int comms::OusterComms::_init(const std::string &sensor_hostname) {

    // Establish communications with the Ouster
    ouster::sdk::core::SensorConfig config; 
    config.udp_dest = "169.254.0.1"; // TODO validate that this works reliably
    config.timestamp_mode = ouster::sdk::core::TimestampMode::TIME_FROM_PTP_1588; // use jetson's PTP clock
    spdlog::info("init claled");

    try {
        _sensors.emplace_back("169.254.77.2", config);
    }
    catch (...) {
        spdlog::error("failed in ouster init");
        return 1;
    }

    _source = std::make_unique<ouster::sdk::sensor::SensorFrameSetSource>(_sensors); // creating the client that will configure the sensors 
    // _packet = std::make_unique<ouster::sdk::sensor::SensorPacketSource>(_sensors);

    spdlog::info("initialized Ouster communications interface with sensor hostname {}", sensor_hostname);

    // LiDAR Slam setup
    _slam_config.deskew_method = "auto";
    _slam_config.min_range = 0.5; // ranges in meters - how far you are allowed to filter points from 
    _slam_config.max_range = 100.0; 
    spdlog::info("created Ouster Slam engine with and deskew method {}", _slam_config.deskew_method);

    // LUT setup
    _lut.emplace_back(*_source->sensor_info()[0], true);
    spdlog::info("initialized Ouster LUT");

    _running = true; 
    _thread = std::thread([this]() { _loop();}); //tells the thread to run the loop

    return 0;
    
}


void comms::OusterComms::_loop() {
    while (_running) {

        // if (packet_event.packet().packet_type() == ouster::sdk::sensor::ClientEvent::ERR) {
        //     spdlog::error("Sensor client error state");
        // }

        /* Lidar Data */
        std::pair<int, std::unique_ptr<ouster::sdk::core::LidarFrame>> result = _source->get_frame(0.5); 

        int index = result.first;
        if (!result.second) continue; // check that you actually received a lidar frame before dereferencing it
        auto& frame = *result.second;
        
        
        /* IMU Data */
        // auto packet_event = _packet->get_packet(1.0); // timeout is 1 second (wait up to 1 second for a packet)
        // if (packet_event.packet().type() == ouster::sdk::core::PacketType::Imu) {
        //     spdlog::info("recieved an IMU packet");

        //     auto& imu_data = packet_event.as<ouster::sdk::core::ImuPacket>

        // }


        // log to dv msgs lidar field 
        // auto timestamp = result.second->get_first_valid_packet_timestamp(); // need to catch the std runtime error if no packets are available
        // auto frame_status = result.second->frame_status; // todo - see what status the lidar can be
        // auto body_to_world = result.second->body_to_world();

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
        ouster::sdk::core::impl::cartesianT<float>(points, frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE), lut.direction, lut.offset);

        // full resolution to mcap
        core::MCAPLogger::instance().log_msg(pc);

        // every Nth point streamed to foxglove 
        constexpr Eigen::Index stream_stride = 4;
        const size_t point_size = 3 * sizeof(float);
        std::string* live_data = live_pc->mutable_data();
        live_data->resize(((n + stream_stride - 1) / stream_stride) * point_size);
        size_t m = 0;
        for (Eigen::Index i = 0; i < n; i += stream_stride, ++m) {
            std::memcpy(&(*live_data)[m * point_size], &(*data)[i * point_size], point_size);
        }

        live_data->resize(m * point_size);
        core::log_foxglove_only(live_pc);
        
    }
}
