#include "OusterComms.hpp"
#include "Telemetry.hpp"
#include "dv_msgs.pb.h"
#include <memory>
#include <cstring>

#include <foxglove/PointCloud.pb.h>
#include <foxglove/PackedElementField.pb.h>
#include <ouster/core/frame_set.h>
#include <ouster/sensor/sensor_packet_source.h>

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


    /* if the auto deskew method doesnt work, calculate accel bias in init/at starting, and integrate that to get velocity?
    */

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
 
        std::shared_ptr<ouster::sdk::core::LidarFrame> lidar_frame = std::move(result.second);
        ouster::sdk::core::FrameSet frame_set{lidar_frame};

        /* IMU Data */
        auto packet_event = _packet->get_packet(1.0); // timeout is 1 second (wait up to 1 second for a packet)
        if (packet_event.packet().type() == ouster::sdk::core::PacketType::Imu) {
            spdlog::info("recieved an IMU packet");

            const auto& imu = packet_event.packet().as<ouster::sdk::core::ImuPacket>();

            imu_status = imu.status();
            imu_timestamp = imu.timestamp();
            imu_accel = imu.accel();
            imu_gyro  = imu.gyro();

            std::shared_ptr<dv_msgs::LidarIMU> imu_data_out = std::make_shared<dv_msgs::LidarIMU>();

            for (int i = 0; i < imu_accel.rows(); i++) {
                imu_data_out->set_accelerometer_x(imu_accel(i, 0));
                imu_data_out->set_accelerometer_y(imu_accel(i, 1));
                imu_data_out->set_accelerometer_z(imu_accel(i, 2));

                imu_data_out->set_gyro_x(imu_gyro(i, 2)); 
                imu_data_out->set_gyro_y(imu_gyro(i, 2));
                imu_data_out->set_gyro_z(imu_gyro(i, 2));
            }

            core::log(imu_data_out);
            spdlog::info("logged lidar imu data");

        }


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
        const auto range = frame.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
        ouster::sdk::core::impl::cartesianT<float>(points, range, lut.direction, lut.offset);

        // full resolution to mcap
        core::MCAPLogger::instance().log_msg(pc);

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
