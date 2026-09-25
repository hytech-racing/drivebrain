#include "OusterComms.hpp"

/****************************************************************
 * PUBLIC METHODS
 ****************************************************************/
comms::OusterComms::OusterComms(const std::string &device_name) {
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
    config.udp_dest = "@auto"; // TODO validate that this works reliably

    try {
        _sensors.emplace_back(sensor_hostname, config); 
    }
    catch (const std::exception& e) {
        spdlog::error("Sensor initialization failed: {}", e.what());
        return 1;
    }

    _source(_sensors); // creating the client that will configure the sensors 
    _packet(_sensors);

    spdlog::info("initialized Ouster communications interface with sensor hostname {}", sensor_hostname);

    // LiDAR Slam setup
    // _slam_config.deskew_method = "auto";
    // _slam_config.min_range = 0.5; // ranges in meters - how far you are allowed to filter points from 
    // _slam_config.max_range = 100.0; 
    // spdlog::info("created Ouster Slam engine with and deskew method {}", _slam_config.deskew_method);

    // LUT setup
    _lut.emplace_back(*_source.sensor_info()[0], true);
    spdlog::info("initialized Ouster LUT");

    _running = true; 
    _thread = std::thread([this]() { _loop();}); //tells the thread to run the loop

    return 0;
    
}


void comms::OusterComms::_loop() {
    while (_running) {

        /* IMU Data */
        auto packet_event = _packet.get_packet(1.0); // example passes 1.0 as parameter?
        if (packet_event.packet().type == ouster::sdk::core::PacketType::Imu) {
            spdlog::info("recieved an IMU packet");
            //auto imu_acc = frame.field(ouster::sdk::core::ChanField::IMU_ACC);

            // log to dv msgs imu field
        }

        if (packet_event.packet().type == ouster::sdk::sensor::ClientEvent::ERR) {
            spdlog::error("Sensor client error state");
        }


        /* Lidar Data */
        std::pair<int, std::unique_ptr<ouster::sdk::core::LidarFrame>> result = _source.get_frame(); 

        int index = result.first;
        if (!result.second) continue; // check that you actually received a lidar frame before dereferencing it
        auto& frame = *result.second;

        // log to dv msgs lidar field 
        // auto timestamp = result.second->get_first_valid_packet_timestamp(); // need to catch the std runtime error if no packets are available
        // auto frame_status = result.second->frame_status; // todo - see what status the lidar can be
        // auto body_to_world = result.second->body_to_world();


        // generate point cloud based on lookup table
        std::vector<std::vector<ouster::sdk::core::PointCloudXYZd>> cloud = _lut[index](frame); // write to dv msgs?

        
        
    }
}
