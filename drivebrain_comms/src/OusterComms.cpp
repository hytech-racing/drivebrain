
#include "OusterComms.hpp"

/****************************************************************
 * PUBLIC METHODS
 ****************************************************************/
comms::OusterComms::OusterComms(const std::string &device_name) {
    // Initialize the Ouster interface
    int rc = _init();
    if (rc < 0) {
        throw std::runtime_error("Failed to initialize Ouster communications interface");
    }
}

/****************************************************************
 * PRIVATE METHODS
 ****************************************************************/
int comms::OusterComms::_init(const std::string &sensor_hostname) {

    // Establish communications with the Ouster
    core::SensorConfig config; 
    config.udp_dest = "@auto"; // TODO validate that this works reliably
    _sensors.emplace_back(sensor_hostname, config); 
    _source(_sensors);

    spdlog::info("initialized Ouster communications interface with sensor hostname {}", sensor_hostname);

    // LiDAR Slam setup
    _slam_config.backend = "kiss";
    _slam_config.deskew_method = "auto";
    // TODO make these configurable
    _slam_config.min_range = 0.5; 
    _slam_config.max_range = 100.0/ 

    _slam_engine(_source.sensor_info(), _slam_config);

    spdlog::info("created Ouster Slam engine with backend {} and deskew method {}", _slam_config.backend, _slam_config.deskew_method);

    // LUT setup
    _lut(*_source.sensor_info()[0], true);

    spdlog::info("initialized Ouster LUT");

    _running = true; 
    _thread = std::thread([this]() { _loop();});
    
}

void _loop() {
    while (_running) {
        auto [idx, scan] = _source.get_scan();
        if (!scan) continue;

        auto cloud = _lut(*scan);

        auto status = scan.status();
        auto it = std::find_if(status.data(), status.data() + status.size(), [](const auto& status_val) { return status_val & 0x01; });

        if (!scan->complete()) {
            spdlog::warn("incomplete scan received from Ouster sensor, skipping");
        }

        for
    }
}