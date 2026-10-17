cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(dbcppp REQUIRED)

add_library(drivebrain_control
    src/LoadCellTorqueController.cpp
    src/SimplePowerLimiter.cpp
)

target_include_directories(drivebrain_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_control PUBLIC 
    drivebrain_core
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    boost::boost
    foxglove-websocket::foxglove-websocket 
    protobuf::libprotobuf
    hytech_msgs_cpp_lib
    libvncxx
    dbcppp::dbcppp
)
