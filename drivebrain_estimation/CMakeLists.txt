cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(dbcppp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(drivebrain_estimation
    src/FzEstimator.cpp
)

target_include_directories(drivebrain_estimation PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_estimation PUBLIC 
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    boost::boost
    foxglove-websocket::foxglove-websocket 
    protobuf::libprotobuf
    hytech_msgs_cpp_lib
    libvncxx
    Eigen3::Eigen
    dbcppp::dbcppp
)
