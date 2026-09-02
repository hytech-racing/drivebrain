cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(mcap REQUIRED)

add_library(drivebrain_core
    src/FoxgloveServer.cpp
    src/MCAPLogger.cpp
    src/StateTracker.cpp
    src/DrivebrainControllerInterface.cpp
)

target_include_directories(drivebrain_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_core PUBLIC 
    drivebrain_estimation
    spdlog::spdlog 
    boost::boost 
    nlohmann_json::nlohmann_json
    foxglove-websocket::foxglove-websocket
    mcap::mcap
    protobuf::libprotobuf
    hytech_msgs_cpp_lib
    hytech_can_msgs_cpp_lib
)