cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(mcap REQUIRED)
find_package(Eigen3 REQUIRED)

#################################
# core
#################################
add_library(core
    src/FoxgloveServer.cpp
    src/MCAPLogger.cpp
    src/StateTracker.cpp
    src/DrivebrainControllerInterface.cpp
)

target_include_directories(core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(core PUBLIC
    spdlog::spdlog
    boost::boost
    nlohmann_json::nlohmann_json
    foxglove-websocket::foxglove-websocket
    mcap::mcap
    protobuf::libprotobuf
    hytech_msgs_cpp_lib
    hytech_can_msgs_cpp_lib
    Eigen3::Eigen
    matlab_model
    simulink_automation_msgs_proto_cpp
)
