cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(dbcppp REQUIRED)

include(FetchContent)
FetchContent_Declare(
  osqp-cpp
  GIT_REPOSITORY https://github.com/google/osqp-cpp.git
  GIT_TAG master
)

FetchContent_GetProperties(osqp-cpp)
if(NOT osqp-cpp_POPULATED)
  FetchContent_Populate(osqp-cpp)
  add_subdirectory(${osqp-cpp_SOURCE_DIR} ${osqp-cpp_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

add_library(drivebrain_control
    src/LoadCellTorqueController.cpp
    src/SimplePowerLimiter.cpp
    src/PurePursuitController.cpp
    src/TeleopController.cpp
    src/BasicQP.cpp
)

target_include_directories(drivebrain_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(drivebrain_control PUBLIC "${osqp-cpp_SOURCE_DIR}/include")

target_link_libraries(drivebrain_control PUBLIC 
    drivebrain_core
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    boost::boost
    foxglove-websocket::foxglove-websocket 
    protobuf::libprotobuf
    ht_proto::hytech_msgs_cpp_lib
    libvncxx
    dbcppp::dbcppp
    osqp-cpp
)
