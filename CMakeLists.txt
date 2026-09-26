cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(drivebrain)

include(FetchContent)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake")

#################################
# Options
#################################
option(HOOTL "Build with HOOTL sim interface" OFF)

### Packages for custom libs
find_package(ht_proto REQUIRED CONFIG)
find_package(ht_can REQUIRED CONFIG)

#################################
# Custom libs
#################################
add_subdirectory(drivebrain_core)
add_subdirectory(drivebrain_comms)
add_subdirectory(drivebrain_control)
add_subdirectory(drivebrain_estimation)

#################################
# Find packages
#################################
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(mcap REQUIRED)
find_package(dbcppp REQUIRED)
find_package(Eigen3 REQUIRED)


if (HOOTL)
    find_package(cppzmq REQUIRED)
endif()

#################################
# Upstreams (non-conan)
#################################

FetchContent_Declare(
    vn_driver_lib
    GIT_REPOSITORY https://github.com/hytech-racing/vn_driver_lib.git
    GIT_TAG 0d40803
)
FetchContent_MakeAvailable(vn_driver_lib)


#################################
# Linking to main app
#################################
add_executable(drivebrain
    drivebrain_app/src/main.cpp
    drivebrain_app/src/drivebrain_app.cpp
)

target_include_directories(drivebrain PUBLIC drivebrain_app/include)

if(HOOTL)
    target_compile_definitions(drivebrain PRIVATE HOOTL_ENABLED)
    target_compile_definitions(drivebrain_core PRIVATE HOOTL_ENABLED)
    target_compile_definitions(drivebrain_comms PRIVATE HOOTL_ENABLED)
    target_link_libraries(drivebrain PRIVATE cppzmq)
endif()

target_link_libraries(drivebrain PUBLIC 
    foxglove-websocket::foxglove-websocket 
    mcap::mcap
    protobuf::libprotobuf 
    boost::boost
    ht_proto::hytech_msgs_cpp_lib
    ht_can::hytech_can_msgs_cpp_lib
    drivebrain_core
    drivebrain_comms
    drivebrain_control
    drivebrain_estimation
)

#################################
# Unit Testing
#################################

if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    # Standalone metrics test avoids hardware and the legacy SIL dependencies.
    find_package(Threads REQUIRED)
    find_package(nlohmann_json REQUIRED)
    add_executable(system_metrics_test
        tests/system_metrics/SystemMetricsTest.cpp
        drivebrain_core/src/SystemMetrics.cpp
    )
    target_include_directories(system_metrics_test PRIVATE drivebrain_core/include)
    target_link_libraries(system_metrics_test PRIVATE
        nlohmann_json::nlohmann_json spdlog::spdlog Threads::Threads)
    add_test(NAME system_metrics COMMAND system_metrics_test)
    # add_subdirectory(tests/sil)
endif()
