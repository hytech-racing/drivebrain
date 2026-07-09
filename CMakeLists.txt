cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(drivebrain_software_2026)

include(FetchContent)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake")

#################################
# Custom libs
#################################
add_subdirectory(drivebrain_core)
add_subdirectory(drivebrain_comms)
add_subdirectory(drivebrain_control)
add_subdirectory(drivebrain_estimation)
add_subdirectory(drivebrain_hardware)

#################################
# Find packages
#################################
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(mcap REQUIRED)
find_package(dbcppp REQUIRED)
find_package(GTSAM REQUIRED)

option(HOOTL "Build with HOOTL sim interface" OFF)
if(HOOTL)
    find_package(cppzmq REQUIRED)
endif()

#################################
# Upstreams (non-conan)
#################################
FetchContent_Declare(
    HT_Proto
    SOURCE_DIR /home/warren/cars/HT_proto
)
FetchContent_MakeAvailable(HT_Proto)

FetchContent_Declare(
    vn_driver_lib
    GIT_REPOSITORY https://github.com/hytech-racing/vn_driver_lib.git
    GIT_TAG 0d40803
)
FetchContent_MakeAvailable(vn_driver_lib)

FetchContent_Declare(
    HT_CAN
    URL "https://github.com/hytech-racing/HT_CAN/releases/download/250/drivebrain_can_lib.tar.gz"
)
FetchContent_MakeAvailable(HT_CAN)

FetchContent_Declare(
    simulink_automation_msgs_cpp
    URL "https://github.com/hytech-racing/drivebrain_simulink_models/releases/download/rel57/proto_outputs.tar.gz"
)

FetchContent_MakeAvailable(simulink_automation_msgs_cpp)

FetchContent_Declare(
    drivebrain_simulink_models
    URL "https://github.com/hytech-racing/drivebrain_simulink_models/releases/download/rel57/codegen_outputs.tar.gz"
)
FetchContent_MakeAvailable(drivebrain_simulink_models)

target_include_directories(drivebrain_core PRIVATE
    $<TARGET_PROPERTY:matlab_model,INTERFACE_INCLUDE_DIRECTORIES>
)

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
    target_compile_definitions(drivebrain_hardware PRIVATE HOOTL_ENABLED)
    target_link_libraries(drivebrain_hardware PRIVATE cppzmq)
    target_link_libraries(drivebrain PRIVATE cppzmq)

endif()

target_link_libraries(drivebrain PUBLIC
    foxglove-websocket::foxglove-websocket
    mcap::mcap
    protobuf::libprotobuf
    boost::boost
    gtsam
    hytech_msgs_cpp_lib
    hytech_can_msgs_cpp_lib
    drivebrain_core
    drivebrain_comms
    drivebrain_control
    drivebrain_hardware
    drivebrain_estimation
    matlab_model
    simulink_automation_msgs_proto_cpp
)

#################################
# Unit Testing
#################################

if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_subdirectory(tests)
endif()
