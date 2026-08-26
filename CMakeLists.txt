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
    HT_Proto 
    GIT_REPOSITORY https://github.com/hytech-racing/HT_proto.git
    GIT_TAG f4fc651
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
    URL "https://github.com/hytech-racing/HT_CAN/releases/download/264/drivebrain_can_lib.tar.gz"
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
    hytech_msgs_cpp_lib
    hytech_can_msgs_cpp_lib
    drivebrain_core
    drivebrain_comms
    drivebrain_control
    matlab_model
    -Wl,--whole-archive
    simulink_automation_msgs_proto_cpp
    -Wl,--no-whole-archive
)

#################################
# Unit Testing
#################################

if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    # add_subdirectory(tests/sil)
endif()
