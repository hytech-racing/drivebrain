cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(drivebrain)

include(FetchContent)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cuda_modules")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#################################
# Options
#################################
option(HOOTL "Build with HOOTL sim interface" OFF)
option(JETSON "Build for the Jetson Orin (else Pi)" OFF)

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
find_package(foxglove-schemas-protobuf REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(mcap REQUIRED)
find_package(dbcppp REQUIRED)
find_package(aravis REQUIRED)
find_package(OpenCV REQUIRED CONFIG)
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
    GIT_TAG d98c8a3892007eb215ac780c30d50a4577d9ecdc
)
FetchContent_MakeAvailable(HT_Proto)
# add_subdirectory(../HT_proto ${CMAKE_BINARY_DIR}/HT_proto)

FetchContent_Declare(
    vn_driver_lib
    GIT_REPOSITORY https://github.com/hytech-racing/vn_driver_lib.git
    GIT_TAG 0d40803
)
FetchContent_MakeAvailable(vn_driver_lib)

FetchContent_Declare(
    HT_CAN
    URL "https://github.com/hytech-racing/HT_CAN/releases/download/268/drivebrain_can_lib.tar.gz"
)
FetchContent_MakeAvailable(HT_CAN)

find_package(OusterSDK REQUIRED)

#################################
# Linking to main app
#################################
add_executable(drivebrain
    drivebrain_app/src/main.cpp
    drivebrain_app/src/drivebrain_app.cpp
)

#add_executable(drivebrain
#    tests/hil/scripts/test_ouster.cpp
#)

target_include_directories(drivebrain PUBLIC drivebrain_app/include)

# add_executable(drivebrain
#     tests/hil/scripts/test_backfly.cpp
# )


if(HOOTL)
    target_compile_definitions(drivebrain PRIVATE HOOTL_ENABLED)
    target_compile_definitions(drivebrain_core PRIVATE HOOTL_ENABLED)
    target_compile_definitions(drivebrain_comms PRIVATE HOOTL_ENABLED)
    target_link_libraries(drivebrain PRIVATE cppzmq)
endif()

if(JETSON)
    target_compile_definitions(drivebrain PRIVATE JETSON_ENABLED)
    target_compile_definitions(drivebrain_core PRIVATE JETSON_ENABLED)
    include(TensorRT)
    set(CMAKE_SKIP_BUILD_RPATH ON)
    target_link_libraries(drivebrain PRIVATE TensorRT)
endif()

target_link_libraries(drivebrain PUBLIC 
    foxglove-websocket::foxglove-websocket 
    foxglove-schemas-protobuf::foxglove-schemas-protobuf
    mcap::mcap
    protobuf::libprotobuf 
    boost::boost
    hytech_msgs_cpp_lib
    hytech_can_msgs_cpp_lib
    drivebrain_core
    drivebrain_comms
    drivebrain_control
    # ouster_client
    # ouster_sensor
    # ouster_perception
    aravis::aravis
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    drivebrain_estimation
)

if(JETSON)
    target_link_libraries(drivebrain PUBLIC TensorRT)
endif()


#################################
# Unit Testing
#################################

if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    # add_subdirectory(tests/sil)
endif()