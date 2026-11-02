cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(drivebrain)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cuda_modules")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#################################
# Options
#################################
option(HOOTL "Build with HOOTL sim interface" OFF)
option(JETSON "Build for the Jetson Orin (else Pi)" OFF)

find_package(ht_proto CONFIG REQUIRED)
find_package(ht_can CONFIG REQUIRED)

#################################
# Upstreams needed by the custom libs below
#################################
FetchContent_Declare(
    delaunator
    GIT_REPOSITORY https://github.com/delfrrr/delaunator-cpp.git
    GIT_TAG v0.4.0
    SOURCE_SUBDIR  none
)
FetchContent_MakeAvailable(delaunator)

#################################
# Custom libs
#################################
add_subdirectory(drivebrain_core)
add_subdirectory(drivebrain_comms)
add_subdirectory(drivebrain_control)
add_subdirectory(drivebrain_estimation)
add_subdirectory(drivebrain_planning)
add_subdirectory(drivebrain_perception)
add_subdirectory(drivebrain_slam)

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
find_package(GTSAM REQUIRED)
find_package(Tracy REQUIRED)

if (HOOTL)
    find_package(cppzmq REQUIRED)
endif()

#################################
# Upstreams (non-conan)
#################################
# 
FetchContent_Declare(
    vn_driver_lib
    GIT_REPOSITORY https://github.com/hytech-racing/vn_driver_lib.git
    GIT_TAG 0d40803
)
FetchContent_MakeAvailable(vn_driver_lib)

find_package(OusterSDK REQUIRED)
find_package(Git REQUIRED)

# 2. Execute Git commands to harvest metadata
execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --format=%h
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --format=%ci
    OUTPUT_VARIABLE GIT_DATE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND ${GIT_EXECUTABLE} status --porcelain
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(GIT_STATUS)
    set(GIT_HAS_UNCOMMITTED_CHANGES "true")
else()
    set(GIT_HAS_UNCOMMITTED_CHANGES "false")
endif()

# 3. Generate the actual header file from the template
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated/include)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/include/version.h
)

#################################
# Linking to main app
#################################
add_executable(drivebrain
    drivebrain_app/src/main.cpp
    drivebrain_app/src/drivebrain_app.cpp
    drivebrain_app/src/autonomy.cpp
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
    target_compile_definitions(drivebrain_perception PRIVATE HOOTL_ENABLED)
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
    gtsam
    ht_proto::hytech_msgs_cpp_lib
    ht_can::hytech_can_msgs_cpp_lib
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
    drivebrain_planning
    drivebrain_perception
    drivebrain_slam
    Tracy::TracyClient
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
