cmake_minimum_required(VERSION 3.10)

find_package(GTSAM REQUIRED)

add_library(drivebrain_slam
    src/GraphSLAM.cpp
)

target_include_directories(drivebrain_slam PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_slam PUBLIC
    drivebrain_core
    gtsam
    hytech_msgs_cpp_lib
)
