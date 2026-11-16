cmake_minimum_required(VERSION 3.10)

find_package(GTSAM REQUIRED)

add_library(drivebrain_slam
    src/SlamFrontend.cpp
    src/GraphSlam.cpp
)

target_include_directories(drivebrain_slam PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_slam PUBLIC
    drivebrain_core
    drivebrain_perception
    gtsam
)
