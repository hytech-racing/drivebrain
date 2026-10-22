cmake_minimum_required(VERSION 3.10)

add_library(drivebrain_perception
    src/LidarFilter.cpp
)

target_include_directories(drivebrain_perception PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_perception PUBLIC
    drivebrain_core
    ht_proto::hytech_msgs_cpp_lib
)