cmake_minimum_required(VERSION 3.10)

add_library(drivebrain_planning INTERFACE)

target_include_directories(drivebrain_planning INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_planning INTERFACE
    drivebrain_core
    ht_proto::hytech_msgs_cpp_lib
)
