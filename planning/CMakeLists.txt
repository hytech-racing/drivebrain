cmake_minimum_required(VERSION 3.10)

add_library(planning INTERFACE)

target_include_directories(planning INTERFACE 
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/planning/external/delaunator-cpp/include"
)

target_link_libraries(planning INTERFACE
    core
    hytech_msgs_cpp_lib
)


