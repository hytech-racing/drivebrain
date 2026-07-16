cmake_minimum_required(VERSION 3.10)

add_library(perception INTERFACE)

target_include_directories(perception INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(perception INTERFACE
    core
    hytech_msgs_cpp_lib
)
