cmake_minimum_required(VERSION 3.10)

add_library(perception INTERFACE)

target_include_directories(perception INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
