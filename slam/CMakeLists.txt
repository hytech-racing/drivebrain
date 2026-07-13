cmake_minimum_required(VERSION 3.10)

add_library(slam INTERFACE)

target_include_directories(slam INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
