cmake_minimum_required(VERSION 3.10)

add_library(planning
    src/PathPlanner.cpp
    include/PathPlanner.hpp
)

target_include_directories(planning PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${delaunator_SOURCE_DIR}/include"
)

target_link_libraries(planning PUBLIC
    core
    hytech_msgs_cpp_lib
)


