cmake_minimum_required(VERSION 3.10)

add_library(planning STATIC
    src/VelocityPlanner.cpp
)

target_include_directories(planning PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${delaunator_SOURCE_DIR}/include"
)

target_link_libraries(planning PUBLIC
    core
    hytech_msgs_cpp_lib
)

target_include_directories(planning SYSTEM PRIVATE
    "${geometric_tools_SOURCE_DIR}/GTE"
)



