cmake_minimum_required(VERSION 3.10)

add_library(drivebrain_planning STATIC
    src/VelocityPlanner.cpp
    src/PathPlanner.cpp
)

target_include_directories(drivebrain_planning PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(drivebrain_planning SYSTEM PRIVATE
    "${delaunator_SOURCE_DIR}/include"   # only PathPlanner.cpp; header defines non-inline functions
    "${geometric_tools_SOURCE_DIR}/GTE"  # only VelocityPlanner.cpp uses NaturalCubicSpline
)

target_link_libraries(drivebrain_planning PUBLIC
    drivebrain_core
    ht_proto::hytech_msgs_cpp_lib
)
