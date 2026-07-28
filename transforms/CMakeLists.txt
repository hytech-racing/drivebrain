cmake_minimum_required(VERSION 3.10)

find_package(Eigen3 REQUIRED)

add_library(transforms src/TransformBuffer.cpp)

target_include_directories(transforms
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(transforms PUBLIC Eigen3::Eigen)
