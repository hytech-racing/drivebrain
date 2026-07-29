cmake_minimum_required(VERSION 3.10)

find_package(Eigen3 REQUIRED)
add_library(estimation src/LatestEstimate.cpp src/EkfEstimator.cpp src/GssModel.cpp
                       src/MotionModel.cpp)

target_include_directories(estimation
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(estimation PUBLIC transforms Eigen3::Eigen)
