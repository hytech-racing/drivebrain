cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)

add_library(
  perception
  src/PointCloudDeskew.cpp
  src/LidarProcessor.cpp
  src/GroundRemoval.cpp
  src/Clustering.cpp
  src/ClusterFeatures.cpp
  src/PointCloudFilters.cpp
  src/ConeFilter.cpp)

target_include_directories(perception
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(perception PUBLIC core hytech_msgs_cpp_lib spdlog::spdlog)
