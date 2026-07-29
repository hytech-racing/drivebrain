cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)

add_library(runtime src/DriverlessEstimatorRunner.cpp src/PerceptionFrontendRunner.cpp)

target_include_directories(runtime PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(runtime PUBLIC transforms perception estimation adapters spdlog::spdlog core)
