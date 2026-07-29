cmake_minimum_required(VERSION 3.10)

find_package(GTSAM REQUIRED)
find_package(spdlog REQUIRED)

add_library(slam src/frontend/SlamFrontend.cpp
                 src/backend/IncrementalGraphSlam.cpp)

target_include_directories(slam PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(slam PUBLIC core gtsam hytech_msgs_cpp_lib transforms
                                  spdlog::spdlog)
