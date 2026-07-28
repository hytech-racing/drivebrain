cmake_minimum_required(VERSION 3.10)

add_library(adapters src/EstimatorMessageAdapters.cpp)

target_include_directories(adapters
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(adapters PUBLIC estimation hytech_msgs_cpp_lib)
