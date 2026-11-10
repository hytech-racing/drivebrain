cmake_minimum_required(VERSION 3.20)
project(uart_contract_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(Protobuf CONFIG QUIET)
if(NOT Protobuf_FOUND)
    find_package(Protobuf REQUIRED)
endif()
find_package(spdlog CONFIG REQUIRED)
set(HT_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../HT_proto/proto" CACHE PATH "HT_proto schema directory")
foreach(schema base_msgs hytech_msgs)
    if(NOT EXISTS "${HT_PROTO_DIR}/${schema}.proto")
        message(FATAL_ERROR "Set HT_PROTO_DIR to matching HT_proto sources")
    endif()
endforeach()
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
add_custom_command(
    OUTPUT generated/base_msgs.pb.cc generated/base_msgs.pb.h generated/hytech_msgs.pb.cc generated/hytech_msgs.pb.h
    COMMAND protobuf::protoc --cpp_out=${CMAKE_CURRENT_BINARY_DIR}/generated -I ${HT_PROTO_DIR}
        ${HT_PROTO_DIR}/base_msgs.proto ${HT_PROTO_DIR}/hytech_msgs.proto
    DEPENDS ${HT_PROTO_DIR}/base_msgs.proto ${HT_PROTO_DIR}/hytech_msgs.proto
    VERBATIM)
add_library(uart_messages generated/base_msgs.pb.cc generated/hytech_msgs.pb.cc)
target_include_directories(uart_messages PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(uart_messages PUBLIC protobuf::libprotobuf)

add_library(uart_transport ../../drivebrain_comms/src/UARTComms.cpp)
target_include_directories(uart_transport PUBLIC ../../drivebrain_comms/include)
target_link_libraries(uart_transport PUBLIC uart_messages Boost::headers spdlog::spdlog Threads::Threads)
target_compile_options(uart_transport PRIVATE -Wall -Wextra -Wpedantic)
add_executable(uart_contract_test uart_contract_test.cpp)
target_link_libraries(uart_contract_test PRIVATE uart_transport)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(uart_contract_test PRIVATE util)
endif()
enable_testing()
add_test(NAME uart_contract COMMAND uart_contract_test)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME uart_wire_reference COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/verify_wire.py
    $<TARGET_FILE:uart_contract_test> ${CMAKE_CURRENT_BINARY_DIR}/fixtures)
