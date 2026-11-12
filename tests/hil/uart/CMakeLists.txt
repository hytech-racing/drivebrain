cmake_minimum_required(VERSION 3.20)
project(uart_stress_peer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(Protobuf CONFIG QUIET)
if(NOT Protobuf_FOUND)
    find_package(Protobuf REQUIRED)
endif()
find_package(spdlog CONFIG REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(HT_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../HT_proto/proto" CACHE PATH "HT_proto schema sources")
set(GENERATED "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GENERATED}")
execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_messages.py
    --ht-proto ${HT_PROTO_DIR} --out ${GENERATED} COMMAND_ERROR_IS_FATAL ANY)
set(PROTOS ${HT_PROTO_DIR}/base_msgs.proto ${HT_PROTO_DIR}/hytech_msgs.proto ${CMAKE_CURRENT_SOURCE_DIR}/proto/uart_bench.proto)
add_custom_command(OUTPUT ${GENERATED}/base_msgs.pb.cc ${GENERATED}/base_msgs.pb.h
    ${GENERATED}/hytech_msgs.pb.cc ${GENERATED}/hytech_msgs.pb.h
    ${GENERATED}/uart_bench.pb.cc ${GENERATED}/uart_bench.pb.h
    COMMAND protobuf::protoc --cpp_out=${GENERATED} -I ${HT_PROTO_DIR}
        -I ${CMAKE_CURRENT_SOURCE_DIR}/proto ${PROTOS}
    DEPENDS ${PROTOS} VERBATIM)
add_library(uart_bench_transport ../../../comms/src/UARTComms.cpp
    ${GENERATED}/base_msgs.pb.cc ${GENERATED}/hytech_msgs.pb.cc ${GENERATED}/uart_bench.pb.cc)
target_include_directories(uart_bench_transport PUBLIC ../../../comms/include ${GENERATED})
target_link_libraries(uart_bench_transport PUBLIC protobuf::libprotobuf Boost::headers spdlog::spdlog Threads::Threads)
add_executable(uart_stress_peer pi/uart_stress_peer.cpp)
target_link_libraries(uart_stress_peer PRIVATE uart_bench_transport)
target_compile_options(uart_stress_peer PRIVATE -Wall -Wextra -Wpedantic)
enable_testing()
add_test(NAME uart_stress_peer_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_peer.py $<TARGET_FILE:uart_stress_peer>)
add_test(NAME uart_tester_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_tester.py)
add_test(NAME uart_checkpoint_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_checkpoints.py)
