cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(dbcppp REQUIRED)
find_package(cppzmq REQUIRED)

set(DRIVEBRAIN_COMMS_SRCS
    src/CANComms.cpp
    src/VNComms.cpp
    src/ETHSendComms.cpp
)

if(HOOTL)
    find_package(cppzmq REQUIRED)
    list(APPEND DRIVEBRAIN_COMMS_SRCS
        src/SimComms.cpp
    )
endif()

add_library(drivebrain_comms ${DRIVEBRAIN_COMMS_SRCS})

target_include_directories(drivebrain_comms PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drivebrain_comms PUBLIC 
    drivebrain_core
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    boost::boost
    foxglove-websocket::foxglove-websocket 
    protobuf::libprotobuf
    hytech_msgs_cpp_lib
    libvncxx
    dbcppp::dbcppp
)

if(HOOTL)
    target_link_libraries(drivebrain_comms
        PUBLIC
            cppzmq
    )
endif()