cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(mcap REQUIRED)
find_package(Eigen3 REQUIRED)

#################################
# foxglove protos
#################################
set(FOXGLOVE_PROTO_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/)
file(MAKE_DIRECTORY ${FOXGLOVE_PROTO_BINARY_DIR})

add_library(foxglove_protos_obj OBJECT
    proto/foxglove/PointCloud.proto
    proto/foxglove/PackedElementField.proto
    proto/foxglove/Pose.proto
    proto/foxglove/Vector3.proto
    proto/foxglove/Quaternion.proto
    proto/foxglove/FrameTransform.proto
)
target_include_directories(foxglove_protos_obj PUBLIC $<BUILD_INTERFACE:${FOXGLOVE_PROTO_BINARY_DIR}>)
target_link_libraries(foxglove_protos_obj PUBLIC protobuf::libprotobuf)
protobuf_generate(TARGET foxglove_protos_obj IMPORT_DIRS proto PROTOC_OUT_DIR "${FOXGLOVE_PROTO_BINARY_DIR}")

add_library(foxglove_protos $<TARGET_OBJECTS:foxglove_protos_obj>)
target_include_directories(foxglove_protos PUBLIC $<BUILD_INTERFACE:${FOXGLOVE_PROTO_BINARY_DIR}>)
target_link_libraries(foxglove_protos PUBLIC protobuf::libprotobuf)

#################################
# core
#################################
add_library(core
    src/FoxgloveServer.cpp
    src/MCAPLogger.cpp
    src/StateTracker.cpp
    src/DrivebrainControllerInterface.cpp
)

target_include_directories(core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(core PUBLIC
    spdlog::spdlog
    boost::boost
    nlohmann_json::nlohmann_json
    foxglove-websocket::foxglove-websocket
    mcap::mcap
    protobuf::libprotobuf
    hytech_msgs_cpp_lib
    hytech_can_msgs_cpp_lib
    foxglove_protos
    Eigen3::Eigen
    matlab_model
    simulink_automation_msgs_proto_cpp
)