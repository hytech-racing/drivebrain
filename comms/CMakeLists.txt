cmake_minimum_required(VERSION 3.10)

find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(foxglove-websocket REQUIRED)
find_package(Protobuf REQUIRED CONFIG)
find_package(dbcppp REQUIRED)
find_package(cppzmq REQUIRED)
find_package(aravis REQUIRED CONFIG)

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

add_library(comms ${DRIVEBRAIN_COMMS_SRCS})

target_include_directories(comms PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(comms PUBLIC 
    core
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    boost::boost
    foxglove-websocket::foxglove-websocket 
    protobuf::libprotobuf
    hytech_msgs_cpp_lib
    libvncxx
    dbcppp::dbcppp
    aravis::aravis
)

if(HOOTL)
    target_link_libraries(comms
        PUBLIC
            cppzmq
    )
endif()


option(DRIVEBRAIN_FLIR "Build the Jetson TensorRT FLIR driver" OFF)
if(DRIVEBRAIN_FLIR)
    if(CMAKE_VERSION VERSION_LESS 3.17)
        message(FATAL_ERROR "The FLIR driver requires CMake 3.17 or newer")
    endif()
    find_package(CUDAToolkit REQUIRED)
    find_package(OpenCV REQUIRED CONFIG)
    find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
    find_library(TENSORRT_LIBRARY nvinfer)
    find_library(TENSORRT_ONNX_LIBRARY nvonnxparser)
    if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY OR NOT TENSORRT_ONNX_LIBRARY)
        message(FATAL_ERROR "Install TensorRT 10.x development libraries and its ONNX parser from JetPack")
    endif()
    if(CMAKE_EXE_LINKER_FLAGS MATCHES "(^| )-static( |$)")
        message(FATAL_ERROR "FLIR uses the JetPack runtime libraries. Configure this build with -DCMAKE_EXE_LINKER_FLAGS=")
    endif()
    target_sources(comms PRIVATE src/FLIRComms.cpp)
    target_compile_definitions(comms PUBLIC DRIVEBRAIN_FLIR_ENABLED=1)
    target_include_directories(comms PRIVATE "${TENSORRT_INCLUDE_DIR}")
    target_link_libraries(comms PRIVATE
        opencv_core opencv_imgproc opencv_imgcodecs
        CUDA::cudart "${TENSORRT_LIBRARY}" "${TENSORRT_ONNX_LIBRARY}"
    )
endif()
