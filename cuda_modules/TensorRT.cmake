set(JETSON_SYSROOT $ENV{JETSON_SYSROOT})
if(NOT JETSON_SYSROOT)
    message(FATAL_ERROR "JETSON_SYSROOT not set - build inside the drivebrain Docker container")
endif()

file(GLOB CUDA_ROOT ${JETSON_SYSROOT}/usr/local/cuda-*/targets/*)   # .../cuda-13.2/targets/sbsa-linux
set(TRT_LIB_DIR ${JETSON_SYSROOT}/usr/lib/aarch64-linux-gnu)

add_library(TensorRT INTERFACE)
target_include_directories(TensorRT SYSTEM INTERFACE
    ${CUDA_ROOT}/include
    ${JETSON_SYSROOT}/usr/include/aarch64-linux-gnu)
target_link_libraries(TensorRT INTERFACE
    ${TRT_LIB_DIR}/libnvinfer.so
    ${TRT_LIB_DIR}/libnvinfer_plugin.so
    ${CUDA_ROOT}/lib/libcudart.so)
target_link_options(TensorRT INTERFACE -Wl,-rpath-link,${TRT_LIB_DIR})
