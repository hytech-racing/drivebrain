set(JETSON_SYSROOT $ENV{JETSON_SYSROOT})
if(NOT JETSON_SYSROOT)
    message(FATAL_ERROR "JETSON_SYSROOT not set - build inside the drivebrain Docker container")
endif()

set(PHOENIX6_LIB_DIR ${JETSON_SYSROOT}/usr/lib/phoenix6)

add_library(phoenix6 INTERFACE)
target_include_directories(phoenix6 SYSTEM INTERFACE
    ${JETSON_SYSROOT}/usr/include)
target_link_libraries(phoenix6 INTERFACE
    ${PHOENIX6_LIB_DIR}/libCTRE_Phoenix6.so
    ${PHOENIX6_LIB_DIR}/libCTRE_PhoenixTools.so)
target_link_options(phoenix6 INTERFACE -Wl,-rpath-link,${PHOENIX6_LIB_DIR})
