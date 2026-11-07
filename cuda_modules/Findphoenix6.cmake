set(JETSON_SYSROOT $ENV{JETSON_SYSROOT})
if(NOT JETSON_SYSROOT)
    message(FATAL_ERROR "JETSON_SYSROOT not set - build inside the drivebrain Docker container")
endif()

# extracted from CTRE's phoenix6 arm64 .deb; the Orin gets the same version via ansible
set(phoenix6_LIB_DIR ${JETSON_SYSROOT}/usr/lib/phoenix6)

# NO_CMAKE_FIND_ROOT_PATH so the conan toolchain doesn't re-root the sysroot paths
find_path(phoenix6_INCLUDE_DIR ctre/phoenix6/TalonFX.hpp
    PATHS ${JETSON_SYSROOT}/usr/include/phoenix6
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
find_library(phoenix6_LIBRARY CTRE_Phoenix6
    PATHS ${phoenix6_LIB_DIR}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
find_library(phoenix6_TOOLS_LIBRARY CTRE_PhoenixTools
    PATHS ${phoenix6_LIB_DIR}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(phoenix6
    REQUIRED_VARS phoenix6_LIBRARY phoenix6_TOOLS_LIBRARY phoenix6_INCLUDE_DIR)

if(phoenix6_FOUND AND NOT TARGET phoenix6)
    add_library(phoenix6 INTERFACE IMPORTED)
    target_include_directories(phoenix6 SYSTEM INTERFACE ${phoenix6_INCLUDE_DIR})
    target_link_libraries(phoenix6 INTERFACE ${phoenix6_LIBRARY} ${phoenix6_TOOLS_LIBRARY})
    target_link_options(phoenix6 INTERFACE -Wl,-rpath-link,${phoenix6_LIB_DIR})
    target_compile_options(phoenix6 INTERFACE -Wno-psabi)
endif()
