cmake_minimum_required(VERSION 3.13)
project(launch_profile_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
enable_testing()
add_executable(launch_profile_tests test_launch_profile.cpp)
target_include_directories(launch_profile_tests PRIVATE ../../drivebrain_control/include)
add_test(NAME launch_profile COMMAND launch_profile_tests)
add_executable(launch_ground_heading_tests test_ground_heading.cpp)
target_include_directories(launch_ground_heading_tests PRIVATE
    ../../drivebrain_control/include ../../drivebrain_estimation/include)
add_test(NAME launch_ground_heading COMMAND launch_ground_heading_tests)
add_executable(launch_remote_mission_tests test_remote_mission.cpp)
target_include_directories(launch_remote_mission_tests PRIVATE
    ../../drivebrain_control/include ../../drivebrain_core/include)
add_test(NAME launch_remote_mission COMMAND launch_remote_mission_tests)

if(TARGET drivebrain_control)
    add_executable(launch_wheel_feedback_tests test_wheel_feedback.cpp)
    target_compile_features(launch_wheel_feedback_tests PRIVATE cxx_std_20)
    target_link_libraries(launch_wheel_feedback_tests PRIVATE drivebrain_control drivebrain_comms)
    add_test(NAME launch_wheel_feedback COMMAND launch_wheel_feedback_tests
        ${CMAKE_SOURCE_DIR}/config/drivebrain_config.json)
endif()
