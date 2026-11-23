find_package(GTest REQUIRED)

add_executable(drivebrain_unit_tests main.cpp PathPlannerTest.cpp)

target_link_libraries(drivebrain_unit_tests PRIVATE drivebrain_planning drivebrain_core GTest::gtest)

target_compile_definitions(drivebrain_unit_tests PRIVATE CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/drivebrain_config.json")

add_test(NAME planner_unit_tests COMMAND drivebrain_unit_tests)