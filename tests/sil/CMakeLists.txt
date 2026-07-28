# ##############################################################################
# Unit Testing
# ##############################################################################

find_package(GTest REQUIRED)

add_executable(
  drivebrain_test
  main.cpp MCAPLoggerTest.cpp ReferencePathTest.cpp
  adapters/EstimatorMessageAdaptersTest.cpp
  estimation/DriverlessEstimatorRunnerTest.cpp
  estimation/GssModelTest.cpp estimation/LatestEstimateTest.cpp
  transforms/TransformBufferTest.cpp)

target_link_libraries(
  drivebrain_test
  PUBLIC foxglove-websocket::foxglove-websocket
         mcap::mcap
         adapters
         core
         planning
         estimation
         transforms
         GTest::gtest)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_configs
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

target_compile_definitions(
  drivebrain_test
  PRIVATE TEST_CONFIG_DIR="${CMAKE_CURRENT_BINARY_DIR}/test_configs")

include(GoogleTest)
gtest_discover_tests(drivebrain_test)

# ##############################################################################
# Scripts
# ##############################################################################
file(GLOB SCRIPT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.cpp")

foreach(SCRIPT_PATH ${SCRIPT_SOURCES})
  get_filename_component(SCRIPT_NAME ${SCRIPT_PATH} NAME_WE)

  # add_executable(${SCRIPT_NAME} ${SCRIPT_PATH})

  # target_link_libraries(${SCRIPT_NAME} PRIVATE
  # foxglove-websocket::foxglove-websocket mcap::mcap core comms )
endforeach()
