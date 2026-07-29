# ##############################################################################
# Unit Testing
# ##############################################################################

find_package(GTest REQUIRED)

add_executable(
  drivebrain_test
  main.cpp MCAPLoggerTest.cpp ReferencePathTest.cpp
  adapters/EstimatorMessageAdaptersTest.cpp
  estimation/GssModelTest.cpp estimation/LatestEstimateTest.cpp
  perception/ClusterFeaturesTest.cpp
  perception/ClusteringTest.cpp
  perception/ConeFilterTest.cpp
  perception/GroundRemovalTest.cpp
  perception/LidarProcessorTest.cpp
  perception/PointCloudFiltersTest.cpp
  perception/PointCloudDeskewTest.cpp
  slam/IncrementalGraphSlamTest.cpp
  slam/SlamFrontendAssociationTest.cpp
  slam/SlamFrontendMapStateTest.cpp
  slam/SlamFrontendOptimizedAssociationTest.cpp
  slam/SlamFrontendTrackingTest.cpp
  runtime/DriverlessEstimatorRunnerTest.cpp
  runtime/PerceptionFrontendRunnerTest.cpp
  transforms/TransformBufferTest.cpp)

target_link_libraries(
  drivebrain_test
  PUBLIC foxglove-websocket::foxglove-websocket
         mcap::mcap
         adapters
         core
         planning
         estimation
          runtime
          slam
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
