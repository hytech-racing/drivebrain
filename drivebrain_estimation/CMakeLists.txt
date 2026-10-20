add_library(drivebrain_estimation
    src/SteeringEstimator.cpp
)

target_include_directories(drivebrain_estimation PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(drivebrain_estimation PUBLIC
    core
    Eigen3::Eigen
)
