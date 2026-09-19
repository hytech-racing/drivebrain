#include "SteeringEstimator.hpp"

using namespace estimation;

void SteeringEstimator::update(double analog_steer_meas) {
    std::unique_lock lk(_steering_mutex);
    _steering_angle_degs = steer_analog_to_degs.interpolate(analog_steer_meas);
}

double SteeringEstimator::get_steering_angle_degs() {
    std::unique_lock lk(_steering_mutex);
    return _steering_angle_degs;
}

double SteeringEstimator::get_steering_angle_rads() {
    std::unique_lock lk(_steering_mutex);
    return _steering_angle_degs * M_PI / 180.0;
}