#include "SteeringEstimator.h"

using namespace estimation;

void SteeringEstimator::update(double analog_steer_meas, double steering_digital_meas) {
    std::unique_lock lk(_steering_mutex);
    _steering_angle_degs = steer_analog_to_degs.interpolate(analog_steer_meas);
    _column_angle_degs = (analog_steer_meas - COLUMN_OFFSET) * COLUMN_SCALING;
}

double SteeringEstimator::get_steering_angle_degs() {
    std::unique_lock lk(_steering_mutex);
    return _steering_angle_degs;
}

double SteeringEstimator::get_steering_angle_rads() {
    std::unique_lock lk(_steering_mutex);
    return _steering_angle_degs * M_PI / 180.0;
}

double SteeringEstimator::get_column_angle_degs() {
    std::unique_lock lk(_steering_mutex);
    return _column_angle_degs;
}