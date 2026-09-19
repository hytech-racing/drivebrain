#pragma once
/** 
 * Estimates steering angle from analog steering sensor
 */
#include <Eigen/Dense>
#include <mutex>
#include <FoxgloveServer.hpp>  
#include "InterpolatingTable.h"
#include <cmath>

using namespace core;

namespace estimation {

class SteeringEstimator {

    public: 

        SteeringEstimator() {
            _steering_angle_degs = 0.0;
        }

        void update(double steering_analog_meas);

        double get_steering_angle_degs(); 

        double get_steering_angle_rads();

    private: 

        mutable std::mutex _steering_mutex;
        double _steering_angle_degs;

        // Maps raw analog steering measurements -> steering angle in degrees

        InterpolatingTable steer_analog_to_degs{{
            {2040.0, -20.86}, {1129.0, 0.0}, {462.0, 20.11}
        }};

};

} // namespace estimation