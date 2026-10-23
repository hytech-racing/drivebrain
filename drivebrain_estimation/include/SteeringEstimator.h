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

        void update(double steering_analog_meas, double steering_digital_meas);

        double get_steering_angle_degs(); 

        double get_steering_angle_rads();

        double get_column_angle_degs();

        // Converts a front wheel angle (degrees) to the steering column angle (degrees) the Kraken is commanded with
        static double wheel_degs_to_column_degs(double wheel_degs);

    private: 

        mutable std::mutex _steering_mutex;
        double _steering_angle_degs;
        double _column_angle_degs;

        // Maps raw analog steering measurements -> steering angle in degrees

        InterpolatingTable steer_analog_to_degs{{
            {2960.0, -20.86}, {2059.0, 0.0}, {1164.0, 20.11}
        }};

        // Constants for conversion form steering analog -> steering column pos
        static constexpr double COLUMN_OFFSET = 2092.0;
        static constexpr double COLUMN_SCALING = -0.09641;

};

} // namespace estimation