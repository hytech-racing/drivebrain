# pragma once
/**
 * Estimates the vehicle's position, orientation, velocity, IMU biases, and more 
 * via an extended Kalman filter. Individual updates depending on the sensor data available.
 */
#include <Eigen/Dense>

constexpr int MIN_INITIALIZATION_SAMPLES = 1000;
constexpr int PN_INDEX = 0;
constexpr int PE_INDEX = 1;
constexpr int YAW_INDEX = 2;
constexpr int VX_INDEX = 3;
constexpr int VY_INDEX = 4;
constexpr int ALPHA_INDEX = 5;
constexpr int BX_INDEX = 6;
constexpr int BY_INDEX = 7;
constexpr int BG_INDEX = 8;

constexpr int NAV_ESTIMATOR_STATE_SIZE = 9;


// Pn Pe yaw Vx Vy alpha bx by bg
typedef Eigen::Matrix<double, NAV_ESTIMATOR_STATE_SIZE, 1> nav_state_vector; // x
typedef Eigen::Matrix<double, NAV_ESTIMATOR_STATE_SIZE, NAV_ESTIMATOR_STATE_SIZE> nav_state_matrix; // P

namespace estimation {

/**
 * The state of the navigation estimator.
 */
enum NavEstimatorState {
    NAV_ESTIMATOR_STATE_INITIALIZING,
    NAV_ESTIMATOR_STATE_RUNNING
};

class NavEstimator {
    public: 

        /**
         * Initializes the navigation estimator object.
         */
        NavEstimator();

        /**
         * Initializes the navigation estimator. This should be called before any other methods at 400hz. 
         * Non-blocking
         * @param ax The measured longitudinal acceleration of the vehicle in m/s^2
         * @param ay The measured lateral acceleration of the vehicle in m/s^2
         * @param r The measured yaw rate of the vehicle in rad/s
         * @return true if the estimator is initialized, false otherwise
         */
        bool initialize(double ax, double ay, double r);

        /**
         * Predicts the next state of the navigation estimator based on the provided IMU measurements.
         * @param ax The measured longitudinal acceleration of the vehicle in m/s^2
         * @param ay The measured lateral acceleration of the vehicle in m/s^2
         * @param r The measured yaw rate of the vehicle in rad/s
         * @param dt The time step in seconds since the last prediction 
         */
        void predict(double ax, double ay, double r, double dt);

    private:

        NavEstimatorState _nav_estimator_state = NAV_ESTIMATOR_STATE_INITIALIZING;

        nav_state_vector _state; // x
        nav_state_matrix _P; // P
        nav_state_matrix _Q; // Q
        nav_state_matrix _R; // R
        nav_state_matrix _F; // F


        std::mutex _kf_mutex;

};

}