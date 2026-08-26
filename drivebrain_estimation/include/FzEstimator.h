#pragma once
/** 
 * Estimates the normal force acting on each wheel
 * Uses an LKF with the state estimated being [dFz_fl, dFz_fr, dFz_rl, dFz_rr] where dFz is the difference between the actual normal force and the static normal force
 */
#include <Eigen/Dense>
#include "InterpolatingTable.h"

// dFz_fl, fFz_fr, dFz_rl, dFz_rr
#define FZ_STATE_SIZE 4

// ax, ay
#define FZ_CONTROL_INPUT_SIZE 2

// dFz_fl, dFz_fr, dFz_rl, dFz_rr
#define FZ_MEASUREMENT_SIZE 4

#define m_b 276.7
#define cg_z 0.29
#define l 1.53
#define tr 1.2

#define fz_fl_static 707.1
#define fz_fr_static 707.1
#define fz_rl_static 650.1
#define fz_rr_static 650.1

namespace estimation {

typedef Eigen::Matrix<double, FZ_STATE_SIZE, 1> fz_state_vector; // x
typedef Eigen::Matrix<double, FZ_STATE_SIZE, FZ_STATE_SIZE> fz_state_covariance; // Q
typedef Eigen::Matrix<double, FZ_MEASUREMENT_SIZE, FZ_MEASUREMENT_SIZE> fz_measurement_covariance; // R
typedef Eigen::Matrix<double, FZ_MEASUREMENT_SIZE, FZ_STATE_SIZE> fz_measurement_matrix; // H 
typedef Eigen::Matrix<double, FZ_STATE_SIZE, FZ_STATE_SIZE> fz_process_model_matrix; // A
typedef Eigen::Matrix<double, FZ_STATE_SIZE, FZ_CONTROL_INPUT_SIZE> fz_control_input_matrix; // B
typedef Eigen::Matrix<double, FZ_MEASUREMENT_SIZE, 1> fz_measurement_vector; // z
typedef Eigen::Matrix<double, FZ_STATE_SIZE, 1> fz_estimates; 


class FzEstimator {

    public: 
        
        FzEstimator(fz_state_covariance Q, fz_measurement_covariance R);

        void predict(const fz_control_input_matrix& u);

        void update(double load_cell_fl, double load_cell_fr, double load_cell_rl, double load_cell_rr);

        fz_estimates getEstimates() const;

    private: 

        fz_state_vector _state; 
        fz_state_covariance _P; 
        fz_process_model_matrix _A; 
        fz_control_input_matrix _B;
        fz_measurement_matrix _H; 
        fz_state_covariance _Q; 
        fz_measurement_covariance _R; 

        fz_estimates _fz_static;
        fz_estimates _estimates;

        // InterpolatingTable fl_load_cell_to_fz;
        // InterpolatingTable fr_load_cell_to_fz;
        // InterpolatingTable rl_load_cell_to_fz;
        // InterpolatingTable rr_load_cell_to_fz;
};

}