#pragma once
/** 
 * Estimates the normal force acting on each wheel
 * Uses an LKF with the state estimated being [dFz_fl, dFz_fr, dFz_rl, dFz_rr] where dFz is the difference between the actual normal force and the static normal force
 */
#include <Eigen/Dense>
#include <mutex>
#include <FoxgloveServer.hpp>  
#include "InterpolatingTable.h"

// dFz_fl, dFz_fr, dFz_rl, dFz_rr
constexpr int FZ_STATE_SIZE = 4;

// ax, ay
constexpr int FZ_CONTROL_INPUT_SIZE = 2;

// dFz_fl, dFz_fr, dFz_rl, dFz_rr
constexpr int FZ_MEASUREMENT_SIZE = 4;

constexpr double FZ_M_B = 276.7;
constexpr double FZ_CG_Z = 0.29;
constexpr double FZ_WHEELBASE = 1.53;
constexpr double FZ_TRACK_WIDTH = 1.2;

constexpr double FZ_FL_STATIC = 707.1;
constexpr double FZ_FR_STATIC = 707.1;
constexpr double FZ_RL_STATIC = 650.1;
constexpr double FZ_RR_STATIC = 650.1;

using namespace core;

namespace estimation {

typedef Eigen::Matrix<double, FZ_STATE_SIZE, 1> fz_state_vector; // x
typedef Eigen::Matrix<double, FZ_STATE_SIZE, FZ_STATE_SIZE> fz_state_covariance; // Q
typedef Eigen::Matrix<double, FZ_MEASUREMENT_SIZE, FZ_MEASUREMENT_SIZE> fz_measurement_covariance; // R
typedef Eigen::Matrix<double, FZ_MEASUREMENT_SIZE, FZ_STATE_SIZE> fz_measurement_matrix; // H 
typedef Eigen::Matrix<double, FZ_STATE_SIZE, FZ_STATE_SIZE> fz_process_model_matrix; // A
typedef Eigen::Matrix<double, FZ_STATE_SIZE, FZ_CONTROL_INPUT_SIZE> fz_control_input_matrix; // B
typedef Eigen::Matrix<double, FZ_CONTROL_INPUT_SIZE, 1> fz_control_input_vector; // u
typedef Eigen::Matrix<double, FZ_MEASUREMENT_SIZE, 1> fz_measurement_vector; // z
typedef Eigen::Matrix<double, FZ_STATE_SIZE, 1> fz_estimates; 


class FzEstimator {

    public: 

        void _handle_param_updates(const std::unordered_map<std::string, core::DBParam> &new_param_map);
        
        FzEstimator();

        void predict(const fz_control_input_vector& u);

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
        mutable std::mutex _kf_mutex;

        // Maps raw analog load cell measurements --> normal force (N) on each wheel

        InterpolatingTable fl_load_cell_to_fz{{
            {673, 422.55}, {924, 615.63}, {982, 701.76}, {1019, 705.49},
            {1098, 720.56}, {1264, 974.46}, {1401, 1029.04}, {1562, 1104.69}
        }};
        
        InterpolatingTable fr_load_cell_to_fz{{
            {385, 304.96}, {524, 358.02}, {670, 459.96}, {778, 698.43},
            {926, 707.26}, {950, 747.38}, {1210, 1007.82}
        }};
        
        InterpolatingTable rl_load_cell_to_fz{{
            {504, 343.19}, {768, 590.51}, {834, 642.06}, {887, 649.73},
            {930, 655.22}, {965, 683.50}, {1139, 902.22}, {1293, 993.22}, {1416, 1062.18}
        }};
        
        InterpolatingTable rr_load_cell_to_fz{{
            {289, 238.29}, {438, 322.96}, {576, 382.96}, {746, 650.29},
            {862, 650.77}, {961, 755.30}, {1229, 934.27}
        }};

};

}