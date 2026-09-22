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

constexpr double FZ_FL_STATIC = 710.1;
constexpr double FZ_FR_STATIC = 720.1;
constexpr double FZ_RL_STATIC = 660.1;
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
            {724, 228.64}, {1162, 465.16}, {1436, 629.61}, {1554, 706.19},
            {2120, 1041.73}, {2285, 1149.06}, {2647, 1526.75}
        }};

        InterpolatingTable fr_load_cell_to_fz{{
            {596, 241.57}, {985, 462.48}, {1250, 617.02}, {1430, 729.90},
            {2000, 1076.11}, {2160, 1177.57}, {2630, 1545.03}
        }};

        InterpolatingTable rl_load_cell_to_fz{{
            {507, 181.43}, {1033, 509.29}, {1168, 596.98}, {1283, 669.91},
            {1762, 971.18}, {2080, 1166.65}, {2400, 1375.90}
        }};

        InterpolatingTable rr_load_cell_to_fz{{
            {466, 186.68}, {996, 507.09}, {1148, 606.73}, {1220, 653.29},
            {1697, 956.59}, {2060, 1187.00}, {2580, 1474.14}
        }};
};

}