#pragma once

#include <Controller.hpp>
#include <StateTracker.hpp>
#include <Literals.hpp>
#include <FoxgloveServer.hpp>
#include <hytech_msgs.pb.h>
#include <utility>
#include <mutex>
#include "osqp++.h"
#include <Eigen/Sparse>
#include "Telemetry.hpp" 

constexpr int QP_NUM_VARS = 4;
constexpr int QP_NUM_CONSTRAINTS = 9;

constexpr double QP_M_B = 276.7;
constexpr double QP_CG_Z = 0.29;
constexpr double QP_WHEELBASE = 1.53;
constexpr double QP_TRACK_WIDTH = 1.2;
constexpr double QP_WHEEL_RADIUS = 0.203; // TODO Fix this
constexpr double QP_MAX_TORQUE = 21.0; 
constexpr double QP_MAX_BRAKING_TORQUE = 10.0;
constexpr double QP_GR = 11.83; // 11.83:1 reduction

namespace control {

class BasicQP : public Controller<core::ControllerOutput, core::VehicleState> {

    public:

        /**
         * Configuration structure for the BasicQP controller.
         * Contains PID gains for yaw rate control, tractive force and regularization factors,
         * yaw rate, and the update rate in Hz.
         */
        struct config {
            double yaw_kp; // Proportional gain for yaw rate
            double yaw_ki; // Integral gain for yaw rate
            double yaw_kd; // Derivative gain for yaw rate
            double alpha; // Tractive force
            double lambda; // Regularization factor
            double omega; // Yaw rate
            double dt_rate_hz; // TODO don't use this calculate it
            double mu;
            bool p_dirty = true;
        };

        /**
         * Constructor for the BasicQP controller.
         */
        BasicQP() {}

        /**
         * Returns the controller's update period in seconds.
         */
        float get_dt_sec() override { 
            return (double) 1.0 / _config.dt_rate_hz;
        }

        /**
         * Initializes the controller and prepares it for operation.
         * @return True if initialization is successful, false otherwise.
         */
        bool init();

        /**
         * Updates the objective vector and constraint bounds.
         * Re-runs a solve
         * @param in The current vehicle state.
         * @return The controller output after the step.  
         */
        core::ControllerOutput step_controller(const VehicleState &in) override;

    private:

        void _handle_param_updates(const std::unordered_map<std::string, DBParam> &new_param_map);
        double _pid_update(const VehicleState &in);

        void _build_constraint_matrix();
        void _build_objective_matrix();

        Eigen::Matrix<double, QP_NUM_VARS, QP_NUM_VARS> _P;
        Eigen::Matrix<double, QP_NUM_CONSTRAINTS, QP_NUM_VARS> _A;
        Eigen::Matrix<double, QP_NUM_VARS, 1> _q;
        Eigen::Matrix<double, QP_NUM_CONSTRAINTS, 1> _l;
        Eigen::Matrix<double, QP_NUM_CONSTRAINTS, 1> _u;

        Eigen::Vector4d _x_prev = Eigen::Vector4d::Zero();

        osqp::OsqpInstance _qp_instance;
        osqp::OsqpSettings _qp_settings;
        osqp::OsqpSolver _qp_solver;
        
        std::mutex _config_mutex;
        config _config{};
    };
}