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

constexpr double QP_M_B = 276.7;
constexpr double QP_CG_Z = 0.29;
constexpr double QP_WHEELBASE = 1.53;
constexpr double QP_TRACK_WIDTH = 1.2;
constexpr double QP_WHEEL_RADIUS = 0.6767676767; // TODO Fix this
constexpr double QP_MAX_TORQUE = 21.0; 
constexpr double QP_MAX_BRAKING_TORQUE = 10.0;
constexpr double QP_FRICTION_COEFF = 1.0; // TODO bruh

using osqp::OsqpInstance;
using osqp::OsqpSettings;
using osqp::OsqpSolver;

namespace control {
    class BasicQP : public Controller<core::ControllerOutput, core::VehicleState> {
    public:
        struct config {
            double kp_yaw_rate; // Proportional gain for yaw rate
            double ki_yaw_rate; // Integral gain for yaw rate
            double kd_yaw_rate; // Derivative gain for yaw rate
            double alpha; // Tractive force
            double lambda; // Regularization factor
            double omega; // Yaw rate
            int dt_rate_hz; // TODO don't use this calculate it
        };

        BasicQP() {}

        float get_dt_sec() override { 
            return (double) 1.0 / _config.dt_rate_hz;
        }

        bool init();

        core::ControllerOutput step_controller(const VehicleState &in) override;

    private:

        void _handle_param_updates(const std::unordered_map<std::string, DBParam> &new_param_map);

        double _pid_update(const VehicleState &in, double dt);

        Eigen::Vector4d _x_prev = Eigen::Vector4d::Zero();

        OsqpInstance _qp_instance;
        OsqpSettings _qp_settings;
        OsqpSolver _qp_solver;
        
        std::mutex _config_mutex;
        config _config{};
    };
}