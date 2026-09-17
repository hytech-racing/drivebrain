#include "BasicQP.hpp"
#include "SimplePowerLimiter.hpp"
#include <variant>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <FoxgloveServer.hpp>
#include <cmath>


void control::BasicQP::_handle_param_updates(const std::unordered_map<std::string, DBParam> &new_param_map) {

}

bool control::BasicQP::init()
{

    FoxgloveServer::instance().register_param_callback(std::bind(&control::BasicQP::_handle_param_updates, this, std::placeholders::_1));

    // QP solver initialization
    _qp_settings.verbose = false;
    _qp_settings.warm_start = true;

    auto status = _qp_solver.Init(_qp_instance, _qp_settings); 
    if (!status.ok()) {
        spdlog::error("Failed to initialize QP solver");
        return false;
    }

    return true;
}

ControllerOutput control::BasicQP::step_controller(const VehicleState &in)
{   
    double des_yaw_moment = _pid_update(in, 1.0 / _config.dt_rate_hz);

    static double b = QP_TRACK_WIDTH / 2.0;

    double omega = _config.omega;
    double alpha = _config.alpha;
    double lambda = _config.lambda;

    // Construct the QP matrix P based on the vehicle parameters and control gains
    Eigen::Matrix<double, 4, 4> P;
    Eigen::Matrix<double, 4, 1> c;
    c <<
        -b / QP_WHEEL_RADIUS, b / QP_WHEEL_RADIUS, -b / QP_WHEEL_RADIUS, b / QP_WHEEL_RADIUS;
    
    P = 2 * omega * (c * c.transpose());
    P += lambda * Eigen::Matrix<double, 4, 4>::Identity();

    // Construct the QP vector q based on the desired yaw moment and control gains
    Eigen::Matrix<double, 4, 1> q;
    q = -2 * omega * des_yaw_moment * c;
    q += -alpha * Eigen::Matrix<double, 4, 1>::Ones(); 
    q += -2 * lambda * _x_prev;

    // Constraints
    double accel = in.input.requested_accel;
    double brake = in.input.requested_brake;

    SpeedControlOut type_set = {};
    ControllerOutput cmd_out = {};
    cmd_out.out = type_set;
    auto& speed_out = std::get<SpeedControlOut>(cmd_out.out);
    speed_out = {};

    if (brake > 0.01) {
        // For now don't run controller on braking
        auto& speed_out = std::get<SpeedControlOut>(cmd_out.out);
        speed_out.desired_rpms.FL = 20000;
        speed_out.desired_rpms.FR = 20000;
        speed_out.desired_rpms.RL = 20000;
        speed_out.desired_rpms.RR = 20000;

        speed_out.torque_lim_nm.FL = -QP_MAX_BRAKING_TORQUE * brake;
        speed_out.torque_lim_nm.FR = -QP_MAX_BRAKING_TORQUE * brake;
        speed_out.torque_lim_nm.RL = -QP_MAX_BRAKING_TORQUE * brake;
        speed_out.torque_lim_nm.RR = -QP_MAX_BRAKING_TORQUE * brake;
        return cmd_out;
    }

    Eigen::Matrix<double, 9, 4> A;
    Eigen::Matrix<double, 9, 1> l;
    Eigen::Matrix<double, 9, 1> u;

    A.setZero();
    
    A.row(0) << 1, 1, 1, 1;

    // Total torque
    if (accel > 0.01) {
        l(0) = -QP_MAX_TORQUE * accel;
        u(0) = QP_MAX_TORQUE * accel;
    } else {
        l(0) = 0.0;
        u(0) = 0.0;
    }

    // Per-wheel motor torque constraints

    // FL
    A.row(1) << 1, 0, 0, 0;
    l(1) = -QP_MAX_TORQUE;
    u(1) = QP_MAX_TORQUE;

    // FR
    A.row(2) << 0, 1, 0, 0;
    l(2) = -QP_MAX_TORQUE;
    u(2) = QP_MAX_TORQUE;

    // RL
    A.row(3) << 0, 0, 1, 0;
    l(3) = -QP_MAX_TORQUE;
    u(3) = QP_MAX_TORQUE;
    
    // RR
    A.row(4) << 0, 0, 0, 1;
    l(4) = -QP_MAX_TORQUE;
    u(4) = QP_MAX_TORQUE;

    // Per-wheel additional traction constraints 

    // FL
    A.row(5) << 1, 0, 0, 0;
    l(5) = -QP_FRICTION_COEFF * in.fz_estimates.FL * QP_WHEEL_RADIUS;
    u(5) = QP_FRICTION_COEFF * in.fz_estimates.FL * QP_WHEEL_RADIUS;

    // FR
    A.row(6) << 0, 1, 0, 0;
    l(6) = -QP_FRICTION_COEFF * in.fz_estimates.FR * QP_WHEEL_RADIUS;
    u(6) = QP_FRICTION_COEFF * in.fz_estimates.FR * QP_WHEEL_RADIUS;

    // RL
    A.row(7) << 0, 0, 1, 0;
    l(7) = -QP_FRICTION_COEFF * in.fz_estimates.RL * QP_WHEEL_RADIUS;
    u(7) = QP_FRICTION_COEFF * in.fz_estimates.RL * QP_WHEEL_RADIUS;

    // RR
    A.row(8) << 0, 0, 0, 1;
    l(8) = -QP_FRICTION_COEFF * in.fz_estimates.RR * QP_WHEEL_RADIUS;
    u(8) = QP_FRICTION_COEFF * in.fz_estimates.RR * QP_WHEEL_RADIUS;

    // Solve

    Eigen::SparseMatrix<double> A_sparse = A.sparseView();
    Eigen::SparseMatrix<double> P_sparse = P.sparseView();

    A_sparse.makeCompressed();
    P_sparse.makeCompressed();

    _qp_instance.objective_matrix = P_sparse;
    _qp_instance.objective_vector = q;

    _qp_instance.constraint_matrix = A_sparse;
    _qp_instance.lower_bounds = l;
    _qp_instance.upper_bounds = u;

    auto solve_status = _qp_solver.Solve();
    Eigen::VectorXd solution = _qp_solver.primal_solution();

    std::cout << solution.transpose() << std::endl;

    _x_prev = solution;

    ControllerOutput out; 
    return out;

}

double control::BasicQP::_pid_update(const VehicleState &in, double dt)
{
    static double yaw_rate_integral_error = 0.0;
    static double previous_yaw_rate_error = 0.0;

    double target_yaw_rate_ref_rps = (in.current_body_vel_ms.x / QP_TRACK_WIDTH) * std::tan(in.steering_angle_deg * M_PI / 180.0);

    double yaw_rate_error = target_yaw_rate_ref_rps - in.current_angular_rate_rads.x; // TODO check that this should be the x component
    yaw_rate_integral_error += yaw_rate_error * (1.0 / _config.dt_rate_hz);
    double yaw_rate_derivative_error = (yaw_rate_error - previous_yaw_rate_error) * _config.dt_rate_hz;
    previous_yaw_rate_error = yaw_rate_error;

    double des_yaw_moment = _config.kp_yaw_rate * yaw_rate_error +
                                     _config.ki_yaw_rate * yaw_rate_integral_error +
                                     _config.kd_yaw_rate * yaw_rate_derivative_error;
    return des_yaw_moment;
}
