#include "BasicQP.hpp"
#include "SimplePowerLimiter.hpp"
#include <variant>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <FoxgloveServer.hpp>
#include <cmath>


void control::BasicQP::_handle_param_updates(const std::unordered_map<std::string, DBParam> &new_param_map) {
    if (auto v = process_param_update<float>(new_param_map, "basicqp/omega")) {
        std::unique_lock lk(_config_mutex);
        _config.omega = v.value();
        _config.p_dirty = true; // Tells the solver to update the objective matrix
    }

    if (auto v = process_param_update<float>(new_param_map, "basicqp/alpha")) {
        std::unique_lock lk(_config_mutex);
        _config.alpha = v.value();
    }

    if (auto v = process_param_update<float>(new_param_map, "basicqp/lambda")) {
        std::unique_lock lk(_config_mutex);
        _config.lambda = v.value();
        _config.p_dirty = true; // Tells the solver to update the objective matrix
    }

    if (auto v = process_param_update<float>(new_param_map, "basicqp/gamma")) {
        std::unique_lock lk(_config_mutex);
        _config.gamma = v.value();
        _config.p_dirty = true; // Tells the solver to update the objective matrix
    }

    if (auto v = process_param_update<float>(new_param_map, "basicqp/dt_rate_hz")) {
        std::unique_lock lk(_config_mutex);
        _config.dt_rate_hz = v.value();
    }

    if (auto v = process_param_update<float>(new_param_map, "basicqp/yaw_kp")) {
        std::unique_lock lk(_config_mutex);
        _config.yaw_kp = v.value();
    }

    if (auto v = process_param_update<float>(new_param_map, "basicqp/yaw_ki")) {
        std::unique_lock lk(_config_mutex);
        _config.yaw_ki = v.value();
    }

    if (auto v = process_param_update<float>(new_param_map, "basicqp/yaw_kd")) {
        std::unique_lock lk(_config_mutex);
        _config.yaw_kd = v.value();
    }

    if (auto v = process_param_update<float>(new_param_map, "basicqp/mu")) {
        std::unique_lock lk(_config_mutex);
        _config.mu = v.value();
    }
}

void control::BasicQP::_build_objective_matrix() {
    std::unique_lock lk(_config_mutex);
    double omega = _config.omega;
    double lambda = _config.lambda;
    double gamma = _config.gamma;

    static double b = QP_TRACK_WIDTH / 2.0;
    static Eigen::Matrix<double, 4, 1> c;
    c <<
        -b / QP_WHEEL_RADIUS, b / QP_WHEEL_RADIUS, -b / QP_WHEEL_RADIUS, b / QP_WHEEL_RADIUS;
    
    _P = 2 * omega * (c * c.transpose());
    _P += 2 * lambda * Eigen::Matrix<double, 4, 4>::Identity();
    _P += 2 * gamma * Eigen::Matrix<double, 4, 4>::Identity();
}

void control::BasicQP::_build_constraint_matrix() {
    _A.row(0) << 1, 1, 1, 1;
    _A.row(1) << 1, 0, 0, 0;
    _A.row(2) << 0, 1, 0, 0;
    _A.row(3) << 0, 0, 1, 0;
    _A.row(4) << 0, 0, 0, 1;
    _A.row(5) << 1, 0, 0, 0;
    _A.row(6) << 0, 1, 0, 0;
    _A.row(7) << 0, 0, 1, 0;
    _A.row(8) << 0, 0, 0, 1;
}

double control::BasicQP::_pid_update(const VehicleState &in, std::shared_ptr<hytech_msgs::QPAllocator> qp_allocator_msg)
{
    std::unique_lock lk(_config_mutex);
    double dt = 1.0 / _config.dt_rate_hz;

    static double yaw_rate_integral_error = 0.0;
    static double previous_yaw_rate_error = 0.0;

    double target_yaw_rate_ref = (in.current_body_vel_ms.x / QP_WHEELBASE) * std::tan(in.steering_angle_deg * M_PI / 180.0);

    double yaw_rate_error = target_yaw_rate_ref - in.current_angular_rate_rads.z;
    yaw_rate_integral_error += yaw_rate_error * dt;
    double yaw_rate_derivative_error = (yaw_rate_error - previous_yaw_rate_error) / dt;
    previous_yaw_rate_error = yaw_rate_error;

    double des_yaw_moment = _config.yaw_kp * yaw_rate_error +
                                     _config.yaw_ki * yaw_rate_integral_error +
                                     _config.yaw_kd * yaw_rate_derivative_error;

    
    qp_allocator_msg->set_yaw_rate_reference(target_yaw_rate_ref);
    qp_allocator_msg->set_des_mz(des_yaw_moment);
    qp_allocator_msg->set_yaw_rate_meas(in.current_angular_rate_rads.z);

    return des_yaw_moment;
}

bool control::BasicQP::init()
{

    // Parameter and config initialization 
    auto opt_omega = FoxgloveServer::instance().get_param<float>("basicqp/omega");
    auto opt_alpha = FoxgloveServer::instance().get_param<float>("basicqp/alpha");
    auto opt_lambda = FoxgloveServer::instance().get_param<float>("basicqp/lambda");
    auto opt_gamma = FoxgloveServer::instance().get_param<float>("basicqp/gamma");
    auto opt_dt_rate_hz = FoxgloveServer::instance().get_param<float>("basicqp/dt_rate_hz");
    auto opt_yaw_kp = FoxgloveServer::instance().get_param<float>("basicqp/yaw_kp");
    auto opt_yaw_ki = FoxgloveServer::instance().get_param<float>("basicqp/yaw_ki");
    auto opt_yaw_kd = FoxgloveServer::instance().get_param<float>("basicqp/yaw_kd");
    auto opt_mu = FoxgloveServer::instance().get_param<float>("basicqp/mu");

    bool all_loaded = true;
    if (!opt_omega)        { spdlog::error("Missing param: basicqp/omega");        all_loaded = false; }
    if (!opt_alpha)        { spdlog::error("Missing param: basicqp/alpha");        all_loaded = false; }
    if (!opt_lambda)       { spdlog::error("Missing param: basicqp/lambda");       all_loaded = false; }
    if (!opt_gamma)        { spdlog::error("Missing param: basicqp/gamma");        all_loaded = false; }
    if (!opt_dt_rate_hz)   { spdlog::error("Missing param: basicqp/dt_rate_hz");   all_loaded = false; }
    if (!opt_yaw_kp)       { spdlog::error("Missing param: basicqp/yaw_kp");       all_loaded = false; }
    if (!opt_yaw_ki)       { spdlog::error("Missing param: basicqp/yaw_ki");       all_loaded = false; }
    if (!opt_yaw_kd)       { spdlog::error("Missing param: basicqp/yaw_kd");       all_loaded = false; }
    if (!opt_mu)           { spdlog::error("Missing param: basicqp/mu");           all_loaded = false; }

    if (!all_loaded) {
        spdlog::error("Couldn't load all params for the qp controller");
        return false;
    }

    _config.omega = opt_omega.value();
    _config.alpha = opt_alpha.value();
    _config.lambda = opt_lambda.value();
    _config.gamma = opt_gamma.value();
    _config.dt_rate_hz = opt_dt_rate_hz.value();
    _config.yaw_kp = opt_yaw_kp.value();
    _config.yaw_ki = opt_yaw_ki.value();
    _config.yaw_kd = opt_yaw_kd.value();
    _config.mu = opt_mu.value();

    FoxgloveServer::instance().register_param_callback(std::bind(&control::BasicQP::_handle_param_updates, this, std::placeholders::_1));

    // QP solver initialization
    _qp_settings.verbose = false;
    _qp_settings.warm_start = true;

    _build_objective_matrix();
    _build_constraint_matrix();

    _q.setZero();
    _l.setZero();
    _u.setZero();

    Eigen::SparseMatrix<double> P_sparse = _P.sparseView();
    Eigen::SparseMatrix<double> A_sparse = _A.sparseView();

    P_sparse.makeCompressed();
    A_sparse.makeCompressed();

    _qp_instance.objective_matrix = P_sparse;
    _qp_instance.objective_vector = _q;
    _qp_instance.constraint_matrix = A_sparse;
    _qp_instance.lower_bounds = _l;
    _qp_instance.upper_bounds = _u;

    auto status = _qp_solver.Init(_qp_instance, _qp_settings); 
    if (!status.ok()) {
        spdlog::error("Failed to initialize qp solver");
        return false;
    }

    return true;
}

ControllerOutput control::BasicQP::step_controller(const VehicleState &in)
{   

    std::shared_ptr<hytech_msgs::QPAllocator> qp_allocator_msg = std::make_shared<hytech_msgs::QPAllocator>();

    // Yaw rate objective vector construction
    double des_yaw_moment = _pid_update(in, qp_allocator_msg);

    static double b = QP_TRACK_WIDTH / 2.0;

    double omega, alpha, lambda, gamma, mu; 
    bool p_dirty;
    {
        std::unique_lock<std::mutex> lock(_config_mutex);
        omega = _config.omega;
        alpha = _config.alpha;
        lambda = _config.lambda;
        gamma = _config.gamma;
        mu = _config.mu;
        p_dirty = _config.p_dirty;
        _config.p_dirty = false;
    }

    double accel = in.input.requested_accel;
    double brake = in.input.requested_brake;
    double intent = accel - brake;
    
    static Eigen::Matrix<double, 4, 1> c;
    c <<
        -b / QP_WHEEL_RADIUS, b / QP_WHEEL_RADIUS, -b / QP_WHEEL_RADIUS, b / QP_WHEEL_RADIUS;
    
    _q = -2 * omega * des_yaw_moment * c;

    if (intent > 0.0) {
        _q += -alpha * Eigen::Matrix<double, 4, 1>::Ones(); 
    } else {
        _q += alpha * Eigen::Matrix<double, 4, 1>::Ones();
    }
    _q += -2 * lambda * _x_prev;

    double fz_total = in.fz_estimates.FL + in.fz_estimates.FR + in.fz_estimates.RL + in.fz_estimates.RR;
    double requested_total_torque = QP_MAX_TORQUE * 4.0 * intent * QP_GR;

    if (fz_total > 0.0) {
        Eigen::Matrix<double, 4, 1> Tref;
        Tref <<
            requested_total_torque * in.fz_estimates.FL / fz_total,
            requested_total_torque * in.fz_estimates.FR / fz_total,
            requested_total_torque * in.fz_estimates.RL / fz_total,
            requested_total_torque * in.fz_estimates.RR / fz_total;

        _q += -2 * gamma * Tref;
    }
    
    // Constraints

    SpeedControlOut type_set = {};
    ControllerOutput cmd_out = {};
    cmd_out.out = type_set;
    auto& speed_out = std::get<SpeedControlOut>(cmd_out.out);
    speed_out = {};

    if (intent < 0.0) {
        // For now don't run controller on braking
        speed_out.desired_rpms.FL = 0.0;
        speed_out.desired_rpms.FR = 0.0;
        speed_out.desired_rpms.RL = 0.0;
        speed_out.desired_rpms.RR = 0.0;

        speed_out.torque_lim_nm.FL = QP_MAX_BRAKING_TORQUE * brake;
        speed_out.torque_lim_nm.FR = QP_MAX_BRAKING_TORQUE * brake;
        speed_out.torque_lim_nm.RL = QP_MAX_BRAKING_TORQUE * brake;
        speed_out.torque_lim_nm.RR = QP_MAX_BRAKING_TORQUE * brake;
    }
    

    // Total torque
    double max_total_torque = std::fabs((QP_MAX_TORQUE * 4.0) * intent * QP_GR);
    if (intent > 0.01) {
        _l(0) = 0.0;
        _u(0) = max_total_torque;
    } else if (intent < -0.01) {
        _l(0) = -max_total_torque;
        _u(0) = 0.0;
    } else {
        _l(0) = 0.0;
        _u(0) = 0.0;
    }

    // Per-wheel motor torque constraints

    // FL
    static double qp_max_motor_torque = QP_MAX_TORQUE * QP_GR;
    _l(1) = -qp_max_motor_torque;
    _u(1) = qp_max_motor_torque;

    // FR
    _l(2) = -qp_max_motor_torque;
    _u(2) = qp_max_motor_torque;

    // RL
    _l(3) = -qp_max_motor_torque;
    _u(3) = qp_max_motor_torque;
    
    // RR
    _l(4) = -qp_max_motor_torque;
    _u(4) = qp_max_motor_torque;

    // Per-wheel additional traction constraints 

    // FL
    double fl_max_traction_torque = mu * in.fz_estimates.FL * QP_WHEEL_RADIUS;
    _l(5) = -fl_max_traction_torque;
    _u(5) = fl_max_traction_torque;

    // FR
    double fr_max_traction_torque = mu * in.fz_estimates.FR * QP_WHEEL_RADIUS;
    _l(6) = -fr_max_traction_torque;
    _u(6) = fr_max_traction_torque;

    // RL
    double rl_max_traction_torque = mu * in.fz_estimates.RL * QP_WHEEL_RADIUS;
    _l(7) = -rl_max_traction_torque;
    _u(7) = rl_max_traction_torque;

    // RR
    double rr_max_traction_torque = mu * in.fz_estimates.RR * QP_WHEEL_RADIUS;
    _l(8) = -rr_max_traction_torque;
    _u(8) = rr_max_traction_torque;

    // Solve
    _qp_solver.SetObjectiveVector(_q);
    _qp_solver.SetBounds(_l, _u);

    if (p_dirty) {
        _build_objective_matrix();
        Eigen::SparseMatrix<double> P_sparse = _P.sparseView();
        P_sparse.makeCompressed();
        _qp_solver.UpdateObjectiveMatrix(P_sparse);
    }

    auto solve_status = _qp_solver.Solve();
    Eigen::VectorXd solution = _qp_solver.primal_solution();

    speed_out.desired_rpms.FL = 20000;
    speed_out.desired_rpms.FR = 20000;
    speed_out.desired_rpms.RL = 20000;
    speed_out.desired_rpms.RR = 20000;

    double fl_torque = solution(0) / QP_GR;
    double fr_torque = solution(1) / QP_GR;
    double rl_torque = solution(2) / QP_GR;
    double rr_torque = solution(3) / QP_GR;

    speed_out.torque_lim_nm.FL = fl_torque;
    speed_out.torque_lim_nm.FR = fr_torque;
    speed_out.torque_lim_nm.RL = rl_torque;
    speed_out.torque_lim_nm.RR = rr_torque;

    qp_allocator_msg->set_final_torque_fl(fl_torque);
    qp_allocator_msg->set_final_torque_fr(fr_torque);
    qp_allocator_msg->set_final_torque_rl(rl_torque);
    qp_allocator_msg->set_final_torque_rr(rr_torque);

    // Put these in terms of motor torques for better understanding
    qp_allocator_msg->set_tot_torque_constraint(max_total_torque / QP_GR);
    qp_allocator_msg->set_amk_torque_constraint_fl(qp_max_motor_torque / QP_GR);
    qp_allocator_msg->set_amk_torque_constraint_fr(qp_max_motor_torque / QP_GR);
    qp_allocator_msg->set_amk_torque_constraint_rl(qp_max_motor_torque / QP_GR);
    qp_allocator_msg->set_amk_torque_constraint_rr(qp_max_motor_torque / QP_GR);

    qp_allocator_msg->set_traction_torque_constraint_fl(fl_max_traction_torque / QP_GR);
    qp_allocator_msg->set_traction_torque_constraint_fr(fr_max_traction_torque / QP_GR);
    qp_allocator_msg->set_traction_torque_constraint_rl(rl_max_traction_torque / QP_GR);
    qp_allocator_msg->set_traction_torque_constraint_rr(rr_max_traction_torque / QP_GR);

    core::log(qp_allocator_msg);

    _x_prev = solution;

    ControllerOutput out; 
    return out;
}