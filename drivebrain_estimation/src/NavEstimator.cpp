#include "NavEstimator.h"

using namespace estimation; 

NavEstimator::NavEstimator() {
    _nav_estimator_state = NAV_ESTIMATOR_STATE_INITIALIZING;

    _state.setZero();
    _P.setIdentity();
    _F.setIdentity();

    _Q.setZero();

    _Q(PN_INDEX, PN_INDEX) = 1e-6;
    _Q(PE_INDEX, PE_INDEX) = 1e-6;
    _Q(YAW_INDEX, YAW_INDEX) = 1e-6;
    _Q(VX_INDEX, VX_INDEX) = 1e-3;
    _Q(VY_INDEX, VY_INDEX) = 1e-3;
    _Q(ALPHA_INDEX, ALPHA_INDEX) = 1e-6;
    _Q(BX_INDEX, BX_INDEX) = 1e-6;
    _Q(BY_INDEX, BY_INDEX) = 1e-6;
    _Q(BG_INDEX, BG_INDEX) = 1e-6;
}

bool NavEstimator::initialize(double ax, double ay, double r) {

    if (_nav_estimator_state == NAV_ESTIMATOR_STATE_RUNNING) {
        return true; 
    }

    static double ax_sum = 0.0; 
    static int ax_count = 0; 
    
    static double ay_sum = 0.0; 
    static int ay_count = 0; 

    static double r_sum = 0.0; 
    static int r_count = 0; 

    ax_sum += ax;
    ax_count++;

    ay_sum += ay;
    ay_count++;

    r_sum += r;
    r_count++;

    if (ax_count >= MIN_INITIALIZATION_SAMPLES && ay_count >= MIN_INITIALIZATION_SAMPLES && r_count >= MIN_INITIALIZATION_SAMPLES) {
        double ax_avg = ax_sum / ax_count; 
        double ay_avg = ay_sum / ay_count; 
        double r_avg = r_sum / r_count;
        
        _state(BX_INDEX) = ax_avg;
        _state(BY_INDEX) = ay_avg;
        _state(BG_INDEX) = r_avg;

        _nav_estimator_state = NAV_ESTIMATOR_STATE_RUNNING;
        spdlog::info("Nav Estimator finished initialization.");
        return true;
    }

    return false;
}

void NavEstimator::predict(double ax, double ay, double r, double dt) {
    if (dt < 1e-6 || dt > 0.1) {
        spdlog::warn("Invalid dt value during prediction: {}", dt);
        return;
    }

    if (_nav_estimator_state != NAV_ESTIMATOR_STATE_RUNNING) {
        spdlog::warn("Navigation estimator is not initialized during prediction.");
        return;
    }

    std::unique_lock lock(_kf_mutex);

    const double vx = _state(VX_INDEX);
    const double vy = _state(VY_INDEX);

    const double ax_corr = ax - _state(BX_INDEX);
    const double ay_corr = ay - _state(BY_INDEX);
    const double r_corr = r - _state(BG_INDEX);

    const double theta = _state(YAW_INDEX) + _state(ALPHA_INDEX);
    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    _state(PN_INDEX) += (vx * ct - vy * st) * dt;
    _state(PE_INDEX) += (vx * st + vy * ct) * dt;
    _state(YAW_INDEX) += r_corr * dt;
    _state(VX_INDEX) += (ax_corr + r_corr * vy) * dt;
    _state(VY_INDEX) += (ay_corr - r_corr * vx) * dt;

    // WRAP ANGLES

    _F.setIdentity();

    _F(PN_INDEX, YAW_INDEX) = -dt * st * vx - dt * ct * vy;
    _F(PN_INDEX, VX_INDEX) = dt * ct;
    _F(PN_INDEX, VY_INDEX) = -dt * st;
    _F(PN_INDEX, ALPHA_INDEX) = -dt * st * vx - dt * ct * vy;

    _F(PE_INDEX, YAW_INDEX) = dt * ct * vx - dt * st * vy;
    _F(PE_INDEX, VX_INDEX) = dt * st;
    _F(PE_INDEX, VY_INDEX) = dt * ct;
    _F(PE_INDEX, ALPHA_INDEX) = dt * ct * vx - dt * st * vy;

    _F(YAW_INDEX, BG_INDEX) = -dt;

    _F(VX_INDEX, VY_INDEX) = r_corr * dt;
    _F(VX_INDEX, BG_INDEX) = -vy * dt;
    _F(VX_INDEX, BX_INDEX) = -dt;

    _F(VY_INDEX, VX_INDEX) = -r_corr * dt;
    _F(VY_INDEX, BG_INDEX) = vx * dt;
    _F(VY_INDEX, BY_INDEX) = -dt;

    _P = _F * _P * _F.transpose() + _Q;

    // Covariance safety
    _P = 0.5 * (_P + _P.transpose());
    _P.diagonal() = _P.diagonal().cwiseMax(1e-9);
    _P.diagonal() = _P.diagonal().cwiseMin(1e6);
}

void NavEstimator::zero_velocity_update() {
    if (_nav_estimator_state != NAV_ESTIMATOR_STATE_RUNNING) {
        spdlog::warn("Navigation estimator not intialized during zero-velocity update.");
        return;
    }
    
    std::unique_lock lock(_kf_mutex);

    Eigen::Matrix<double, NAV_ESTIMATOR_ZERO_VEL_UPDATE_SIZE, 1> z;
    z.setZero(); // Zero velocity measurement

    static Eigen::Matrix<double, NAV_ESTIMATOR_ZERO_VEL_UPDATE_SIZE, NAV_ESTIMATOR_STATE_SIZE> H;
    H <<
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0;
    
    static Eigen::Matrix<double, NAV_ESTIMATOR_ZERO_VEL_UPDATE_SIZE, NAV_ESTIMATOR_ZERO_VEL_UPDATE_SIZE> R;
    // TODO: Tune this as needed
    R <<
        1e-3, 0,
        0, 1e-3;
    
    Eigen::Matrix<double, NAV_ESTIMATOR_ZERO_VEL_UPDATE_SIZE, 1> y = z - H * _state;
    Eigen::Matrix<double, NAV_ESTIMATOR_ZERO_VEL_UPDATE_SIZE, NAV_ESTIMATOR_ZERO_VEL_UPDATE_SIZE> S = H * _P * H.transpose() + R;
    Eigen::Matrix<double, NAV_ESTIMATOR_STATE_SIZE, NAV_ESTIMATOR_ZERO_VEL_UPDATE_SIZE> K = _P * H.transpose() * S.inverse();
    _state += K * y;
    _P = (Eigen::Matrix<double, NAV_ESTIMATOR_STATE_SIZE, NAV_ESTIMATOR_STATE_SIZE>::Identity() - K * H) * _P;
    
    // Covariance safety
    _P = 0.5 * (_P + _P.transpose());
    _P.diagonal() = _P.diagonal().cwiseMax(1e-9);
    _P.diagonal() = _P.diagonal().cwiseMin(1e6);
}