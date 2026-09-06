#include "NavEstimator.h"

using namespace estimation; 

NavEstimator::NavEstimator() {
    _nav_estimator_state = NAV_ESTIMATOR_STATE_UNINITIALIZED;
    
    


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