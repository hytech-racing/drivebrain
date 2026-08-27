#include "FzEstimator.h"

using namespace estimation;

FzEstimator::FzEstimator(fz_state_covariance Q, fz_measurement_covariance R) {
    // Set matricies and vectors
    _state.setZero();
    _A.setZero();
    _P.setIdentity();
    _H.setIdentity();
    _Q = Q;
    _R = R;

    _B <<
        -0.5 * (FZ_M_B * FZ_CG_Z) / FZ_WHEELBASE, -0.5 * (FZ_M_B * FZ_CG_Z) / FZ_TRACK_WIDTH,
        -0.5 * (FZ_M_B * FZ_CG_Z) / FZ_WHEELBASE, 0.5 * (FZ_M_B * FZ_CG_Z) / FZ_TRACK_WIDTH,
        0.5 * (FZ_M_B * FZ_CG_Z) / FZ_WHEELBASE, -0.5 * (FZ_M_B * FZ_CG_Z) / FZ_TRACK_WIDTH,
        0.5 * (FZ_M_B * FZ_CG_Z) / FZ_WHEELBASE, 0.5 * (FZ_M_B * FZ_CG_Z) / FZ_TRACK_WIDTH;

    _fz_static << FZ_FL_STATIC, FZ_FR_STATIC, FZ_RL_STATIC, FZ_RR_STATIC;

    // Initialize LUTs

}

void FzEstimator::predict(const fz_control_input_vector& u) {
    _state = (_A * _state) + (_B * u);
    _P = (_A * _P * _A.transpose()) + _Q;
}

void FzEstimator::update(double load_cell_fl, double load_cell_fr, double load_cell_rl, double load_cell_rr) {
    double fz_fl = fl_load_cell_to_fz.interpolate(load_cell_fl);
    double fz_fr = fr_load_cell_to_fz.interpolate(load_cell_fr);
    double fz_rl = rl_load_cell_to_fz.interpolate(load_cell_rl);
    double fz_rr = rr_load_cell_to_fz.interpolate(load_cell_rr);

    fz_measurement_vector z;
    z << FZ_FL_STATIC - fz_fl, FZ_FR_STATIC - fz_fr, FZ_RL_STATIC - fz_rl, FZ_RR_STATIC - fz_rr;

    fz_measurement_vector y = z - _H * _state;
    fz_measurement_covariance S = _H * _P * _H.transpose() + _R;
    fz_state_covariance K = _P * _H.transpose() * S.inverse();
    _state = _state + K * y;
    _P = (fz_state_covariance::Identity() - K * _H) * _P;
            
    _estimates = _state + _fz_static;
}

fz_estimates FzEstimator::getEstimates() const {
    return _estimates;
}