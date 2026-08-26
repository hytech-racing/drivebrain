#include "FzEstimator.h"

using namespace estimation;

FzEstimator::FzEstimator(fz_state_covariance Q, fz_measurement_covariance R) {
    _state.setZero();
    _A.setZero();
    _P.setIdentity();
    _H.setIdentity();
    _Q = Q;
    _R = R;

    _B <<
        -0.5 * (m_b * cg_z) / l, -0.5 * (m_b * cg_z) / tr,
        -0.5 * (m_b * cg_z) / l, 0.5 * (m_b * cg_z) / tr,
        0.5 * (m_b * cg_z) / l, -0.5 * (m_b * cg_z) / tr,
        0.5 * (m_b * cg_z) / l, 0.5 * (m_b * cg_z) / tr;

    _fz_static << fz_fl_static, fz_fr_static, fz_rl_static, fz_rr_static;
}

void FzEstimator::predict(const fz_control_input_matrix& u) {
    _state = _A * _state + _B * u;
    _P = _A * _P * _A.transpose() + _Q;
}

void FzEstimator::update(double load_cell_fl, double load_cell_fr, double load_cell_rl, double load_cell_rr) {
    // fz_measurement_vector y = z - _H * _state;
    // fz_measurement_covariance S = _H * _P * _H.transpose() + _R;
    // fz_state_covariance K = _P * _H.transpose() * S.inverse();
    // _state = _state + K * y;
    // _P = (fz_state_covariance::Identity() - K * _H) * _P;
            
    // _estimates = _state + _fz_static;
    // estimates = _estimates;
}

fz_estimates FzEstimator::getEstimates() const {
    return _estimates + fz_static;
}