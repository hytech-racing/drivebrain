#include "FzEstimator.h"

using namespace estimation;

FzEstimator::FzEstimator() {
    // Set matricies and vectors
    _state.setZero();
    _A.setZero();
    _P.setIdentity(); // setting the prediction covariance matrix to identity assumes that every state variable has an initial variance of 1 and that no state errors are correlated with each other (default)
    _H.setIdentity();

    std::optional<float> Q_gain = FoxgloveServer::instance().get_param<float>("FzEstimator/Q_gain");
    std::optional<float> R_gain = FoxgloveServer::instance().get_param<float>("FzEstimator/R_gain");

    if (!Q_gain || !R_gain) {
        spdlog::error("FzEstimator: Q_gain or R_gain parameter not found");
        throw std::runtime_error("FzEstimator: Q_gain or R_gain parameter not found.");
    }

    _Q = fz_state_covariance::Identity() * Q_gain.value();
    _R = fz_measurement_covariance::Identity() * R_gain.value();

    _B <<
        -0.5 * (FZ_M_B * FZ_CG_Z) / FZ_WHEELBASE, -0.5 * (FZ_M_B * FZ_CG_Z) / FZ_TRACK_WIDTH,
        -0.5 * (FZ_M_B * FZ_CG_Z) / FZ_WHEELBASE, 0.5 * (FZ_M_B * FZ_CG_Z) / FZ_TRACK_WIDTH,
        0.5 * (FZ_M_B * FZ_CG_Z) / FZ_WHEELBASE, -0.5 * (FZ_M_B * FZ_CG_Z) / FZ_TRACK_WIDTH,
        0.5 * (FZ_M_B * FZ_CG_Z) / FZ_WHEELBASE, 0.5 * (FZ_M_B * FZ_CG_Z) / FZ_TRACK_WIDTH;

    _fz_static << FZ_FL_STATIC, FZ_FR_STATIC, FZ_RL_STATIC, FZ_RR_STATIC;

    core::FoxgloveServer::instance().register_param_callback(std::bind(&FzEstimator::_handle_param_updates, this, std::placeholders::_1));
}

void FzEstimator::_handle_param_updates(const std::unordered_map<std::string, core::DBParam> &new_param_map) {
    if (auto v = core::process_param_update<float>(new_param_map, "FzEstimator/Q_gain")) {
        std::unique_lock lk(_kf_mutex);
        _Q = fz_state_covariance::Identity() * (*v);
    }

    if (auto v = core::process_param_update<float>(new_param_map, "FzEstimator/R_gain")) {
        std::unique_lock lk(_kf_mutex);
        _R = fz_measurement_covariance::Identity() * (*v);
    }
}


void FzEstimator::predict(const fz_control_input_vector& u) {
    std::unique_lock lock(_kf_mutex);
    _state = (_A * _state) + (_B * u);
    _P = (_A * _P * _A.transpose()) + _Q;
}

void FzEstimator::update(double load_cell_fl, double load_cell_fr, double load_cell_rl, double load_cell_rr) {
    std::unique_lock lock(_kf_mutex);
    double fz_fl = fl_load_cell_to_fz.interpolate(load_cell_fl);
    double fz_fr = fr_load_cell_to_fz.interpolate(load_cell_fr);
    double fz_rl = rl_load_cell_to_fz.interpolate(load_cell_rl);
    double fz_rr = rr_load_cell_to_fz.interpolate(load_cell_rr);

    fz_measurement_vector z;
    z << 
        fz_fl - FZ_FL_STATIC, 
        fz_fr - FZ_FR_STATIC, 
        fz_rl - FZ_RL_STATIC, 
        fz_rr - FZ_RR_STATIC;

    fz_measurement_vector y = z - _H * _state;
    fz_measurement_covariance S = _H * _P * _H.transpose() + _R;
    // fz_state_covariance K = _P * _H.transpose() * S.inverse();

    /*
        LDLT computes the Cholesky decomposition of a matrix, which breaks it down into a lower triangular matrix and its transpose (A = LDL_transpose)
        The solver is able to compute the columns of the inverse using forward and backward substitution on each matrix in the decomposition.
        The identity matrix is used as an input to compute the inverse of the original matrix. Essentially telling the solver AX = I, solve for X (aka A^-1)
    */
    Eigen::LDLT<fz_measurement_covariance> ldlt(S); // First have to turn S into an LDLT decomposition --> becomes (LDL_transpose)X = B

    fz_measurement_covariance S_inverse = ldlt.solve(fz_measurement_covariance::Identity()); // solves for each column of the new matrix to get B (which is I in this case)
    fz_state_covariance K = _P * _H.transpose() * S_inverse; 

    _state = _state + K * y;
    _P = (fz_state_covariance::Identity() - K * _H) * _P;
}

fz_estimates FzEstimator::getEstimates() const {
    std::unique_lock lock(_kf_mutex);
    return _state + _fz_static;
}