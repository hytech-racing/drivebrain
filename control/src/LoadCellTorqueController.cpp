#include "LoadCellTorqueController.hpp"
#include "SimplePowerLimiter.hpp"
#include <variant>
#include <spdlog/spdlog.h>
#include <algorithm>

void control::LoadCellTorqueController::_handle_param_updates(const std::unordered_map<std::string, DBParam> &new_param_map) {

    spdlog::info("Entering load cell torque controller parameter updates.");

    if (auto it = new_param_map.find("loadcelltorquecontroller/max_torque"); it != new_param_map.end()) {
        if (auto pval = std::get_if<float>(&it->second)) {
            std::unique_lock lk(_config_mutex);
            _config.max_torque = *pval;
        }
    }

    if (auto it = new_param_map.find("loadcelltorquecontroller/max_regen_torque"); it != new_param_map.end()) {
        if (auto pval = std::get_if<float>(&it->second)) {
            std::unique_lock lk(_config_mutex);
            _config.max_regen_torque = *pval;
        }
    }

    if (auto it = new_param_map.find("loadcelltorquecontroller/rear_torque_scale"); it != new_param_map.end()) {
        if (auto pval = std::get_if<float>(&it->second)) {
            std::unique_lock lk(_config_mutex);
            _config.rear_torque_scale = *pval;
        }
    }

    if (auto it = new_param_map.find("loadcelltorquecontroller/regen_torque_scale"); it != new_param_map.end()) {
        if (auto pval = std::get_if<float>(&it->second)) {
            std::unique_lock lk(_config_mutex);
            _config.regen_torque_scale = *pval;
        }
    }

    if (auto it = new_param_map.find("loadcelltorquecontroller/positive_speed_set"); it != new_param_map.end()) {
        if (auto pval = std::get_if<float>(&it->second)) {
            std::unique_lock lk(_config_mutex);
            _config.positive_speed_set = *pval;
        }
    }

    if (auto it = new_param_map.find("loadcelltorquecontroller/max_power_kw"); it != new_param_map.end()) {
        if (auto pval = std::get_if<float>(&it->second)) {
            std::unique_lock lk(_config_mutex);
            _config.max_power_kw = *pval;
        }
    }

    if (auto it = new_param_map.find("loadcelltorquecontroller/dt_rate_hz"); it != new_param_map.end()) {
        if (auto pval = std::get_if<float>(&it->second)) {
            std::unique_lock lk(_config_mutex);
            _config.dt_rate_hz = *pval;
        }
    }

    if (auto it = new_param_map.find("loadcelltorquecontroller/apply_vectoring_in_regen"); it != new_param_map.end()) {
        if (auto pval = std::get_if<float>(&it->second)) {
            std::unique_lock lk(_config_mutex);
            _config.apply_vectoring_in_regen = *pval;
        }
    }

    spdlog::info("Exiting load cell torque controller parameter updates.");
}

bool control::LoadCellTorqueController::init()
{
    auto opt_max_torque = FoxgloveServer::instance().get_param<float>("LoadCellTorqueController/max_torque");
    auto opt_max_regen_torque = FoxgloveServer::instance().get_param<float>("LoadCellTorqueController/max_regen_torque");
    auto opt_rear_torque_scale = FoxgloveServer::instance().get_param<float>("LoadCellTorqueController/rear_torque_scale");
    auto opt_regen_torque_scale = FoxgloveServer::instance().get_param<float>("LoadCellTorqueController/regen_torque_scale");
    auto opt_positive_speed_set = FoxgloveServer::instance().get_param<float>("LoadCellTorqueController/positive_speed_set");
    auto opt_max_power_kw = FoxgloveServer::instance().get_param<float>("LoadCellTorqueController/max_power_kw");
    auto opt_dt_rate_hz = FoxgloveServer::instance().get_param<float>("LoadCellTorqueController/dt_rate_hz");
    auto opt_apply_vectoring_in_regen = FoxgloveServer::instance().get_param<bool>("LoadCellTorqueController/apply_vectoring_in_regen");
    
    bool all_loaded = true;
    if (!opt_max_torque)              { spdlog::error("Missing param: LoadCellTorqueController/max_torque");              all_loaded = false; }
    if (!opt_max_regen_torque)        { spdlog::error("Missing param: LoadCellTorqueController/max_regen_torque");        all_loaded = false; }
    if (!opt_rear_torque_scale)       { spdlog::error("Missing param: LoadCellTorqueController/rear_torque_scale");       all_loaded = false; }
    if (!opt_regen_torque_scale)      { spdlog::error("Missing param: LoadCellTorqueController/regen_torque_scale");      all_loaded = false; }
    if (!opt_positive_speed_set)      { spdlog::error("Missing param: LoadCellTorqueController/positive_speed_set");      all_loaded = false; }
    if (!opt_max_power_kw)            { spdlog::error("Missing param: LoadCellTorqueController/max_power_kw");            all_loaded = false; }
    if (!opt_dt_rate_hz)              { spdlog::error("Missing param: LoadCellTorqueController/dt_rate_hz");              all_loaded = false; }
    if (!opt_apply_vectoring_in_regen){ spdlog::error("Missing param: LoadCellTorqueController/apply_vectoring_in_regen");all_loaded = false; }

    if (!all_loaded) {
        spdlog::error("Couldn't load all params for loadcell torque vectoring controller.");
        return false;
    }
    _config.max_torque               = opt_max_torque.value();
    _config.max_regen_torque         = opt_max_regen_torque.value();
    _config.rear_torque_scale        = opt_rear_torque_scale.value();
    _config.regen_torque_scale       = opt_regen_torque_scale.value();
    _config.positive_speed_set       = opt_positive_speed_set.value();
    _config.max_power_kw             = opt_max_power_kw.value();
    _config.dt_rate_hz               = opt_dt_rate_hz.value();
    _config.apply_vectoring_in_regen = opt_apply_vectoring_in_regen.value();

    FoxgloveServer::instance().register_param_callback(std::bind(&control::LoadCellTorqueController::_handle_param_updates, this, std::placeholders::_1));
    
    return true;
}

ControllerOutput control::LoadCellTorqueController::step_controller(const VehicleState &in)
{
    config cur_config;
    {
        std::unique_lock lk(_config_mutex);
        cur_config = _config;
    }

    // Both pedals are not pressed and no implausibility has been detected
    // accelRequest goes between 1.0 and -1.0
    float accelRequest = (in.input.requested_accel) - (in.input.requested_brake);

    veh_vec<float> current_rpms = in.current_rpms;

    torque_nm torqueRequest = {};

    SpeedControlOut type_set = {};
    ControllerOutput cmd_out = {};
    cmd_out.out = type_set;
    auto& speed_out = std::get<SpeedControlOut>(cmd_out.out);
    speed_out = {};
    speed_out.mcu_recv_millis = in.prev_MCU_recv_millis; // heartbeat TODO this isnt needed any more, prob should remove    
    
    float sum_normal = in.loadcells.FL + in.loadcells.FR + in.loadcells.RL+ in.loadcells.RR;
    if (accelRequest >= 0.0)
    {
        // Positive torque request
        torqueRequest = ((float)accelRequest) * cur_config.max_torque;
        
        float accel_torque_pool = accelRequest * cur_config.max_torque * 4; 
        
        
        auto max_rpm = cur_config.positive_speed_set * constants::METERS_PER_SECOND_TO_RPM;
        speed_out.desired_rpms.FL = 20000;
        speed_out.desired_rpms.FR = 20000;
        speed_out.desired_rpms.RL = 20000;
        speed_out.desired_rpms.RR = 20000;

        speed_out.torque_lim_nm.FL = ((2.0 - cur_config.rear_torque_scale) * accel_torque_pool * (in.loadcells.FL / sum_normal) );
        speed_out.torque_lim_nm.FR = ((2.0 - cur_config.rear_torque_scale) * accel_torque_pool * (in.loadcells.FR / sum_normal) );
        speed_out.torque_lim_nm.RL = (cur_config.rear_torque_scale * accel_torque_pool * (in.loadcells.RL / sum_normal) );
        speed_out.torque_lim_nm.RR = (cur_config.rear_torque_scale * accel_torque_pool * (in.loadcells.RR / sum_normal) );

        // cmd_out.out = control::apply_power_limit(speed_out, in.current_rpms, cur_config.max_power_kw);

        // spdlog::info("Ticked torque controller mode, {} {} {}", torqueRequest, sum_normal, max_rpm);
    }
    else
    {
        // Negative torque request
        
        float regen_torque_pool = accelRequest * cur_config.max_regen_torque * 4; 
        
        speed_out.desired_rpms.FL = 0;
        speed_out.desired_rpms.FR = 0;
        speed_out.desired_rpms.RL = 0;
        speed_out.desired_rpms.RR = 0;
        
        if (cur_config.apply_vectoring_in_regen) {
            speed_out.torque_lim_nm.FL = (regen_torque_pool * (in.loadcells.FL / sum_normal) * (2.0 - cur_config.rear_torque_scale));
            speed_out.torque_lim_nm.FR = (regen_torque_pool * (in.loadcells.FR / sum_normal) * (2.0 - cur_config.rear_torque_scale));
            speed_out.torque_lim_nm.RL = (regen_torque_pool * (in.loadcells.RL / sum_normal) * cur_config.rear_torque_scale);
            speed_out.torque_lim_nm.RR = (regen_torque_pool * (in.loadcells.RR / sum_normal) * cur_config.rear_torque_scale);
            cmd_out.out = speed_out; // no need to apply power limit for regen request
        } else {
            float reg_torq = accelRequest * cur_config.max_regen_torque;
            speed_out.torque_lim_nm.FL = ( reg_torq * ( 2.0 - cur_config.rear_torque_scale) );
            speed_out.torque_lim_nm.FR = ( reg_torq * ( 2.0 - cur_config.rear_torque_scale) );
            speed_out.torque_lim_nm.RL = ( reg_torq * cur_config.rear_torque_scale);
            speed_out.torque_lim_nm.RR = ( reg_torq * cur_config.rear_torque_scale);
            cmd_out.out = speed_out; // no need to apply power limit for regen request
        }
        
    }

    return cmd_out;
}
