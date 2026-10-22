#include "ControllerManager.hpp"
#include <cmath>
#include <spdlog/spdlog.h>
#include <iostream>


/****************************************************************
 * SINGLETON METHODS
 ****************************************************************/
template<typename ControllerType, size_t NumControllers> 
void core::ControllerManager<ControllerType, NumControllers>::create(std::array<std::shared_ptr<ControllerType>, NumControllers> controllers) {
    ControllerManager* expected = nullptr;
    ControllerManager* local = new ControllerManager(controllers);
    if(!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        // Already initialized, delete local instance
        delete local;
    }
}

template<typename ControllerType, size_t NumControllers> 
core::ControllerManager<ControllerType, NumControllers>& core::ControllerManager<ControllerType, NumControllers>::instance() {
    ControllerManager* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "ControllerManager has not been initialized");
    return *instance;
}

template<typename ControllerType, size_t NumControllers> 
void core::ControllerManager<ControllerType, NumControllers>::destroy() {
    ControllerManager* instance = _s_instance.exchange(nullptr, std::memory_order_acq_rel);
    if (instance) {
        delete instance;
    }
}

/****************************************************************
 * PUBLIC CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
template <typename ControllerType, size_t NumControllers>
bool core::ControllerManager<ControllerType, NumControllers>::init()
{

    if(_max_accel_req_switch > 1.0f){
        spdlog::error("max accel switch float > 1.0");
        return false;
    }
    // Initialize state
    _current_ctr_manager_state = ControllerManagerState{
        .current_status = ControllerManagerStatus::NO_ERROR,
        .current_controller_output = std::monostate{}
    };
    
    return true;

}

template<typename ControllerType, size_t NumControllers>
core::ControllerManagerStatus core::ControllerManager<ControllerType, NumControllers>::_can_switch_controller(const core::VehicleState &current_state,
                                                                                                                       const core::ControllerOutput &previous_output,
                                                                                                                       const core::ControllerOutput &next_controller_output)
{

    using status_type = core::ControllerManagerStatus;

    // shared function to check if values are above a maximum value
    auto check_veh_vec = [](core::veh_vec<float> vehicle_vector, float max_val, bool check_with_abs) -> bool
    {
        if (check_with_abs) {
            return (abs(vehicle_vector.FL) > max_val) ||
                   (abs(vehicle_vector.FR) > max_val) ||
                   (abs(vehicle_vector.RL) > max_val) ||
                   (abs(vehicle_vector.RR) > max_val);
        } else {
            return (vehicle_vector.FL > max_val) ||
                   (vehicle_vector.FR > max_val) ||
                   (vehicle_vector.RL > max_val) ||
                   (vehicle_vector.RR > max_val);
        }
    };

    // check to see if current drivetrain rpms are too high to switch controller
    if (check_veh_vec(current_state.current_rpms, _max_switch_rpm, true)) {
        _current_ctr_manager_state.current_status = status_type::ERROR_SPEED_TOO_HIGH;
        return _current_ctr_manager_state.current_status;
    }

    if (current_state.input.requested_accel > _max_accel_req_switch) {
        _current_ctr_manager_state.current_status = status_type::ERROR_DRIVER_ON_PEDAL;
        return _current_ctr_manager_state.current_status;
    }

    // function to check whether or not the controller output is with range. 
    // can determine what type the controller output is and checks to see whether or not it has issues.
    // if the controller output is a speed controller type: checks both desired rpms level and max torque limit level to verify range.
    // if the controller output is a torque controller type: only checks the torque setpoint
    auto verify_controller_output = [this, &check_veh_vec](const core::ControllerOutput &controller_output) -> status_type
    {
        // 1: "trajectory/speed" control (speed control with torque limit)
        // check: make sure that either the setpoint rpm is low enough OR (AND?) the torque setpoint is low enough
        if (const core::SpeedControlOut *pval = std::get_if<core::SpeedControlOut>(&controller_output.out)) {
            if (check_veh_vec(pval->desired_rpms, _max_rpm_req_switch, true)) {
                return status_type::ERROR_OUTPUT_EXCEEDS_PHYS_LIMITS;
            } else if (check_veh_vec(pval->torque_lim_nm, _max_torque_req_switch, false)) {
                return status_type::ERROR_TORQUE_TOO_HIGH;
            } else {
                return status_type::NO_ERROR;
            }
        }
        // 2: "torque" control (just torque setpoints)
        // check: make sure the torque setpoint is low enough
        else if (const core::TorqueControlOut *pval = std::get_if<core::TorqueControlOut>(&controller_output.out))
        {
            if (check_veh_vec(pval->desired_torques_nm, _max_torque_req_switch, false)) {
                return status_type::ERROR_TORQUE_TOO_HIGH;
            } else {
                return status_type::NO_ERROR;
            }
        } else {
            return status_type::ERROR_CONTROLLER_NO_TORQUE_OR_SPEED_OUTPUT;
        }
    };

    status_type prev_status = verify_controller_output(previous_output);
    status_type switch_status = verify_controller_output(next_controller_output);

    if (prev_status == status_type::NO_ERROR && switch_status == status_type::NO_ERROR) {
        _current_ctr_manager_state.current_status = status_type::NO_ERROR;
    } else if (prev_status != status_type::NO_ERROR && switch_status == status_type::NO_ERROR) {
        _current_ctr_manager_state.current_status = prev_status;
    } else {
        _current_ctr_manager_state.current_status = switch_status;
    }

    return _current_ctr_manager_state.current_status;
}


template <typename ControllerType, size_t NumControllers>
bool core::ControllerManager<ControllerType, NumControllers>::swap_active_controller(size_t new_controller_index, const core::VehicleState& input)
{
    using status_type = core::ControllerManagerStatus;
    static const size_t num_controllers = _controllers.size(); // size_t is unsigned, new_controller_index being less than 0 is always false. 
    
    if (new_controller_index >= num_controllers) {
        _current_ctr_manager_state.current_status = status_type::ERROR_CONTROLLER_INDEX_OUT_OF_RANGE;
        spdlog::info("Switch mode failed with error code: " + std::to_string(static_cast<int>(_current_ctr_manager_state.current_status)));
        return false;
    } else {
        if (_current_controller_index == new_controller_index) {
            _current_ctr_manager_state.current_status = status_type::ERROR_REQUESTING_SAME_CTR_TYPE;
            spdlog::info("Switch mode failed with error code: " + std::to_string(static_cast<int>(_current_ctr_manager_state.current_status)));
            return false;
        }
    }

    if (!_controllers[new_controller_index]) {
        _current_ctr_manager_state.current_status = status_type::ERROR_NULLPTR_CONTROLLER;
        spdlog::error("Switch mode failed due to nullptr controller with error code: " + std::to_string(static_cast<int>(_current_ctr_manager_state.current_status)));
        return false;
    }

    core::ControllerOutput next_output = _controllers[new_controller_index]->step_controller(input); 
    
    status_type can_switch_controller = _can_switch_controller(
        input, 
        _controllers[_current_controller_index]->step_controller(input), 
        next_output
    );
    
    if (can_switch_controller == status_type::NO_ERROR) {
        _current_controller_index = new_controller_index;
        _current_ctr_manager_state.current_controller_output = next_output; 
        spdlog::info("Switched mode: {}", std::to_string(new_controller_index));
        return true;
    } else {
        spdlog::info("Switch mode failed with error code: {}", std::to_string(static_cast<int>(_current_ctr_manager_state.current_status)));
        return false;
    }

}

