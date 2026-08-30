#pragma once

#include <array>
#include <memory>
#include <thread>
#include <chrono>
#include <pthread.h>
#include <sched.h>

#include <Controller.hpp>
#include <DriverBus.hpp>
#include <StateTracker.hpp>
#include <Literals.hpp>
#include <StateTracker.hpp>
#include <FoxgloveServer.hpp>


namespace core {


template <typename ControllerType, size_t NumControllers>
class ControllerManager {
public:

    /**
     * @brief Constructs a new singleton instance of the controller manager
     * @param controllers the controllers to construct the controller manager with 
     */
    static void create(std::array<std::shared_ptr<ControllerType>, NumControllers> controllers);

    /**
     * @brief Fetches a controller manager instance
     * 
     * @return the instance of the controler manager
     */
    static ControllerManager<ControllerType, NumControllers>& instance();

    /**
     * Destroys the instance of the controller manager
     */
    static void destroy();


    /**
     * @brief Custom destructor for safely releasing the controler manager
     */
    ~ControllerManager() {
        spdlog::info("Destructing controller manager");
        _s_instance.store(nullptr, std::memory_order_release);
        spdlog::info("Controller manager singleton instance released");
    }

    /**
     * Initializes the controller manager as long as 
     * the limits are reasonable. 
     * @return Whether or not the controller manager successfully initialized
     */
    bool init();

    /**
     * Replaces the controllers with new ones
     */
    void update_controllers(std::array<std::shared_ptr<ControllerType>, NumControllers> controllers) {
        _controllers = controllers;
    }

    /**
     * @brief Attempts to switch the active controller
     * @param new_controller_index the desired controller index
     * @return true if it successfully switches and false if it does not

     */
    bool swap_active_controller(size_t new_controller_index, const core::VehicleState& input);
    
    /**
     * @brief Fetches the active controller's desired seconds between controller evaluations
     * @return The period in seconds for the active controller
     */
    float get_active_controller_timestep() {
        return _controllers[_current_controller_index]->get_dt_sec();
    }

    /**
     * @brief Fetches the active controller index
     * @return The index of the active controller
     */
    int get_active_controller_index() {
        return _current_controller_index;
    }


    /**
     * @brief Fetches the current controller manager state
     * @return ControllerManagerState types: ControllerManagerStatus, ControllerOutput
     */
    ControllerManagerState get_current_ctr_manager_state() {
        return _current_ctr_manager_state;
    }

    /**
     * @brief Evaluates the currently active controler
     * @param input The current vehicle state maintained by the state estimator
     * @return respective controller output to command the drivetrain
     */
    core::ControllerOutput step_active_controller(const core::VehicleState& input)
    {
        if(!_controllers[_current_controller_index] || _controllers[_current_controller_index] == nullptr) {
            _current_ctr_manager_state.current_status = core::ControllerManagerStatus::ERROR_NULLPTR_CONTROLLER;
            return {std::monostate()};
        }
        return _controllers[_current_controller_index]->step_controller(input);
    }

private:
    

    /**
     * @brief Private constructor to construct an instance of the controller manager
     * @param controllers list of controllers that the manager will mux between and manage
     */
    ControllerManager(std::array<std::shared_ptr<ControllerType>, NumControllers> controllers) : _controllers(std::move(controllers)) {
        auto& foxglove = core::FoxgloveServer::instance();
        auto max_speed = foxglove.get_param<float>("controllermanager/max_controller_switch_speed_ms");
        auto max_torque = foxglove.get_param<float>("controllermanager/max_torque_switch_nm");
        auto max_accel = foxglove.get_param<float>("controllermanager/max_accel_switch_float");
        auto max_rpm = foxglove.get_param<float>("controllermanager/max_requested_rpm");
        _max_switch_rpm = max_speed.value_or(5.0f) * constants::METERS_PER_SECOND_TO_RPM;
        _max_torque_req_switch = max_torque.value_or(10.0f);
        _max_accel_req_switch = max_accel.value_or(0.5f);
        _max_rpm_req_switch = max_rpm.value_or(20000.0);  
    }

    ControllerManager(const ControllerManager&) = delete;
    ControllerManager& operator=(const ControllerManager&) = delete;

    /**
     * @brief Helper method to decide if we can switch the controller based on current and next state and next controller output
     */
    core::ControllerManagerStatus _can_switch_controller(const core::VehicleState &current_state, const core::ControllerOutput &previous_output, const core::ControllerOutput &next_controller_output);

    size_t _current_controller_index = 0;
    core::ControllerManagerState _current_ctr_manager_state;
    std::array<std::shared_ptr<ControllerType>, NumControllers> _controllers;

    float _max_switch_rpm; /* the maximum rpm at which you allow switching */
    float _max_torque_req_switch; /* the maximum torque at which you allow switching */
    float _max_accel_req_switch; /* the maximum accel request at which you allow switching */
    float _max_rpm_req_switch; /* the maximum rpm request at which you allow switching */

    /* Controller manager instance */
    inline static std::atomic<ControllerManager*> _s_instance;

};

#include "ControllerManager.tpp"

}
