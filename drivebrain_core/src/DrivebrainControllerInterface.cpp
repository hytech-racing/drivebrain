#include "Controller.hpp"
#include "ControllerManager.hpp"
#include "StateTracker.hpp"
#include <DrivebrainControllerInterface.hpp>
#include <cstddef>

namespace matlab_model_gen {
inline constexpr std::size_t num_controllers = 1;
}

/** Singleton Methods */
void core::DrivebrainControllerInterface::create() {
    DrivebrainControllerInterface* expected = nullptr;
    DrivebrainControllerInterface* local = new DrivebrainControllerInterface();
    if(!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        // Already initialized, delete local instance
        delete local;
    }
}

core::DrivebrainControllerInterface& core::DrivebrainControllerInterface::instance() {
    DrivebrainControllerInterface* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "Drivebrain Controller Interface has not been initialized");
    return *instance;
}

void core::DrivebrainControllerInterface::destroy() {
    DrivebrainControllerInterface* instance = _s_instance.exchange(nullptr, std::memory_order_acq_rel);
    if (instance) {
        delete instance;
    }
}

core::DrivebrainControllerInterface::DrivebrainControllerInterface() {
    core::FoxgloveServer::instance().register_param_callback(std::bind(&core::DrivebrainControllerInterface::_handle_parameter_updates, this, std::placeholders::_1));
}

/* Private Methods */
void core::DrivebrainControllerInterface::_handle_parameter_updates(const std::unordered_map<std::string, core::DBParam> &new_params) {

    spdlog::info("Entering drivebrain controller interface parameter updates.");
    
    // TODO update this
    if (auto pval = std::get_if<bool>(&new_params.at("drivebraincontrollerinterface/should_log"))) {
        bool should_log = *pval;
        if (should_log) {
            _request_start_logging();
        } else {
            _request_stop_logging();
        }
    }

    if (auto pval = std::get_if<int>(&new_params.at("drivebraincontrollerinterface/controller_index"))) {
        int controller_index = *pval;
        _request_controller_change(controller_index);
    }

    spdlog::info("Exiting drivebrain controller interface parameter updates.");

}

void core::DrivebrainControllerInterface::_request_start_logging() {
    std::tuple<std::string, bool> mcap_status = core::MCAPLogger::instance().status();
    bool is_logging = std::get<1>(mcap_status);
    if (!is_logging) {
        core::MCAPLogger::instance().open_new_mcap(); 
        core::MCAPLogger::instance().init_logging();
    }
}

void core::DrivebrainControllerInterface::_request_stop_logging() {
    std::tuple<std::string, bool> mcap_status = core::MCAPLogger::instance().status();
    bool is_logging = std::get<1>(mcap_status);
    if (is_logging) {
        core::MCAPLogger::instance().stop_logging();
        core::MCAPLogger::instance().close_active_mcap(); 
    }
}

void core::DrivebrainControllerInterface::_request_controller_change(int controller_index) {
    const size_t num_controllers = 1 + matlab_model_gen::num_controllers;
    auto& controller_manager = ControllerManager<control::Controller<ControllerOutput, VehicleState>, num_controllers>::instance();

    if (controller_index == controller_manager.get_active_controller_index()) {
        return;
    }

    spdlog::info("Controller swap requested to controller: {}", controller_index);
    
    if (controller_index < 0) {
        spdlog::warn("Ignoring negative controller index request: {}", controller_index);
        return;
    }

    const auto state_and_validity = core::StateTracker::instance().get_latest_state_and_validity();
    const bool switched = controller_manager.swap_active_controller(static_cast<size_t>(controller_index), state_and_validity.first);

    if (!switched) {
        spdlog::warn("Controller switch rejected for index {}", controller_index);
    }

}






