#include "Controller.hpp"
#include "ControllerManager.hpp"
#include "StateTracker.hpp"
#include <DrivebrainControllerInterface.hpp>
#include <cstddef>
#include <FoxgloveServer.hpp>


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
    auto drive_mode = FoxgloveServer::instance().get_param<int>("drivebraincontrollerinterface/drive_mode"); 
    auto enabled = FoxgloveServer::instance().get_param<bool>("drivebraincontrollerinterface/enabled");
    if (!enabled) {
        spdlog::warn("Drivebrain Controller Interface is not enabled");
        _enabled = false;
    } else {
        _enabled = *enabled;
    }
    
    if (!drive_mode) {
        spdlog::warn("Drive mode parameter not found, defaulting to MANUAL");
        _drive_mode = static_cast<int>(core::DriveMode::MANUAL);
    } else {
        _drive_mode = *drive_mode;
    }
    core::FoxgloveServer::instance().register_param_callback(std::bind(&core::DrivebrainControllerInterface::_handle_parameter_updates, this, std::placeholders::_1));
}

/* Private Methods */
void core::DrivebrainControllerInterface::_handle_parameter_updates(const std::unordered_map<std::string, core::DBParam> &new_params) {

    spdlog::info("Entering drivebrain controller interface parameter updates.");
    
    if (auto controller_index = process_param_update<int>(new_params, "drivebraincontrollerinterface/controller_index")) {
        _request_controller_change(*controller_index);
    }

    if (auto drive_mode = process_param_update<int>(new_params, "drivebraincontrollerinterface/drive_mode")) {
        spdlog::info("Drive mode updated to: {}", *drive_mode);
        if (*drive_mode < static_cast<int>(core::DriveMode::MANUAL) || *drive_mode > static_cast<int>(core::DriveMode::TELEOP)) {
            spdlog::warn("Invalid drive mode: {}", *drive_mode);
        } else {
            _drive_mode = *drive_mode;
        }
    }

    if (auto enabled = process_param_update<bool>(new_params, "drivebraincontrollerinterface/enabled")) {
        spdlog::info("Enabled updated to: {}", *enabled);
        _enabled = *enabled;
    }

    spdlog::info("Exiting drivebrain controller interface parameter updates.");
}

void core::DrivebrainControllerInterface::_request_controller_change(int controller_index) {
    auto& controller_manager = ControllerManager<control::Controller<ControllerOutput, VehicleState>, constants::NUM_DRIVERED_CONTROLLERS>::instance();

    if (controller_index == controller_manager.get_active_controller_index()) {
        return;
    }

    spdlog::info("Controller swap requested to controller: {}", controller_index);
    
    if (controller_index < 0) {
        spdlog::warn("Ignoring negative controller index request: {}", controller_index);
        return;
    }

    const auto state = core::StateTracker::instance().vehicle_state();
    const bool switched = controller_manager.swap_active_controller(static_cast<size_t>(controller_index), state.first);

    if (!switched) {
        spdlog::warn("Controller switch rejected for index {}", controller_index);
    }

}






