#pragma once

#include <Controller.hpp>
#include <StateTracker.hpp>
#include <Literals.hpp>
#include <FoxgloveServer.hpp>

#include <hytech_msgs.pb.h>

#include <utility>
#include <mutex>

using namespace core;

namespace control
{
    class LoadCellTorqueController : public Controller<core::ControllerOutput, core::VehicleState> {
    public:
        // rear_torque_scale:
        // 0 to 2 scale on forward torque to rear wheels. 0 = FWD, 1 = Balanced, 2 = RWD

        // regen_torque_scale:
        // same as rear_torque_scale but applies to regen torque split. 0 = All regen
        // torque on the front, 1 = 50/50, 2 = all regen torque on the rear
        
        struct config {
            torque_nm max_torque;
            torque_nm max_regen_torque;
            float rear_torque_scale;  
            float regen_torque_scale; 
            speed_m_s positive_speed_set;
            float max_power_kw;
            int dt_rate_hz;
            bool apply_vectoring_in_regen;
        };

        LoadCellTorqueController() {}

        float get_dt_sec() override { 
            return (double) 1.0 / _config.dt_rate_hz;
        }

        bool init();

        core::ControllerOutput step_controller(const VehicleState &in) override;

    private:

        void _handle_param_updates(const std::unordered_map<std::string, DBParam> &new_param_map);
        
        std::mutex _config_mutex;
        config _config{};
    };
}