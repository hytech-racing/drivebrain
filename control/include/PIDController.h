#pragma once

#include <StateTracker.hpp>
#include <algorithm>
#include <Controller.hpp>

namespace control {
namespace driverless {

class PIDController : public Controller<core::SpeedControlOut, core::VehicleState> {
public:
    PIDController(core::PIDGains gains, float dt, float integral_limit_l_, float integral_limit_u_) 
        : dt_(dt), gains_(gains),
         integral_limit_l_(integral_limit_l_), integral_limit_u_(integral_limit_u_) {}
    PIDController() = default;

    core::SpeedControlOut step_controller(const core::VehicleState& in) override {
        // todo compute error, update, clamp 
        core::SpeedControlOut output;
    }

    float update(float error) {
        integral_ += error * dt_;
        integral_ = std::clamp(integral_, integral_limit_l_, integral_limit_u_);
        float derivative = (error - previous_error_) / dt_;
        previous_error_ = error;
        return gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
    }

    void setGains(core::PIDGains gains) {
        gains_ = gains;
    }

    float get_dt_sec() override {
        return dt_;
    }
    
private:
    core::PIDGains gains_;
    float dt_;
    float integral_limit_l_;
    float integral_limit_u_;
    float integral_{0.0f};
    float previous_error_{0.0f};
};

} // namespace driverless
} // namespace control