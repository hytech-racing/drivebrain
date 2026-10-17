#pragma once

#include <Controller.hpp>
#include <StateTracker.hpp>

using namespace core;

namespace control
{

class PurePursuitController : public Controller<ControllerOutput, VehicleState> {
public:
bool init();
ControllerOutput step_controller(const VehicleState& in) override;

private:
float _lookahead_dist;  

};

}
