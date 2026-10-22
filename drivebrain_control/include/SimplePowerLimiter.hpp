#include <StateTracker.hpp>
#include <Literals.hpp>

using namespace core;

namespace control {

    core::SpeedControlOut apply_power_limit(SpeedControlOut current_control, veh_vec<float> current_rpms, float max_power_kw);

}