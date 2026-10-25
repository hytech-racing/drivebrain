#pragma once

#include <Controller.hpp>
#include <StateTracker.hpp>

#include <optional>
#include <vector>

using namespace core;

namespace control
{

class PurePursuitController {

public:
    bool init();
    float get_dt_sec() { return 0.01f; }
    ControllerOutput step(const VehicleState& in);

private:
    /* Front-wheel angle (rad) that arcs onto the point one lookahead past the closest path point */
    std::optional<float> _steering_angle_rad(const VehicleState& in, const std::vector<xyz_vec<float>>& path) const;

    float _lookahead_dist = 2.5f;
    float _wheelbase = 1.53f;
    float _target_speed = 3.0f; 
    float _torque_lim = 5.0f;

};

}
