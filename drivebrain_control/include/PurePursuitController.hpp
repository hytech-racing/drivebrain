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
    static constexpr float WHEELBASE_M = 1.53f;

    bool init();
    float get_dt_sec() { return 0.01f; }
    ControllerOutput step(const VehicleState& in);

private:
    struct Steering {
        float angle_rad;
        float curvature;  
        xyz_vec<float> target;
    };

    /* Front-wheel angle (rad) that arcs onto the point one lookahead past the closest path point, and that point */
    std::optional<Steering> _steering(const VehicleState& in, const std::vector<xyz_vec<float>>& path) const;

    float _lookahead_dist = 0.0f;
    float _min_speed = 0.0f;
    float _max_speed = 0.0f;
    float _max_lat_accel = 0.0f;
    float _torque_lim = 0.0f;

};

}
