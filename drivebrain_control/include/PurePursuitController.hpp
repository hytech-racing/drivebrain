#pragma once

#include <Controller.hpp>
#include <StateTracker.hpp>

#include <FoxgloveServer.hpp>

#include <atomic>
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
        xyz_vec<float> target;
    };

    /* Front-wheel angle (rad) that arcs onto the point one lookahead past the closest path point, and that point */
    std::optional<Steering> _steering(const VehicleState& in, const std::vector<xyz_vec<float>>& path) const;

    /* Fastest speed now that can still brake (at max_brake) down to each upcoming corner's limit, sqrt(max_lat_accel / curvature) */
    float _profile_speed(const std::vector<xyz_vec<float>>& path) const;

    std::atomic<float> _lookahead_dist{0.0f};
    std::atomic<float> _min_speed{0.0f};
    std::atomic<float> _max_speed{0.0f};
    std::atomic<float> _max_lat_accel{0.0f};
    std::atomic<float> _max_brake{0.0f};
    std::atomic<float> _torque_lim{0.0f};
    boost::signals2::scoped_connection _param_connection;

};

}
