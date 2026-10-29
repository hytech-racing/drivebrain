#pragma once

#include <Controller.hpp>
#include "SpeedProfile.hpp"
#include <StateTracker.hpp>

#include <atomic>
#include <chrono>
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

    /* Constant speed held until the track is closed (the discovery lap), and for the whole run if there is no speed profile */
    void set_constant_speed(float speed_mps) { _constant_speed = speed_mps; }

    /* Corner-dependent target speed (see SpeedProfile.hpp); nullopt keeps the constant speed */
    void set_speed_profile(std::optional<SpeedProfileParams> params) { _speed_profile = params; }

    /* Once the run is over the car is told to stop (zero speed); the steering still follows the path */
    void set_finished(bool finished) { _finished = finished; }

    /* The speed profile is only used once the track is closed; until then (the discovery lap) the car holds the constant speed */
    void set_track_closed(bool closed) { _track_closed = closed; }

private:
    struct Steering {
        float angle_rad;
        xyz_vec<float> target;
        size_t closest;  // index of the path point nearest the car
    };

    /* Front-wheel angle (rad) that arcs onto the point one lookahead past the closest path point, and that point */
    std::optional<Steering> _steering(const VehicleState& in, const std::vector<xyz_vec<float>>& path) const;

    float _lookahead_dist = 2.5f;
    float _constant_speed = 0.0f;  // m/s, from the config; held on the discovery lap and whenever there is no speed profile
    std::optional<SpeedProfileParams> _speed_profile;
    float _commanded_speed = 0.0f;                          // what the speed profile has ramped up to; only touched by step()
    std::chrono::steady_clock::time_point _last_step_time{};
    std::atomic<bool> _finished{false};
    std::atomic<bool> _track_closed{false};  // set by the autonomy thread, read by the controller thread
    float _torque_lim = 5.0f;

};

}
