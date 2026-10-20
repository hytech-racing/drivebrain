#include <LaunchProfile.hpp>
#include <cstdlib>
#include <iostream>
#include <limits>

using Profile = control::LaunchProfile;
using Phase = Profile::Phase;
void check(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
std::array<float, 4> wheels(float speed) { return {speed, speed, speed, speed}; }

int main() {
    Profile::Config config;
    Profile p;
    check(p.step(0.004f, wheels(0), true, false).torque_limit_nm == 0, "Idle must not drive");
    p.start(config, true, wheels(0));
    auto out = p.step(0.1f, wheels(0), true, false);
    check(std::abs(out.speed_mps - 0.05f) < 1e-6f, "Linear acceleration slope");
    for (int i = 0; i < 100; ++i) out = p.step(0.1f, wheels(0), true, false);
    check(out.speed_mps == config.max_speed_mps, "Speed must be capped");

    // Simulate perfect wheel speed tracking through a complete run.
    p = Profile{};
    p.start(config, true, wheels(0));
    float speed = 0;
    float brake_start = 0;
    float brake_entry_speed = 0;
    for (int i = 0; i < 10000 && p.phase() != Phase::COMPLETE; ++i) {
        const auto before = p.phase();
        out = p.step(0.004f, wheels(speed), true, false);
        check(std::isfinite(out.speed_mps) && out.speed_mps >= 0, "Finite forward target");
        if (before == Phase::ACCELERATING && p.phase() == Phase::BRAKING) {
            brake_start = p.distance_m();
            brake_entry_speed = speed;
            check(std::abs(brake_start - config.accel_distance_m) < 0.02f, "Measured accel distance");
            check(std::abs(p.deceleration_mps2() - speed * speed / (2 * config.braking_distance_m)) < 1e-5f,
                  "Braking deceleration from measured entry speed");
        }
        if (before == Phase::BRAKING) check(out.speed_mps <= speed, "Braking target must decrease");
        speed = out.speed_mps;
    }
    check(p.phase() == Phase::COMPLETE, "Full profile must finish");
    check(brake_entry_speed > 0, "Simulation entered braking");
    check(std::abs(p.distance_m() - brake_start - config.braking_distance_m) < 0.03f,
          "Simulated stopping distance");
    check(p.step(0.004f, wheels(0), true, false).speed_mps == 0, "No automatic repeat");

    p = Profile{};
    p.start(config, true, wheels(0));
    // Only one wheel turning still advances virtual distance, without cancellation.
    p.step(0.1f, {1, -1, 1, -1}, true, false);
    check(std::abs(p.distance_m() - 0.05f) < 1e-6f, "All four measured wheels contribute");
    out = p.step(0.01f, wheels(1), true, true);
    check(p.phase() == Phase::BRAKING && out.speed_mps == 1, "Early brake starts from measured speed");
    check(std::abs(p.deceleration_mps2() - 0.1f) < 1e-6f, "Early brake respects configured distance");
    p.step(0.01f, wheels(1), false, false);
    check(p.phase() == Phase::FAULT, "Stale feedback must fault");
    check(p.step(0.01f, wheels(0), true, false).torque_limit_nm == 0, "Fault latches zero torque");
    p.start(config, false, wheels(0));
    check(p.phase() == Phase::FAULT, "Unhealthy start rejected");
    p.start(config, true, {0, 0, 0, 1});
    check(p.phase() == Phase::FAULT, "Every wheel must be stopped to start");
    p.start(config, true, wheels(0));
    p.step(0.101f, wheels(0), true, false);
    check(p.phase() == Phase::FAULT, "Loop stall rejected");
    p.start(config, true, wheels(0));
    p.step(0.004f, wheels(std::numeric_limits<float>::quiet_NaN()), true, false);
    check(p.phase() == Phase::FAULT, "Nonfinite measurement rejected");
    config.braking_distance_m = 0;
    check(!Profile::valid(config), "Zero braking distance rejected");
    config.braking_distance_m = std::numeric_limits<float>::infinity();
    check(!Profile::valid(config), "Infinite configuration rejected");
    std::cout << "Launch profile checks passed\n";
}
