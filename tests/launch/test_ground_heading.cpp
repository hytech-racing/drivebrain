#include <OMSGroundEstimator.hpp>
#include <HeadingHold.hpp>
#include <LaunchProfile.hpp>
#include <cstdlib>
#include <iostream>

void check(bool value, const char* message) {
    if (!value) { std::cerr << message << '\n'; std::exit(1); }
}
bool near(double a, double b) { return std::abs(a - b) < 1e-4; }
int main() {
    estimation::OMSGroundEstimator sensor({});
    sensor.update(0, 3.6f, 0, 10);
    sensor.update(0.1f, 3.6f, 0, 10);
    check(near(sensor.state().speed_mps, 1), "Explicit km/h to m/s conversion");
    check(near(sensor.state().distance_m, 0.1), "Ground distance uses optical velocity");
    check(near(sensor.state().heading_deg, 1), "Relative heading integrates yaw rate");
    check(!sensor.update(0.1f, 3.6f, 0, 10), "Duplicate samples are not integrated twice");
    sensor.update(0, 3.6f, 0, 10);
    check(near(sensor.state().heading_deg, 1), "Sensor clock reset does not jump heading");
    estimation::OMSGroundEstimator rotated({1.0f, 90, -1, 2});
    rotated.update(0, 1, 0, 12);
    rotated.update(0.1f, 1, 0, 12);
    check(near(rotated.state().vx_mps, 0) && near(rotated.state().vy_mps, 1), "Mount rotation applies to optical velocity");
    check(near(rotated.state().heading_deg, -1), "Yaw sign and gyro bias apply");
    check(near(rotated.state().sideslip_deg, 90), "Velocity direction is sideslip, separate from heading");

    control::HeadingHold heading;
    control::HeadingHold::Config tuning;
    heading.reset(30);
    check(near(heading.step(tuning, 30, 0), 0), "Initial heading is held");
    check(near(heading.step(tuning, 32, 0), -4), "Left heading deviation commands right correction");
    check(near(heading.step(tuning, 28, 0), 4), "Right heading deviation commands left correction");
    check(near(heading.step(tuning, 30, 2), -1), "Yaw rate damps a developing turn");
    check(near(heading.step(tuning, 80, 0), -15), "Column correction is limited");
    heading.reset(359);
    check(near(heading.step(tuning, 1, 0), -4), "Heading wraps at 360 degrees");
    tuning.steering_sign = -1;
    check(near(heading.step(tuning, 1, 0), 4), "Steering sign is configurable");

    control::LaunchProfile profile;
    control::LaunchProfile::Config config;
    profile.start(config, true, {0, 0, 0, 0}, {{0, 100}});
    profile.step(0.004f, {5, 5, 5, 5}, true, false, {{1, 102}});
    check(near(profile.distance_m(), 2), "Wheel slip does not change ground travel");
    profile.step(0.004f, {5, 5, 5, 5}, true, false, {{1, 105}});
    check(profile.phase() == control::LaunchProfile::Phase::BRAKING, "Ground distance triggers braking");
    check(near(profile.deceleration_mps2(), 0.1), "Braking uses ground entry speed rather than wheel spin");
    auto output = profile.step(0.004f, {5, 5, 5, 5}, true, false, {{1, 110}});
    check(output.speed_mps == 0, "Ground braking distance forces zero target");
    check(profile.phase() == control::LaunchProfile::Phase::BRAKING, "Moving wheels do not report complete");
    profile.step(0.004f, {0, 0, 0, 0}, true, false, {{0, 110}});
    check(profile.phase() == control::LaunchProfile::Phase::COMPLETE, "Both ground and wheel speeds stop");
    std::cout << "Ground speed and heading checks passed\n";
}
