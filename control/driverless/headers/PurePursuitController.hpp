#pragma once

#include <DriverlessController.hpp>
#include <StateTracker.hpp>

using namespace core;

namespace control {
namespace driverless {

struct PurePursuitConfig {
    float lookahead_dist_min;
    float lookahead_dist_max;
    float lookahead_dist_speed_k_p;
};

class PurePursuitController : public DriverlessController {
public:
    PurePursuitController(const PurePursuitConfig& config) : _config(config) {}
    void readInputs(const DriverlessControllerInput& input) override;
    void process() override;
    void writeOutputs(const DriverlessControllerOutput& output) override;
    void step_control(const DriverlessControllerInput& input, DriverlessControllerOutput& output) override;
private:
    PurePursuitConfig _config;
};

}
}
