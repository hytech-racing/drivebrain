#pragma once

#include <Controller.hpp>
#include <StateTracker.hpp>
#include <memory>

using namespace core;

namespace control {
namespace driverless {

struct DriverlessControllerInput {
    VehicleState vehicle_state;
    std::shared_ptr<const std::vector<xyz_vec<float>>> path;
    std::chrono::steady_clock::time_point timestamp;
};
using DriverlessControllerOutput = ControllerOutput;

class DriverlessController {
public:

    virtual void step_control(const DriverlessControllerInput& input, DriverlessControllerOutput& output) = 0;
    virtual void readInputs(const DriverlessControllerInput& input) = 0;
    virtual void process() = 0;
    virtual void writeOutputs(const DriverlessControllerOutput& output) = 0;
protected:
    std::shared_ptr<std::vector<xyz_vec<float>>> getOrderedPathVertices();

private:
    std::vector<xyz_vec<float>> ordered_path_vertices_;
};

}
}
