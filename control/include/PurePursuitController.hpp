#pragma once

#include <Controller.hpp>
#include <StateTracker.hpp>

using namespace core;

namespace control
{

class PurePursuitController : public Controller<ControllerOutput, VehicleState>
{
   public:
    bool init();
    ControllerOutput step_controller(const VehicleState& in) override;

   private:
    float _lookahead_dist;
};

}  // namespace control

namespace control
{

struct Point2D
{
    double x_m{};
    double y_m{};
};

struct PurePursuitInput
{
    Point2D vehicle_pos_world{};
    double vehicle_yaw_world_rad{};

    Point2D target_pos_world{};
};

struct PurePursuitParams
{
    double wheelbase_m{};
    double max_steer_deg{};
};

struct PurePursuitResult
{
    Point2D target_pos_body{};
    double curvature_inv_m{};
    double steering_rad{};
};

std::optional<PurePursuitResult> compute_pure_pursuit(
    const PurePursuitInput& input, const PurePursuitParams& params);

}  // namespace control
