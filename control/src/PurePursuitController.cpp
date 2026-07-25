#include "PurePursuitController.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace control
{
namespace
{
constexpr double PI = 3.14159265358979323846;

inline double deg_to_rad(double degrees) { return degrees * (PI / 180.0); }

Point2D transform_world_to_body(const Point2D& world_delta, double yaw_rad)
{
    const double cy = std::cos(yaw_rad);
    const double sy = std::sin(yaw_rad);
    return Point2D{cy * world_delta.x_m + sy * world_delta.y_m,
                   -sy * world_delta.x_m + cy * world_delta.y_m};
}
}  // namespace

std::optional<PurePursuitResult> compute_pure_pursuit(
    const PurePursuitInput& input, const PurePursuitParams& params)
{
    if (!std::isfinite(input.target_pos_world.x_m) ||
        !std::isfinite(input.target_pos_world.y_m) ||
        !std::isfinite(input.vehicle_pos_world.x_m) ||
        !std::isfinite(input.vehicle_pos_world.y_m) ||
        !std::isfinite(input.vehicle_yaw_world_rad))
    {
        return std::nullopt;
    }

    if (!std::isfinite(params.wheelbase_m) || params.wheelbase_m <= 0.0 ||
        !std::isfinite(params.max_steer_deg) || params.max_steer_deg < 0.0)
    {
        return std::nullopt;
    }

    PurePursuitResult result;

    const Point2D delta_world{
        input.target_pos_world.x_m - input.vehicle_pos_world.x_m,
        input.target_pos_world.y_m - input.vehicle_pos_world.y_m};

    const Point2D target_pos_body =
        transform_world_to_body(delta_world, input.vehicle_yaw_world_rad);

    // target behind car, return null
    if (target_pos_body.x_m <= 0.0)
    {
        return std::nullopt;
    }

    const double distance_squared = target_pos_body.x_m * target_pos_body.x_m +
                                    target_pos_body.y_m * target_pos_body.y_m;

    if (distance_squared < 1e-6)
    {
        return std::nullopt;
    }

    const double curvature = (2.0 * target_pos_body.y_m) / (distance_squared);

    const double delta_rad = std::atan(params.wheelbase_m * curvature);

    const double max_steer_rad = deg_to_rad(params.max_steer_deg);

    result.curvature_inv_m = curvature;
    result.steering_rad = std::clamp(delta_rad, -max_steer_rad, max_steer_rad);
    result.target_pos_body = target_pos_body;

    return result;
}
}  // namespace control
