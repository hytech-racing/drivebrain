#pragma once

#include <optional>
#include <string>
#include <vector>

namespace planning
{

struct Point2D
{
    double x_m{};
    double y_m{};
};

struct ReferencePathPoint
{
    double x_m{};
    double y_m{};
    double s_m{};
};

struct PathProjection
{
    ReferencePathPoint point{};
    double distance_squared{};
    std::size_t segment_index{};
    double segment_t{};
};

struct ReferencePath
{
    std::vector<ReferencePathPoint> points;
    double length_m{};
};

std::optional<ReferencePath> load_reference_path_csv(
    const std::string& csv_path);

std::optional<PathProjection> project_onto_path(
    const Point2D& vehicle_position_world, const ReferencePath& reference_path);

std::optional<ReferencePathPoint> interpolate_at_s(const ReferencePath& path,
                                                   double requested_s_m);
}  // namespace planning
