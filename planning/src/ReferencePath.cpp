#include "ReferencePath.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace planning
{

std::optional<ReferencePath> load_reference_path_csv(
    const std::string& csv_path)
{
    std::ifstream file(csv_path);

    if (!file.is_open())
    {
        return std::nullopt;
    }

    std::string line;

    // Skip the header row
    std::getline(file, line);

    ReferencePath path;
    double cumulative_s_m{};

    while (std::getline(file, line))
    {
        if (line.empty())
        {
            continue;
        }

        std::stringstream ss(line);
        double x_m, y_m, r;
        char comma1, comma2;

        if (!(ss >> x_m >> comma1 >> y_m >> comma2 >> r))
        {
            spdlog::error("Failed to parse this line: \"{}\"", line);
            continue;
        }

        if (!path.points.empty())
        {
            const double dx = x_m - path.points.back().x_m;
            const double dy = y_m - path.points.back().y_m;

            cumulative_s_m += std::hypot(dx, dy);
        }

        path.points.emplace_back(ReferencePathPoint{x_m, y_m, cumulative_s_m});
    }

    if (path.points.empty())
    {
        spdlog::error("No valid points were loaded from the CSV file.");
        return std::nullopt;
    }

    const ReferencePathPoint& first = path.points.front();
    const ReferencePathPoint& last = path.points.back();

    const double closing_length =
        std::hypot(first.x_m - last.x_m, first.y_m - last.y_m);
    path.length_m = last.s_m + closing_length;

    return path;
}

std::optional<PathProjection> project_onto_path(
    const Point2D& vehicle_position_world, const ReferencePath& path)
{
    if (path.points.empty())
    {
        return std::nullopt;
    }

    PathProjection best_projection;

    if (path.points.size() == 1)
    {
        best_projection.point = path.points[0];
        double dx = path.points[0].x_m - vehicle_position_world.x_m;
        double dy = path.points[0].y_m - vehicle_position_world.y_m;
        best_projection.distance_squared = (dx * dx) + (dy * dy);
        return best_projection;
    }

    best_projection.distance_squared = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < path.points.size(); ++i)
    {
        const std::size_t next_i = (i + 1) % path.points.size();

        const ReferencePathPoint& a = path.points[i];
        const ReferencePathPoint& b = path.points[next_i];

        Point2D ab = {b.x_m - a.x_m, b.y_m - a.y_m};
        Point2D ap = {vehicle_position_world.x_m - a.x_m,
                      vehicle_position_world.y_m - a.y_m};

        double dot_ap_ab = (ap.x_m * ab.x_m) + (ap.y_m * ab.y_m);
        double dot_ab_ab = (ab.x_m * ab.x_m) + (ab.y_m * ab.y_m);

        double t = 0.0;

        if (dot_ab_ab > 1e-6)
        {
            t = std::clamp(dot_ap_ab / dot_ab_ab, 0.0, 1.0);
        }

        PathProjection current_proj;

        current_proj.point.x_m = a.x_m + (t * ab.x_m);
        current_proj.point.y_m = a.y_m + (t * ab.y_m);

        const double segment_length_m = std::sqrt(dot_ab_ab);

        current_proj.point.s_m = a.s_m + t * segment_length_m;
        current_proj.segment_index = i;
        current_proj.segment_t = t;

        double dx = current_proj.point.x_m - vehicle_position_world.x_m;
        double dy = current_proj.point.y_m - vehicle_position_world.y_m;
        current_proj.distance_squared = (dx * dx) + (dy * dy);

        if (current_proj.distance_squared < best_projection.distance_squared)
        {
            best_projection = current_proj;
        }
    }

    return best_projection;
}

ReferencePathPoint lerp_points(const ReferencePathPoint& p1,
                               const ReferencePathPoint& p2, double target_s)
{
    if (p1.s_m == p2.s_m) return p1;
    double t = (target_s - p1.s_m) / (p2.s_m - p1.s_m);

    return ReferencePathPoint{p1.x_m + t * (p2.x_m - p1.x_m),
                              p1.y_m + t * (p2.y_m - p1.y_m), target_s};
}

std::optional<ReferencePathPoint> interpolate_at_s(const ReferencePath& path,
                                                   double requested_s_m)
{
    if (path.points.empty())
    {
        return std::nullopt;
    }

    if (path.points.size() == 1)
    {
        return path.points.front();
    }

    requested_s_m = std::fmod(requested_s_m, path.length_m);

    if (requested_s_m < 0.0)
    {
        requested_s_m += path.length_m;
    }

    auto it =
        std::lower_bound(path.points.begin(), path.points.end(), requested_s_m,
                         [](const ReferencePathPoint& point, double target_s)
                         { return point.s_m < target_s; });

    if (it == path.points.begin())
    {
        return lerp_points(path.points[0], path.points[1], requested_s_m);
    }

    if (it == path.points.end())
    {
        const ReferencePathPoint closing_endpoint{
            path.points.front().x_m,
            path.points.front().y_m,
            path.length_m,
        };

        return lerp_points(path.points.back(), closing_endpoint, requested_s_m);
    }

    return lerp_points(*(it - 1), *it, requested_s_m);
}

}  // namespace planning
