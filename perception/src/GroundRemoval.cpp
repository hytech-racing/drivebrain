
#include "GroundRemoval.hpp"

#include <algorithm>
#include <cmath>

namespace
{

struct GroundCell
{
    std::vector<std::size_t> point_indices;
    std::vector<double> z_values;

    double estimated_ground_z_m{};
    bool valid{};
};

struct GridShape
{
    std::size_t num_radial_bins{};
    std::size_t num_angular_bins{};
};

std::size_t flatten_cell_index(const std::size_t radial_index,
                               const std::size_t angular_index,
                               const std::size_t num_angular_bins)
{
    return radial_index * num_angular_bins + angular_index;
}

}  // namespace

namespace perception
{
GridShape compute_grid_shape(const GroundRemovalParams& params);

void assign_points_to_cells(std::vector<GroundCell>& cells,
                            const PointCloud& input,
                            const GroundRemovalParams& params,
                            const GridShape& grid);

void estimate_ground_per_cell(std::vector<GroundCell>& cells,
                              const GroundRemovalParams& params);

void classify_points(GroundRemovalResult& result,
                     const std::vector<GroundCell>& cells,
                     const PointCloud& input,
                     const GroundRemovalParams& params);

std::vector<GroundCellDebug> make_cell_debug(
    const std::vector<GroundCell>& cells, const GroundRemovalParams& params,
    const GridShape& grid);

GroundRemovalResult remove_ground(const StampedPointCloud& input,
                                  const GroundRemovalParams& params)
{
    GroundRemovalResult result;
    const PointCloud& input_point_cloud = input.points;

    result.debug.input_points = input_point_cloud.size();

    const GridShape grid = compute_grid_shape(params);

    std::vector<GroundCell> cells(grid.num_radial_bins * grid.num_angular_bins);

    assign_points_to_cells(cells, input_point_cloud, params, grid);

    estimate_ground_per_cell(cells, params);

    classify_points(result, cells, input_point_cloud, params);

    result.debug.ground_points = result.ground_points.points.size();
    result.debug.non_ground_points = result.non_ground_points.points.size();
    result.debug.cells = make_cell_debug(cells, params, grid);

    result.ground_points.frame = input.frame;
    result.ground_points.timestamp_ns = input.timestamp_ns;

    result.non_ground_points.frame = input.frame;
    result.non_ground_points.timestamp_ns = input.timestamp_ns;

    for (const GroundCellDebug& cell_debug : result.debug.cells)
    {
        if (cell_debug.valid)
        {
            ++result.debug.valid_cells;
        }
        else
        {
            ++result.debug.invalid_cells;
        }
    }

    return result;
}

void assign_points_to_cells(std::vector<GroundCell>& cells,
                            const PointCloud& input,
                            const GroundRemovalParams& params,
                            const GridShape& grid)
{
    for (std::size_t point_index = 0; point_index < input.size(); ++point_index)
    {
        const PointXYZI& current_point = input.at(point_index);

        const double r = std::sqrt((current_point.x * current_point.x) +
                                   (current_point.y * current_point.y));

        const double theta = std::atan2(current_point.y, current_point.x);

        if (r < params.min_range_m || r > params.max_range_m)
        {
            continue;
        }

        if (theta < params.min_theta_rad || theta > params.max_theta_rad)
        {
            continue;
        }

        const std::size_t radial_index = static_cast<std::size_t>(
            std::floor((r - params.min_range_m) / params.radial_bin_size_m));

        const std::size_t angular_index = static_cast<std::size_t>(std::floor(
            (theta - params.min_theta_rad) / params.angular_bin_size_rad));

        if (radial_index >= grid.num_radial_bins ||
            angular_index >= grid.num_angular_bins)
        {
            continue;
        }

        const std::size_t cell_flat_index = flatten_cell_index(
            radial_index, angular_index, grid.num_angular_bins);

        GroundCell& cell = cells.at(cell_flat_index);
        cell.point_indices.push_back(point_index);
        cell.z_values.push_back(current_point.z);
    }
}

void estimate_ground_per_cell(std::vector<GroundCell>& cells,
                              const GroundRemovalParams& params)
{
    for (GroundCell& cell : cells)
    {
        if (cell.z_values.size() < params.min_points_per_cell)
        {
            cell.valid = false;
            cell.estimated_ground_z_m = params.flat_ground_z_max_m;
            continue;
        }

        std::sort(cell.z_values.begin(), cell.z_values.end());

        std::size_t percentile_index = static_cast<std::size_t>(
            floor(params.ground_percentile * (cell.z_values.size() - 1)));

        cell.estimated_ground_z_m = cell.z_values.at(percentile_index);

        cell.valid = true;
    }
}

void classify_points(GroundRemovalResult& result,
                     const std::vector<GroundCell>& cells,
                     const PointCloud& input, const GroundRemovalParams& params)
{
    for (const GroundCell& cell : cells)
    {
        for (const std::size_t point_index : cell.point_indices)
        {
            const PointXYZI& curr_point = input.at(point_index);
            if (curr_point.z > (cell.estimated_ground_z_m +
                                params.non_ground_height_threshold_m))
            {
                result.non_ground_points.points.push_back(curr_point);
            }
            else
            {
                result.ground_points.points.push_back(curr_point);
            }
        }
    }
}

std::vector<GroundCellDebug> make_cell_debug(
    const std::vector<GroundCell>& cells, const GroundRemovalParams& params,
    const GridShape& grid)
{
    std::vector<GroundCellDebug> debug_cells;
    debug_cells.reserve(cells.size());

    for (std::size_t radial_index = 0; radial_index < grid.num_radial_bins;
         ++radial_index)
    {
        for (std::size_t angular_index = 0;
             angular_index < grid.num_angular_bins; ++angular_index)
        {
            const std::size_t cell_index = flatten_cell_index(
                radial_index, angular_index, grid.num_angular_bins);

            const GroundCell& cell = cells.at(cell_index);

            GroundCellDebug debug;

            debug.r_min_m =
                params.min_range_m +
                static_cast<double>(radial_index) * params.radial_bin_size_m;

            debug.r_max_m = debug.r_min_m + params.radial_bin_size_m;

            debug.theta_min_rad =
                params.min_theta_rad + static_cast<double>(angular_index) *
                                           params.angular_bin_size_rad;

            debug.theta_max_rad =
                debug.theta_min_rad + params.angular_bin_size_rad;

            debug.estimated_ground_z_m = cell.estimated_ground_z_m;
            debug.num_points = cell.point_indices.size();
            debug.valid = cell.valid;

            debug_cells.push_back(debug);
        }
    }

    return debug_cells;
}

GridShape compute_grid_shape(const GroundRemovalParams& params)
{
    GridShape grid;

    grid.num_angular_bins = static_cast<std::size_t>(
        std::ceil((params.max_theta_rad - params.min_theta_rad) /
                  params.angular_bin_size_rad));

    grid.num_radial_bins = static_cast<std::size_t>(std::ceil(
        (params.max_range_m - params.min_range_m) / params.radial_bin_size_m));

    return grid;
}

}  // namespace perception
