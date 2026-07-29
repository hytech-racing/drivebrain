
#pragma once

#include <cstddef>
#include <vector>

#include "PointCloudTypes.hpp"

namespace perception
{

struct GroundRemovalParams
{
    double min_range_m{1.0};
    double max_range_m{20.0};
    double min_theta_rad{-90.0 * 3.1415926 / 180.0};
    double max_theta_rad{90.0 * 3.1415926 / 180.0};

    // flat-z fallback
    double flat_ground_z_max_m{-0.5};

    // adaptive local ground estimation
    double radial_bin_size_m{0.5};
    double angular_bin_size_rad{5.0 * 3.1415926 / 180.0};
    double ground_percentile{0.15};
    std::size_t min_points_per_cell{5};

    double non_ground_height_threshold_m{0.02};
};

struct GroundCellDebug
{
    double r_min_m{};
    double r_max_m{};
    double theta_min_rad{};
    double theta_max_rad{};

    double estimated_ground_z_m{};
    std::size_t num_points{};

    bool valid{};
};

struct GroundRemovalDebug
{
    std::size_t input_points{};
    std::size_t ground_points{};
    std::size_t non_ground_points{};
    std::size_t valid_cells{};
    std::size_t invalid_cells{};

    std::vector<GroundCellDebug> cells;
};

struct GroundRemovalResult
{
    StampedPointCloud ground_points;
    StampedPointCloud non_ground_points;
    GroundRemovalDebug debug;
};

GroundRemovalResult remove_ground(const StampedPointCloud& input,
                                  const GroundRemovalParams& params);

}  // namespace perception
