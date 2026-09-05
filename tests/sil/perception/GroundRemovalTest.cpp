#include <gtest/gtest.h>

#include <GroundRemoval.hpp>

#include <cmath>
#include <utility>

namespace perception
{
namespace
{

PointXYZI make_point(const float x, const float y, const float z,
                     const float intensity = 1.0F)
{
    PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    return point;
}

StampedPointCloud make_cloud(PointCloud points)
{
    StampedPointCloud cloud;
    cloud.timestamp_ns = 100;
    cloud.frame = transforms::FrameId::Lidar;
    cloud.points = std::move(points);
    return cloud;
}

GroundRemovalParams make_one_cell_params()
{
    GroundRemovalParams params;

    params.grid_min_range_m = 0.0;
    params.grid_max_range_m = 2.0;
    params.radial_bin_size_m = 2.0;

    params.grid_min_theta_rad = -0.1;
    params.grid_max_theta_rad = 0.1;
    params.angular_bin_size_rad = 0.2;

    params.flat_ground_z_max_m = -0.15;

    params.ground_percentile = 0.15;
    params.min_points_per_cell = 3;
    params.non_ground_height_threshold_m = 0.05;

    return params;
}

TEST(GroundRemovalTest, ClassifiesHighPointAboveLocalGroundAsNonGround)
{
    const GroundRemovalParams params = make_one_cell_params();

    PointCloud input;
    input.push_back(make_point(1.0F, 0.0F, 0.00F));
    input.push_back(make_point(1.0F, 0.0F, 0.01F));
    input.push_back(make_point(1.0F, 0.0F, 0.02F));
    input.push_back(make_point(1.0F, 0.0F, 0.25F));

    const GroundRemovalResult result = remove_ground(make_cloud(input), params);

    EXPECT_EQ(result.debug.input_points, 4U);
    EXPECT_EQ(result.ground_points.points.size(), 3U);
    EXPECT_EQ(result.non_ground_points.points.size(), 1U);

    EXPECT_EQ(result.debug.ground_points, 3U);
    EXPECT_EQ(result.debug.non_ground_points, 1U);

    ASSERT_EQ(result.debug.cells.size(), 1U);

    const GroundCellDebug& cell = result.debug.cells.at(0);

    EXPECT_TRUE(cell.valid);
    EXPECT_EQ(cell.num_points, 4U);
    EXPECT_NEAR(cell.estimated_ground_z_m, 0.0, 1e-9);

    ASSERT_EQ(result.non_ground_points.points.size(), 1U);
    EXPECT_FLOAT_EQ(result.non_ground_points.points.at(0).z, 0.25F);
    EXPECT_EQ(result.non_ground_points.timestamp_ns, 100);
    EXPECT_EQ(result.non_ground_points.frame, transforms::FrameId::Lidar);
}

TEST(GroundRemovalTest, UsesFlatFallbackForCellsWithTooFewPoints)
{
    GroundRemovalParams params = make_one_cell_params();

    params.min_points_per_cell = 3;
    params.flat_ground_z_max_m = -0.10;
    params.non_ground_height_threshold_m = 0.05;

    PointCloud input;
    input.push_back(make_point(1.0F, 0.0F, -0.20F));
    input.push_back(make_point(1.0F, 0.0F, 0.00F));

    const GroundRemovalResult result = remove_ground(make_cloud(input), params);

    EXPECT_EQ(result.debug.input_points, 2U);
    EXPECT_EQ(result.ground_points.points.size(), 1U);
    EXPECT_EQ(result.non_ground_points.points.size(), 1U);

    ASSERT_EQ(result.debug.cells.size(), 1U);

    const GroundCellDebug& cell = result.debug.cells.at(0);

    EXPECT_FALSE(cell.valid);
    EXPECT_EQ(cell.num_points, 2U);
    EXPECT_NEAR(cell.estimated_ground_z_m, -0.10, 1e-9);

    EXPECT_EQ(result.debug.valid_cells, 0U);
    EXPECT_EQ(result.debug.invalid_cells, 1U);

    ASSERT_EQ(result.ground_points.points.size(), 1U);
    ASSERT_EQ(result.non_ground_points.points.size(), 1U);

    EXPECT_FLOAT_EQ(result.ground_points.points.at(0).z, -0.20F);
    EXPECT_FLOAT_EQ(result.non_ground_points.points.at(0).z, 0.00F);
}

TEST(GroundRemovalTest, IgnoresPointsOutsideConfiguredRangeAndAngle)
{
    GroundRemovalParams params = make_one_cell_params();

    params.min_points_per_cell = 1;

    PointCloud input;
    input.push_back(make_point(1.0F, 0.0F, 0.0F));
    input.push_back(make_point(2.1F, 0.0F, 1.0F));
    input.push_back(make_point(1.0F, 1.0F, 1.0F));

    const GroundRemovalResult result = remove_ground(make_cloud(input), params);

    EXPECT_EQ(result.debug.input_points, 3U);
    EXPECT_EQ(result.ground_points.points.size(), 1U);
    EXPECT_EQ(result.non_ground_points.points.size(), 0U);

    ASSERT_EQ(result.debug.cells.size(), 1U);
    EXPECT_EQ(result.debug.cells.at(0).num_points, 1U);
    EXPECT_TRUE(result.debug.cells.at(0).valid);
}

TEST(GroundRemovalTest, BuildsExpectedDebugGridBounds)
{
    GroundRemovalParams params;

    params.grid_min_range_m = 0.0;
    params.grid_max_range_m = 2.0;
    params.radial_bin_size_m = 1.0;

    params.grid_min_theta_rad = -1.0;
    params.grid_max_theta_rad = 1.0;
    params.angular_bin_size_rad = 1.0;

    params.min_points_per_cell = 1;

    const GroundRemovalResult result = remove_ground(make_cloud({}), params);

    ASSERT_EQ(result.debug.cells.size(), 4U);

    EXPECT_EQ(result.debug.valid_cells, 0U);
    EXPECT_EQ(result.debug.invalid_cells, 4U);

    const GroundCellDebug& cell_00 = result.debug.cells.at(0);
    EXPECT_NEAR(cell_00.r_min_m, 0.0, 1e-9);
    EXPECT_NEAR(cell_00.r_max_m, 1.0, 1e-9);
    EXPECT_NEAR(cell_00.theta_min_rad, -1.0, 1e-9);
    EXPECT_NEAR(cell_00.theta_max_rad, 0.0, 1e-9);

    const GroundCellDebug& cell_01 = result.debug.cells.at(1);
    EXPECT_NEAR(cell_01.r_min_m, 0.0, 1e-9);
    EXPECT_NEAR(cell_01.r_max_m, 1.0, 1e-9);
    EXPECT_NEAR(cell_01.theta_min_rad, 0.0, 1e-9);
    EXPECT_NEAR(cell_01.theta_max_rad, 1.0, 1e-9);

    const GroundCellDebug& cell_10 = result.debug.cells.at(2);
    EXPECT_NEAR(cell_10.r_min_m, 1.0, 1e-9);
    EXPECT_NEAR(cell_10.r_max_m, 2.0, 1e-9);
    EXPECT_NEAR(cell_10.theta_min_rad, -1.0, 1e-9);
    EXPECT_NEAR(cell_10.theta_max_rad, 0.0, 1e-9);
}

}  // namespace
}  // namespace perception
