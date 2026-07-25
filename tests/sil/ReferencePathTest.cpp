#include <gtest/gtest.h>

#include <ReferencePath.hpp>

namespace planning
{
namespace
{

constexpr double kTolerance = 1e-9;

ReferencePath make_square_path()
{
    ReferencePath path;

    path.points = {
        ReferencePathPoint{0.0, 0.0, 0.0},
        ReferencePathPoint{10.0, 0.0, 10.0},
        ReferencePathPoint{10.0, 10.0, 20.0},
        ReferencePathPoint{0.0, 10.0, 30.0},
    };

    path.length_m = 40.0;

    return path;
}

TEST(ReferencePathProjectionTest, ProjectsOntoSegmentInterior)
{
    const ReferencePath path = make_square_path();
    const Point2D vehicle_position{4.0, -3.0};

    const auto projection = project_onto_path(vehicle_position, path);

    EXPECT_NEAR(projection.value().point.x_m, 4.0, kTolerance);
    EXPECT_NEAR(projection.value().point.y_m, 0.0, kTolerance);
    EXPECT_NEAR(projection.value().point.s_m, 4.0, kTolerance);
    EXPECT_NEAR(projection.value().distance_squared, 9.0, kTolerance);
}

TEST(ReferencePathProjectionTest, ClampsProjectionBeforeSegmentStart)
{
    const ReferencePath path = make_square_path();
    const Point2D vehicle_position{-2.0, -1.0};

    const auto projection = project_onto_path(vehicle_position, path);

    EXPECT_NEAR(projection.value().point.x_m, 0.0, kTolerance);
    EXPECT_NEAR(projection.value().point.y_m, 0.0, kTolerance);
    EXPECT_NEAR(projection.value().point.s_m, 0.0, kTolerance);
    EXPECT_NEAR(projection.value().distance_squared, 5.0, kTolerance);
}

TEST(ReferencePathProjectionTest, ClampsProjectionAfterSegmentEnd)
{
    const ReferencePath path = make_square_path();
    const Point2D vehicle_position{12.0, -1.0};

    const auto projection = project_onto_path(vehicle_position, path);

    EXPECT_NEAR(projection.value().point.x_m, 10.0, kTolerance);
    EXPECT_NEAR(projection.value().point.y_m, 0.0, kTolerance);
    EXPECT_NEAR(projection.value().point.s_m, 10.0, kTolerance);
    EXPECT_NEAR(projection.value().distance_squared, 5.0, kTolerance);
}

TEST(ReferencePathProjectionTest, ProjectsOntoClosingSegment)
{
    const ReferencePath path = make_square_path();
    const Point2D vehicle_position{-1.0, 5.0};

    const auto projection = project_onto_path(vehicle_position, path);

    EXPECT_NEAR(projection.value().point.x_m, 0.0, kTolerance);
    EXPECT_NEAR(projection.value().point.y_m, 5.0, kTolerance);
    EXPECT_NEAR(projection.value().point.s_m, 35.0, kTolerance);
    EXPECT_NEAR(projection.value().distance_squared, 1.0, kTolerance);
}

TEST(ReferencePathProjectionTest, EmptyPathReturnsNoProjection)
{
    const ReferencePath path;
    const Point2D vehicle_position{0.0, 0.0};

    EXPECT_FALSE(project_onto_path(vehicle_position, path).has_value());
}

TEST(ReferencePathProjectionTest, InterpolateAtOrdinarySegment)
{
    const ReferencePath path = make_square_path();

    const auto interpolated_point = interpolate_at_s(path, 5.0);

    EXPECT_NEAR(interpolated_point->x_m, 5.0, kTolerance);
    EXPECT_NEAR(interpolated_point->y_m, 0.0, kTolerance);
}

TEST(ReferencePathProjectionTest, InterpolateAtExactPoint)
{
    const ReferencePath path = make_square_path();

    const auto interpolated_point = interpolate_at_s(path, 10.0);

    EXPECT_NEAR(interpolated_point->x_m, 10.0, kTolerance);
    EXPECT_NEAR(interpolated_point->y_m, 0.0, kTolerance);
}

TEST(ReferencePathProjectionTest, InterpolateAtClosingSegment)
{
    const ReferencePath path = make_square_path();

    const auto interpolated_point = interpolate_at_s(path, 35.0);

    EXPECT_NEAR(interpolated_point->x_m, 0.0, kTolerance);
    EXPECT_NEAR(interpolated_point->y_m, 5.0, kTolerance);
}

TEST(ReferencePathProjectionTest, WrapsPastPathLength)
{
    const ReferencePath path = make_square_path();

    const auto interpolated_point = interpolate_at_s(path, 45.0);

    EXPECT_NEAR(interpolated_point->x_m, 5.0, kTolerance);
    EXPECT_NEAR(interpolated_point->y_m, 0.0, kTolerance);
}

TEST(ReferencePathProjectionTest, WrapsNegativeArcLength)
{
    const ReferencePath path = make_square_path();

    const auto interpolated_point = interpolate_at_s(path, -5);

    EXPECT_NEAR(interpolated_point->x_m, 0.0, kTolerance);
    EXPECT_NEAR(interpolated_point->y_m, 5.0, kTolerance);
}

}  // namespace
}  // namespace planning
