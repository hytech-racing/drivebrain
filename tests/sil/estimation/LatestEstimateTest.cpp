#include <gtest/gtest.h>

#include <LatestEstimate.hpp>

namespace estimation
{

constexpr double kTolerance = 1e-9;

StateEstimate make_state_estimate(double value)
{
    StateEstimate estimate;

    estimate.timestamp_ns = 1;
    estimate.sequence = 1;

    estimate.x_odom_m = value;
    estimate.y_odom_m = value;
    estimate.yaw_odom_rad = value;

    estimate.vx_vehicle_flu_mps = value;
    estimate.vy_vehicle_flu_mps = value;
    estimate.yaw_rate_vehicle_flu_radps = value;

    estimate.initialized = true;

    return estimate;
}

TEST(LatestEstimateTest, EmptyLatestEstimate)
{
    LatestEstimate latest_estimate;

    const std::optional<StateEstimate> latest = latest_estimate.latest();

    EXPECT_EQ(latest, std::nullopt);
}

TEST(LatestEstimateTest, SingleEstimate)
{
    LatestEstimate latest_estimate;
    const double estimate_value = 1.0;
    StateEstimate temp_estimate = make_state_estimate(estimate_value);

    latest_estimate.store(temp_estimate);

    const std::optional<StateEstimate> latest = latest_estimate.latest();

    EXPECT_EQ(latest.has_value(), true);

    EXPECT_NEAR(latest->x_odom_m, estimate_value, kTolerance);
    EXPECT_NEAR(latest->y_odom_m, estimate_value, kTolerance);
    EXPECT_NEAR(latest->yaw_odom_rad, estimate_value, kTolerance);

    EXPECT_NEAR(latest->vx_vehicle_flu_mps, estimate_value, kTolerance);
    EXPECT_NEAR(latest->vy_vehicle_flu_mps, estimate_value, kTolerance);
    EXPECT_NEAR(latest->yaw_rate_vehicle_flu_radps, estimate_value, kTolerance);
}

TEST(LatestEstimateTest, ReplaceEstimate)
{
    LatestEstimate latest_estimate;
    const double first_estimate_value = 1.0;
    StateEstimate first_estimate = make_state_estimate(first_estimate_value);

    latest_estimate.store(first_estimate);

    const double second_estimate_value = 2.0;
    StateEstimate second_estimate = make_state_estimate(second_estimate_value);

    latest_estimate.store(second_estimate);

    const std::optional<StateEstimate> latest = latest_estimate.latest();

    EXPECT_EQ(latest.has_value(), true);

    EXPECT_NEAR(latest->x_odom_m, second_estimate_value, kTolerance);
    EXPECT_NEAR(latest->y_odom_m, second_estimate_value, kTolerance);
    EXPECT_NEAR(latest->yaw_odom_rad, second_estimate_value, kTolerance);

    EXPECT_NEAR(latest->vx_vehicle_flu_mps, second_estimate_value, kTolerance);
    EXPECT_NEAR(latest->vy_vehicle_flu_mps, second_estimate_value, kTolerance);
    EXPECT_NEAR(latest->yaw_rate_vehicle_flu_radps, second_estimate_value,
                kTolerance);
}

}  // namespace estimation
