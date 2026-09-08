#include <gtest/gtest.h>

#include <EstimatorMessageAdapters.hpp>
#include <limits>
#include <optional>

namespace adapters
{
namespace
{

constexpr double kTolerance = 1e-9;

hytech_msgs::VnImuData make_imu_message()
{
    hytech_msgs::VnImuData message;
    message.set_vn_time_startup_ns(123);
    message.mutable_comp_no_gravity_accel_vehicle_flu_m_ss()->set_x(1.0F);
    message.mutable_comp_no_gravity_accel_vehicle_flu_m_ss()->set_y(2.0F);
    message.mutable_comp_gyro_vehicle_flu_rad_s()->set_z(3.0F);
    return message;
}

hytech_msgs::GssData make_gss_message()
{
    hytech_msgs::GssData message;
    message.set_sensor_timestamp_ns(456);
    message.mutable_vel_sensor_flu_mps()->set_x(4.0F);
    message.mutable_vel_sensor_flu_mps()->set_y(5.0F);
    return message;
}

TEST(EstimatorMessageAdaptersTest, ValidImuMapsEveryField)
{
    const hytech_msgs::VnImuData message = make_imu_message();

    const std::optional<estimation::ImuMeasurement> measurement =
        to_imu_measurement(message);

    ASSERT_TRUE(measurement.has_value());
    EXPECT_EQ(measurement->timestamp_ns, 123U);
    EXPECT_NEAR(measurement->ax_vehicle_flu_mps2, 1.0, kTolerance);
    EXPECT_NEAR(measurement->ay_vehicle_flu_mps2, 2.0, kTolerance);
    EXPECT_NEAR(measurement->yaw_rate_vehicle_flu_radps, 3.0, kTolerance);
}

TEST(EstimatorMessageAdaptersTest, ZeroImuTimestampIsRejected)
{
    hytech_msgs::VnImuData message = make_imu_message();
    message.set_vn_time_startup_ns(0);

    EXPECT_EQ(to_imu_measurement(message), std::nullopt);
}

TEST(EstimatorMessageAdaptersTest, NanImuValueIsRejected)
{
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();

    hytech_msgs::VnImuData nan_x = make_imu_message();
    nan_x.mutable_comp_no_gravity_accel_vehicle_flu_m_ss()->set_x(nan);
    EXPECT_EQ(to_imu_measurement(nan_x), std::nullopt);

    hytech_msgs::VnImuData nan_y = make_imu_message();
    nan_y.mutable_comp_no_gravity_accel_vehicle_flu_m_ss()->set_y(nan);
    EXPECT_EQ(to_imu_measurement(nan_y), std::nullopt);

    hytech_msgs::VnImuData nan_z = make_imu_message();
    nan_z.mutable_comp_gyro_vehicle_flu_rad_s()->set_z(nan);
    EXPECT_EQ(to_imu_measurement(nan_z), std::nullopt);
}

TEST(EstimatorMessageAdaptersTest, ValidGssMapsEveryField)
{
    const hytech_msgs::GssData message = make_gss_message();

    const std::optional<estimation::GssMeasurement> measurement =
        to_gss_measurement(message);

    ASSERT_TRUE(measurement.has_value());
    EXPECT_EQ(measurement->timestamp_ns, 456U);
    EXPECT_NEAR(measurement->vx_sensor_flu_mps, 4.0, kTolerance);
    EXPECT_NEAR(measurement->vy_sensor_flu_mps, 5.0, kTolerance);
}

TEST(EstimatorMessageAdaptersTest, ZeroGssTimestampIsRejected)
{
    hytech_msgs::GssData message = make_gss_message();
    message.set_sensor_timestamp_ns(0);

    EXPECT_EQ(to_gss_measurement(message), std::nullopt);
}

TEST(EstimatorMessageAdaptersTest, NanGssValueIsRejected)
{
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();

    hytech_msgs::GssData nan_x = make_gss_message();
    nan_x.mutable_vel_sensor_flu_mps()->set_x(nan);
    EXPECT_EQ(to_gss_measurement(nan_x), std::nullopt);

    hytech_msgs::GssData nan_y = make_gss_message();
    nan_y.mutable_vel_sensor_flu_mps()->set_y(nan);
    EXPECT_EQ(to_gss_measurement(nan_y), std::nullopt);
}

}  // namespace
}  // namespace adapters
