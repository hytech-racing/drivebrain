#include <gtest/gtest.h>

#include <SlamVisualizationAdapters.hpp>

#include <cmath>
#include <vector>

namespace adapters
{
namespace
{

slam::PoseEstimate make_pose_estimate()
{
    slam::PoseEstimate pose;
    pose.initial_pose_map_from_base = transforms::Pose2D{1.0, 2.0, 0.1};
    pose.optimized_pose_map_from_base = transforms::Pose2D{3.0, 4.0, 0.2};
    return pose;
}

slam::LandmarkEstimate make_landmark_estimate()
{
    slam::LandmarkEstimate landmark;
    landmark.landmark_id = 42U;
    landmark.initial_position_map = transforms::Point2D{1.0, 2.0};
    landmark.optimized_position_map = transforms::Point2D{3.0, 4.0};
    return landmark;
}

slam::FrontendResult make_frontend_result()
{
    slam::LandmarkObservation observation;
    observation.landmark_id = 7U;
    observation.measurement_base_m = transforms::Point2D{5.0, 6.0};
    observation.association = slam::LandmarkAssociation::ExistingMapLandmark;
    observation.residual_m = 0.25;

    slam::FrontendResult result;
    result.timestamp_ns = 123;
    result.landmark_observations.push_back(observation);
    return result;
}

TEST(SlamVisualizationAdaptersTest, EmptyPoseMarkersReturnClearingScene)
{
    const auto scene = to_foxglove_slam_pose_markers({}, true, 123,
                                                     "optimized_poses");

    ASSERT_NE(scene, nullptr);
    EXPECT_EQ(scene->deletions_size(), 1);
    EXPECT_EQ(scene->entities_size(), 0);
}

TEST(SlamVisualizationAdaptersTest, OptimizedPoseMarkersUseMapFrame)
{
    const auto scene = to_foxglove_slam_pose_markers(
        {make_pose_estimate()}, true, 123, "optimized_poses");

    ASSERT_NE(scene, nullptr);
    ASSERT_EQ(scene->entities_size(), 1);
    const foxglove::SceneEntity& entity = scene->entities(0);
    EXPECT_EQ(entity.frame_id(), "map");
    EXPECT_EQ(entity.id(), "optimized_poses");
    ASSERT_EQ(entity.lines_size(), 1);
    ASSERT_EQ(entity.lines(0).points_size(), 1);
    EXPECT_EQ(entity.lines(0).type(), foxglove::LinePrimitive::LINE_STRIP);
    EXPECT_DOUBLE_EQ(entity.lines(0).points(0).x(), 3.0);
    EXPECT_DOUBLE_EQ(entity.lines(0).points(0).y(), 4.0);
}

TEST(SlamVisualizationAdaptersTest, PoseMarkersKeepNewestTenThousandPoints)
{
    std::vector<slam::PoseEstimate> poses;
    poses.reserve(10001U);

    for (std::size_t pose_index = 0; pose_index < 10001U; ++pose_index)
    {
        slam::PoseEstimate pose;
        pose.optimized_pose_map_from_base = transforms::Pose2D{
            static_cast<double>(pose_index), 0.0, 0.0};
        poses.push_back(pose);
    }

    const auto scene = to_foxglove_slam_pose_markers(
        poses, true, 123, "optimized_poses");

    ASSERT_NE(scene, nullptr);
    ASSERT_EQ(scene->entities_size(), 1);
    const foxglove::SceneEntity& entity = scene->entities(0);
    ASSERT_EQ(entity.lines_size(), 1);
    EXPECT_EQ(entity.lines(0).points_size(), 10000);
    EXPECT_DOUBLE_EQ(entity.lines(0).points(0).x(), 1.0);
    EXPECT_DOUBLE_EQ(entity.lines(0).points(9999).x(), 10000.0);
}

TEST(SlamVisualizationAdaptersTest, LandmarkMarkersUseRequestedEstimate)
{
    const auto scene = to_foxglove_slam_landmark_markers(
        {make_landmark_estimate()}, false, 123, "initial_landmarks");

    ASSERT_NE(scene, nullptr);
    ASSERT_EQ(scene->entities_size(), 1);
    const foxglove::SceneEntity& entity = scene->entities(0);
    ASSERT_EQ(entity.spheres_size(), 1);
    EXPECT_DOUBLE_EQ(entity.spheres(0).pose().position().x(), 1.0);
    EXPECT_DOUBLE_EQ(entity.spheres(0).pose().position().y(), 2.0);
    EXPECT_DOUBLE_EQ(entity.spheres(0).color().r(), 1.0);
    EXPECT_DOUBLE_EQ(entity.spheres(0).color().g(), 1.0);
}

TEST(SlamVisualizationAdaptersTest, LandmarkTextLabelsIds)
{
    const auto scene = to_foxglove_slam_landmark_text(
        {make_landmark_estimate()}, 123);

    ASSERT_NE(scene, nullptr);
    ASSERT_EQ(scene->entities_size(), 1);
    const foxglove::SceneEntity& entity = scene->entities(0);
    ASSERT_EQ(entity.texts_size(), 1);
    EXPECT_EQ(entity.texts(0).text(), "id: 42");
    EXPECT_DOUBLE_EQ(entity.texts(0).pose().position().x(), 3.0);
}

TEST(SlamVisualizationAdaptersTest, FrontendAssociationMarkersUseAssociationColor)
{
    const auto scene = to_foxglove_frontend_association_markers(
        make_frontend_result(), "base_link");

    ASSERT_NE(scene, nullptr);
    ASSERT_EQ(scene->entities_size(), 1);
    const foxglove::SceneEntity& entity = scene->entities(0);
    EXPECT_EQ(entity.frame_id(), "base_link");
    ASSERT_EQ(entity.spheres_size(), 1);
    EXPECT_DOUBLE_EQ(entity.spheres(0).pose().position().x(), 5.0);
    EXPECT_DOUBLE_EQ(entity.spheres(0).color().g(), 1.0);
}

TEST(SlamVisualizationAdaptersTest, MapOdomTransformMapsEveryField)
{
    const auto transform = to_foxglove_map_odom_transform(
        transforms::Pose2D{1.0, 2.0, 0.5}, 123);

    ASSERT_NE(transform, nullptr);
    EXPECT_EQ(transform->parent_frame_id(), "map");
    EXPECT_EQ(transform->child_frame_id(), "odom");
    EXPECT_DOUBLE_EQ(transform->translation().x(), 1.0);
    EXPECT_DOUBLE_EQ(transform->translation().y(), 2.0);
    EXPECT_DOUBLE_EQ(transform->translation().z(), 0.0);
    EXPECT_DOUBLE_EQ(transform->rotation().z(), std::sin(0.25));
    EXPECT_DOUBLE_EQ(transform->rotation().w(), std::cos(0.25));
}

}  // namespace
}  // namespace adapters
