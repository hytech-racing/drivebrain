#include "SlamVisualizationAdapters.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>

namespace adapters
{
namespace
{

constexpr const char* kMapFrame = "map";
constexpr std::size_t kMaximumSlamPathPoints = 10000;

void set_timestamp(google::protobuf::Timestamp* timestamp,
                   const std::int64_t timestamp_ns)
{
    timestamp->set_seconds(static_cast<int64_t>(timestamp_ns / 1'000'000'000));
    timestamp->set_nanos(static_cast<int32_t>(timestamp_ns % 1'000'000'000));
}

std::shared_ptr<foxglove::SceneUpdate> make_clearing_scene(
    const std::int64_t timestamp_ns)
{
    auto scene = std::make_shared<foxglove::SceneUpdate>();

    auto* deletion = scene->add_deletions();
    deletion->set_type(foxglove::SceneEntityDeletion_Type_ALL);
    set_timestamp(deletion->mutable_timestamp(), timestamp_ns);

    return scene;
}

foxglove::SceneEntity* add_entity(foxglove::SceneUpdate* scene,
                                  std::string_view frame_id,
                                  std::string_view entity_id,
                                  const std::int64_t timestamp_ns)
{
    auto* entity = scene->add_entities();
    entity->set_frame_id(std::string(frame_id));
    entity->set_id(std::string(entity_id));
    set_timestamp(entity->mutable_timestamp(), timestamp_ns);
    return entity;
}

void set_identity_orientation(foxglove::Quaternion* orientation)
{
    orientation->set_w(1.0);
}

void set_yaw_orientation(foxglove::Quaternion* orientation, const double yaw_rad)
{
    orientation->set_x(0.0);
    orientation->set_y(0.0);
    orientation->set_z(std::sin(0.5 * yaw_rad));
    orientation->set_w(std::cos(0.5 * yaw_rad));
}

void set_color(foxglove::Color* color, const double r, const double g,
               const double b, const double a)
{
    color->set_r(r);
    color->set_g(g);
    color->set_b(b);
    color->set_a(a);
}

transforms::Pose2D pose_for_estimate(const slam::PoseEstimate& pose,
                                     const bool optimized)
{
    return optimized ? pose.optimized_pose_map_from_base
                     : pose.initial_pose_map_from_base;
}

transforms::Point2D point_for_estimate(const slam::LandmarkEstimate& landmark,
                                       const bool optimized)
{
    return optimized ? landmark.optimized_position_map
                     : landmark.initial_position_map;
}

void add_pose_marker(foxglove::LinePrimitive* line,
                     const transforms::Pose2D& pose)
{
    auto* point = line->add_points();
    point->set_x(pose.x_m);
    point->set_y(pose.y_m);
    point->set_z(0.05);
}

void add_landmark_marker(foxglove::SceneEntity* entity,
                         const transforms::Point2D& position_map,
                         const bool optimized)
{
    auto* sphere = entity->add_spheres();
    auto* position = sphere->mutable_pose()->mutable_position();
    position->set_x(position_map.x_m);
    position->set_y(position_map.y_m);
    position->set_z(0.0);
    set_identity_orientation(sphere->mutable_pose()->mutable_orientation());
    sphere->mutable_size()->set_x(0.2);
    sphere->mutable_size()->set_y(0.2);
    sphere->mutable_size()->set_z(0.2);

    if (optimized)
    {
        set_color(sphere->mutable_color(), 0.0, 1.0, 0.0, 1.0);
    }
    else
    {
        set_color(sphere->mutable_color(), 1.0, 1.0, 0.0, 1.0);
    }
}

void association_color(const slam::LandmarkAssociation association,
                       double& r, double& g, double& b)
{
    switch (association)
    {
        case slam::LandmarkAssociation::ExistingMapLandmark:
            r = 0.0;
            g = 1.0;
            b = 0.0;
            return;
        case slam::LandmarkAssociation::PendingLandmark:
            r = 0.0;
            g = 0.6;
            b = 1.0;
            return;
        case slam::LandmarkAssociation::NewLandmark:
            r = 1.0;
            g = 0.8;
            b = 0.0;
            return;
    }

    r = 1.0;
    g = 1.0;
    b = 1.0;
}

void cone_color(const slam::ConeColor color, double& r, double& g, double& b)
{
    switch (color)
    {
        case slam::ConeColor::Blue:
            r = 0.0;
            g = 0.25;
            b = 1.0;
            return;
        case slam::ConeColor::Yellow:
            r = 1.0;
            g = 0.85;
            b = 0.0;
            return;
        case slam::ConeColor::OrangeSmall:
        case slam::ConeColor::OrangeBig:
            r = 1.0;
            g = 0.45;
            b = 0.0;
            return;
        case slam::ConeColor::Unknown:
            r = 0.75;
            g = 0.75;
            b = 0.75;
            return;
    }

    r = 0.75;
    g = 0.75;
    b = 0.75;
}

std::string cone_color_text(const slam::ConeColor color)
{
    switch (color)
    {
        case slam::ConeColor::Blue:
            return "blue";
        case slam::ConeColor::Yellow:
            return "yellow";
        case slam::ConeColor::OrangeSmall:
            return "orange_small";
        case slam::ConeColor::OrangeBig:
            return "orange_big";
        case slam::ConeColor::Unknown:
            return "unknown";
    }

    return "unknown";
}

std::string planner_landmark_state_text(
    const slam::PlannerLandmarkState state)
{
    switch (state)
    {
        case slam::PlannerLandmarkState::Pending:
            return "pending";
        case slam::PlannerLandmarkState::Optimized:
            return "optimized";
    }

    return "unknown";
}

const char* association_to_string(const slam::LandmarkAssociation association)
{
    switch (association)
    {
        case slam::LandmarkAssociation::ExistingMapLandmark:
            return "existing";
        case slam::LandmarkAssociation::PendingLandmark:
            return "pending";
        case slam::LandmarkAssociation::NewLandmark:
            return "new";
    }

    return "unknown";
}

std::string frontend_observation_text(
    const slam::LandmarkObservation& observation)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "id: " << observation.landmark_id << "\n"
           << "assoc: " << association_to_string(observation.association)
           << "\nresidual: " << observation.residual_m << " m";
    return stream.str();
}

}  // namespace

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_slam_pose_markers(
    const std::vector<slam::PoseEstimate>& poses, const bool optimized,
    const std::int64_t timestamp_ns, std::string_view entity_id)
{
    auto scene = make_clearing_scene(timestamp_ns);
    if (poses.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), kMapFrame, entity_id, timestamp_ns);
    auto* line = entity->add_lines();
    line->set_type(foxglove::LinePrimitive::LINE_STRIP);
    line->mutable_pose()->mutable_orientation()->set_w(1.0);
    line->set_thickness(0.08);
    if (optimized)
    {
        set_color(line->mutable_color(), 0.0, 0.9, 0.2, 1.0);
    }
    else
    {
        set_color(line->mutable_color(), 1.0, 0.8, 0.0, 1.0);
    }

    const std::size_t first_pose_index =
        poses.size() > kMaximumSlamPathPoints
            ? poses.size() - kMaximumSlamPathPoints
            : 0U;

    for (std::size_t pose_index = first_pose_index; pose_index < poses.size();
         ++pose_index)
    {
        add_pose_marker(line, pose_for_estimate(poses[pose_index], optimized));
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_slam_landmark_markers(
    const std::vector<slam::LandmarkEstimate>& landmarks, const bool optimized,
    const std::int64_t timestamp_ns, std::string_view entity_id)
{
    auto scene = make_clearing_scene(timestamp_ns);
    if (landmarks.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), kMapFrame, entity_id, timestamp_ns);
    for (const slam::LandmarkEstimate& landmark : landmarks)
    {
        add_landmark_marker(entity, point_for_estimate(landmark, optimized),
                            optimized);
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_slam_landmark_text(
    const std::vector<slam::LandmarkEstimate>& landmarks,
    const std::int64_t timestamp_ns, std::string_view entity_id)
{
    auto scene = make_clearing_scene(timestamp_ns);
    if (landmarks.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), kMapFrame, entity_id, timestamp_ns);
    for (const slam::LandmarkEstimate& landmark : landmarks)
    {
        const transforms::Point2D position = landmark.optimized_position_map;
        auto* text = entity->add_texts();
        auto* text_position = text->mutable_pose()->mutable_position();
        text_position->set_x(position.x_m);
        text_position->set_y(position.y_m);
        text_position->set_z(0.4);
        set_identity_orientation(text->mutable_pose()->mutable_orientation());
        text->set_billboard(true);
        text->set_font_size(0.12);
        text->set_text("id: " + std::to_string(landmark.landmark_id));
        set_color(text->mutable_color(), 1.0, 1.0, 1.0, 1.0);
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_frontend_association_markers(
    const slam::FrontendResult& result, std::string_view frame_id,
    std::string_view entity_id)
{
    auto scene = make_clearing_scene(result.timestamp_ns);
    if (result.landmark_observations.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), frame_id, entity_id,
                              result.timestamp_ns);
    for (const slam::LandmarkObservation& observation :
         result.landmark_observations)
    {
        double r{};
        double g{};
        double b{};
        association_color(observation.association, r, g, b);

        auto* sphere = entity->add_spheres();
        auto* position = sphere->mutable_pose()->mutable_position();
        position->set_x(observation.measurement_base_m.x_m);
        position->set_y(observation.measurement_base_m.y_m);
        position->set_z(0.15);
        set_identity_orientation(sphere->mutable_pose()->mutable_orientation());
        sphere->mutable_size()->set_x(0.18);
        sphere->mutable_size()->set_y(0.18);
        sphere->mutable_size()->set_z(0.18);
        set_color(sphere->mutable_color(), r, g, b, 0.9);
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_frontend_association_text(
    const slam::FrontendResult& result, std::string_view frame_id,
    std::string_view entity_id)
{
    auto scene = make_clearing_scene(result.timestamp_ns);
    if (result.landmark_observations.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), frame_id, entity_id,
                              result.timestamp_ns);
    for (const slam::LandmarkObservation& observation :
         result.landmark_observations)
    {
        auto* text = entity->add_texts();
        auto* position = text->mutable_pose()->mutable_position();
        position->set_x(observation.measurement_base_m.x_m);
        position->set_y(observation.measurement_base_m.y_m);
        position->set_z(0.5);
        set_identity_orientation(text->mutable_pose()->mutable_orientation());
        text->set_billboard(true);
        text->set_font_size(0.10);
        text->set_text(frontend_observation_text(observation));
        set_color(text->mutable_color(), 1.0, 1.0, 1.0, 1.0);
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_planner_landmark_markers(
    const slam::PlannerMap& map, std::string_view entity_id)
{
    auto scene = make_clearing_scene(map.timestamp_ns);
    auto* entity = add_entity(scene.get(), kMapFrame, entity_id,
                              map.timestamp_ns);

    for (const slam::PlannerLandmark& landmark : map.landmarks)
    {
        double r{};
        double g{};
        double b{};
        cone_color(landmark.color, r, g, b);

        auto* sphere = entity->add_spheres();
        auto* position = sphere->mutable_pose()->mutable_position();
        position->set_x(landmark.position_map_m.x_m);
        position->set_y(landmark.position_map_m.y_m);
        position->set_z(0.15);
        set_identity_orientation(sphere->mutable_pose()->mutable_orientation());

        const bool optimized =
            landmark.state == slam::PlannerLandmarkState::Optimized;
        const double size = optimized ? 0.24 : 0.18;
        sphere->mutable_size()->set_x(size);
        sphere->mutable_size()->set_y(size);
        sphere->mutable_size()->set_z(size);
        set_color(sphere->mutable_color(), r, g, b, optimized ? 1.0 : 0.45);
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_planner_landmark_text(
    const slam::PlannerMap& map, std::string_view entity_id)
{
    auto scene = make_clearing_scene(map.timestamp_ns);
    auto* entity = add_entity(scene.get(), kMapFrame, entity_id,
                              map.timestamp_ns);

    for (const slam::PlannerLandmark& landmark : map.landmarks)
    {
        auto* text = entity->add_texts();
        auto* position = text->mutable_pose()->mutable_position();
        position->set_x(landmark.position_map_m.x_m);
        position->set_y(landmark.position_map_m.y_m);
        position->set_z(0.55);
        set_identity_orientation(text->mutable_pose()->mutable_orientation());
        text->set_billboard(true);
        text->set_font_size(0.10);

        std::ostringstream label;
        label << "id: " << landmark.landmark_id << "\n"
              << planner_landmark_state_text(landmark.state) << "\n"
              << cone_color_text(landmark.color) << " " << std::fixed
              << std::setprecision(2) << landmark.color_confidence;
        text->set_text(label.str());
        set_color(text->mutable_color(), 1.0, 1.0, 1.0, 1.0);
    }

    return scene;
}

std::shared_ptr<foxglove::FrameTransform> to_foxglove_map_odom_transform(
    const transforms::Pose2D& pose_map_from_odom,
    const std::int64_t timestamp_ns)
{
    auto transform = std::make_shared<foxglove::FrameTransform>();
    set_timestamp(transform->mutable_timestamp(), timestamp_ns);
    transform->set_parent_frame_id("map");
    transform->set_child_frame_id("odom");

    transform->mutable_translation()->set_x(pose_map_from_odom.x_m);
    transform->mutable_translation()->set_y(pose_map_from_odom.y_m);
    transform->mutable_translation()->set_z(0.0);
    set_yaw_orientation(transform->mutable_rotation(),
                        pose_map_from_odom.yaw_rad);

    return transform;
}

}  // namespace adapters
