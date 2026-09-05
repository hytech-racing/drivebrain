#include "PointCloudMessageAdapters.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

namespace adapters
{
namespace
{

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

const char* rejection_reason_to_string(
    const perception::ConeRejectionReason reason)
{
    using perception::ConeRejectionReason;

    switch (reason)
    {
        case ConeRejectionReason::None:
            return "None";
        case ConeRejectionReason::TooFar:
            return "TooFar";
        case ConeRejectionReason::TooFewPoints:
            return "TooFewPoints";
        case ConeRejectionReason::TooShort:
            return "TooShort";
        case ConeRejectionReason::TooTall:
            return "TooTall";
        case ConeRejectionReason::TooWide:
            return "TooWide";
        case ConeRejectionReason::TooElongated:
            return "TooElongated";
    }

    return "Unknown";
}

std::string make_candidate_debug_text(
    const perception::ConeCandidate& candidate)
{
    const auto& features = candidate.feature;

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);

    stream << "ACCEPTED\n"
           << "range: " << features.range_m << " m\n"
           << "points: " << features.num_points << "\n"
           << "height: " << features.height_z_m << " m\n"
           << "width x/y: " << features.width_x_m << " / "
           << features.width_y_m << " m\n"
           << "confidence: " << candidate.confidence;

    return stream.str();
}

std::string make_rejected_debug_text(
    const perception::RejectedCluster& rejected_cluster)
{
    const auto& features = rejected_cluster.features;

    const double small_width = std::min(features.width_x_m, features.width_y_m);
    const double elongation = small_width > 0.0
                                  ? features.max_horizontal_width_m / small_width
                                  : std::numeric_limits<double>::infinity();

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);

    stream << "REJECTED: "
           << rejection_reason_to_string(rejected_cluster.reason) << "\n"
           << "range: " << features.range_m << " m\n"
           << "points: " << features.num_points << "\n"
           << "height: " << features.height_z_m << " m\n"
           << "width x/y: " << features.width_x_m << " / "
           << features.width_y_m << " m\n"
           << "elongation: " << elongation;

    return stream.str();
}

}  // namespace

std::optional<perception::StampedPointCloud> to_core_point_cloud(
    const foxglove::PointCloud& message)
{
    if (message.frame_id() != "lidar")
    {
        spdlog::warn("Rejected PointCloud in unexpected frame '{}'",
                     message.frame_id());
        return std::nullopt;
    }

    int32_t x_offset = -1;
    int32_t y_offset = -1;
    int32_t z_offset = -1;
    int32_t intensity_offset = -1;

    bool x_valid = false;
    bool y_valid = false;
    bool z_valid = false;

    constexpr auto FLOAT32_TYPE =
        foxglove::PackedElementField_NumericType_FLOAT32;

    for (const auto& field : message.fields())
    {
        if (field.name() == "x")
        {
            x_offset = field.offset();
            x_valid = (field.type() == FLOAT32_TYPE);
        }
        else if (field.name() == "y")
        {
            y_offset = field.offset();
            y_valid = (field.type() == FLOAT32_TYPE);
        }
        else if (field.name() == "z")
        {
            z_offset = field.offset();
            z_valid = (field.type() == FLOAT32_TYPE);
        }
        else if (field.name() == "intensity")
        {
            // intensity field doesn't exist in sim yet
            intensity_offset = field.offset();
        }
    }

    if (x_offset == -1 || y_offset == -1 || z_offset == -1 || !x_valid ||
        !y_valid || !z_valid)
    {
        spdlog::warn("x, y, z fields are missing or are not FLOAT32");
        return std::nullopt;
    }

    const std::uint32_t point_stride = message.point_stride();

    if (point_stride == 0)
    {
        spdlog::warn("Point cloud point_stride is zero");
        return std::nullopt;
    }

    const int32_t max_offset =
        std::max({x_offset, y_offset, z_offset, intensity_offset});
    if (point_stride < static_cast<std::uint32_t>(max_offset) + sizeof(float))
    {
        // hard coded float check for now
        spdlog::warn("point_stride ({}) is smaller than required field offsets",
                     point_stride);
        return std::nullopt;
    }

    const std::size_t num_points = message.data().size() / point_stride;

    perception::StampedPointCloud stamped_point_cloud;
    stamped_point_cloud.frame = transforms::FrameId::Lidar;
    perception::PointCloud& points = stamped_point_cloud.points;

    points.reserve(num_points);

    const uint8_t* raw_data_ptr =
        reinterpret_cast<const uint8_t*>(message.data().data());

    for (size_t i = 0; i < num_points; ++i)
    {
        const uint8_t* base = raw_data_ptr + (i * point_stride);

        perception::PointXYZI point;

        std::memcpy(&point.x, base + x_offset, sizeof(float));
        std::memcpy(&point.y, base + y_offset, sizeof(float));
        std::memcpy(&point.z, base + z_offset, sizeof(float));

        if (intensity_offset != -1)
        {
            std::memcpy(&point.intensity, base + intensity_offset,
                        sizeof(float));
        }
        else
        {
            point.intensity = 0.0F;
        }

        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z) || !std::isfinite(point.intensity))
        {
            spdlog::warn("Skipping malformed PointXYZI");
            continue;
        }

        points.push_back(point);
    }

    stamped_point_cloud.timestamp_ns =
        (message.timestamp().seconds() * 1'000'000'000LL) +
        message.timestamp().nanos();

    return stamped_point_cloud;
}

std::shared_ptr<foxglove::PointCloud> to_foxglove_point_cloud(
    const perception::StampedPointCloud& cloud, std::string_view frame_id)
{
    auto message = std::make_shared<foxglove::PointCloud>();

    message->set_frame_id(std::string(frame_id));

    if (!cloud.points.empty())
    {
        auto* timestamp = message->mutable_timestamp();
        timestamp->set_seconds(
            static_cast<int64_t>(cloud.timestamp_ns / 1'000'000'000));
        timestamp->set_nanos(
            static_cast<int32_t>(cloud.timestamp_ns % 1'000'000'000));
    }

    constexpr std::uint32_t kFloat32Size = sizeof(float);
    constexpr std::uint32_t kPointStride = kFloat32Size * 4;
    constexpr auto kFloat32Type =
        foxglove::PackedElementField_NumericType_FLOAT32;

    message->set_point_stride(kPointStride);

    auto* x_field = message->add_fields();
    x_field->set_name("x");
    x_field->set_offset(0);
    x_field->set_type(kFloat32Type);

    auto* y_field = message->add_fields();
    y_field->set_name("y");
    y_field->set_offset(kFloat32Size);
    y_field->set_type(kFloat32Type);

    auto* z_field = message->add_fields();
    z_field->set_name("z");
    z_field->set_offset(kFloat32Size * 2);
    z_field->set_type(kFloat32Type);

    auto* intensity_field = message->add_fields();
    intensity_field->set_name("intensity");
    intensity_field->set_offset(kFloat32Size * 3);
    intensity_field->set_type(kFloat32Type);

    std::string byte_buffer;
    byte_buffer.resize(cloud.points.size() * kPointStride);

    std::uint8_t* raw_data_ptr =
        reinterpret_cast<std::uint8_t*>(byte_buffer.data());
    std::size_t valid_point_count = 0;

    for (const auto& point : cloud.points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z) || !std::isfinite(point.intensity))
        {
            continue;
        }

        std::uint8_t* base = raw_data_ptr + (valid_point_count * kPointStride);
        std::memcpy(base + 0, &point.x, kFloat32Size);
        std::memcpy(base + kFloat32Size, &point.y, kFloat32Size);
        std::memcpy(base + (kFloat32Size * 2), &point.z, kFloat32Size);
        std::memcpy(base + (kFloat32Size * 3), &point.intensity,
                    kFloat32Size);
        valid_point_count++;
    }

    if (valid_point_count < cloud.points.size())
    {
        byte_buffer.resize(valid_point_count * kPointStride);
    }

    message->set_data(std::move(byte_buffer));
    return message;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_cluster_markers(
    const std::vector<perception::ClusterFeatures>& cluster_features,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id)
{
    auto scene = make_clearing_scene(timestamp_ns);

    if (cluster_features.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), frame_id, entity_id, timestamp_ns);

    for (const auto& curr_cluster_features : cluster_features)
    {
        auto* sphere = entity->add_spheres();
        auto* position = sphere->mutable_pose()->mutable_position();
        position->set_x(curr_cluster_features.centroid.x);
        position->set_y(curr_cluster_features.centroid.y);
        position->set_z(curr_cluster_features.centroid.z);
        sphere->mutable_pose()->mutable_orientation()->set_w(1.0);

        sphere->mutable_size()->set_x(0.2);
        sphere->mutable_size()->set_y(0.2);
        sphere->mutable_size()->set_z(0.2);

        auto* color = sphere->mutable_color();
        color->set_r(0.0);
        color->set_g(1.0);
        color->set_b(0.0);
        color->set_a(0.5);
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_cone_candidate_markers(
    const std::vector<perception::ConeCandidate>& cone_candidates,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id)
{
    auto scene = make_clearing_scene(timestamp_ns);

    if (cone_candidates.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), frame_id, entity_id, timestamp_ns);

    for (const auto& cone_candidate : cone_candidates)
    {
        auto* sphere = entity->add_spheres();
        auto* position = sphere->mutable_pose()->mutable_position();
        position->set_x(cone_candidate.position.x);
        position->set_y(cone_candidate.position.y);
        position->set_z(cone_candidate.position.z);
        sphere->mutable_pose()->mutable_orientation()->set_w(1.0);

        sphere->mutable_size()->set_x(0.2);
        sphere->mutable_size()->set_y(0.2);
        sphere->mutable_size()->set_z(0.2);

        auto* color = sphere->mutable_color();
        color->set_r(0.0);
        color->set_g(1.0);
        color->set_b(0.0);
        color->set_a(0.25 + 0.75 * cone_candidate.confidence);
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_cone_candidate_text(
    const std::vector<perception::ConeCandidate>& cone_candidates,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id)
{
    auto scene = make_clearing_scene(timestamp_ns);

    if (cone_candidates.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), frame_id, entity_id, timestamp_ns);

    for (const auto& cone_candidate : cone_candidates)
    {
        auto* text = entity->add_texts();
        auto* text_position = text->mutable_pose()->mutable_position();
        text_position->set_x(cone_candidate.position.x);
        text_position->set_y(cone_candidate.position.y);
        text_position->set_z(cone_candidate.feature.bbox.max.z + 0.5);
        text->mutable_pose()->mutable_orientation()->set_w(1.0);
        text->set_billboard(true);
        text->set_font_size(0.10);
        text->set_text(make_candidate_debug_text(cone_candidate));

        auto* text_color = text->mutable_color();
        text_color->set_r(1.0);
        text_color->set_g(1.0);
        text_color->set_b(1.0);
        text_color->set_a(1.0);
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_rejected_cluster_markers(
    const std::vector<perception::RejectedCluster>& rejected_clusters,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id)
{
    auto scene = make_clearing_scene(timestamp_ns);

    if (rejected_clusters.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), frame_id, entity_id, timestamp_ns);

    for (const auto& rejected_cluster : rejected_clusters)
    {
        auto* cube = entity->add_cubes();
        auto* position = cube->mutable_pose()->mutable_position();
        position->set_x(rejected_cluster.features.centroid.x);
        position->set_y(rejected_cluster.features.centroid.y);
        position->set_z(rejected_cluster.features.centroid.z);
        cube->mutable_pose()->mutable_orientation()->set_w(1.0);

        cube->mutable_size()->set_x(0.2);
        cube->mutable_size()->set_y(0.2);
        cube->mutable_size()->set_z(0.2);

        auto* color = cube->mutable_color();
        color->set_r(1.0);
        color->set_g(0.0);
        color->set_b(0.0);
        color->set_a(0.5);
    }

    return scene;
}

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_rejected_cluster_text(
    const std::vector<perception::RejectedCluster>& rejected_clusters,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id)
{
    auto scene = make_clearing_scene(timestamp_ns);

    if (rejected_clusters.empty())
    {
        return scene;
    }

    auto* entity = add_entity(scene.get(), frame_id, entity_id, timestamp_ns);

    for (const auto& rejected_cluster : rejected_clusters)
    {
        auto* text = entity->add_texts();
        auto* text_position = text->mutable_pose()->mutable_position();
        text_position->set_x(rejected_cluster.features.centroid.x);
        text_position->set_y(rejected_cluster.features.centroid.y);
        text_position->set_z(rejected_cluster.features.bbox.max.z + 0.5);
        text->mutable_pose()->mutable_orientation()->set_w(1.0);
        text->set_billboard(true);
        text->set_font_size(0.10);
        text->set_text(make_rejected_debug_text(rejected_cluster));

        auto* text_color = text->mutable_color();
        text_color->set_r(1.0);
        text_color->set_g(1.0);
        text_color->set_b(1.0);
        text_color->set_a(1.0);
    }

    return scene;
}

}  // namespace adapters
