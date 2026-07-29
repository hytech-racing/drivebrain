#include "PointCloudMessageAdapters.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace adapters
{

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

}  // namespace adapters
