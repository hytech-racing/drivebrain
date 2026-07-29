#pragma once

#include <foxglove/PointCloud.pb.h>

#include <memory>
#include <optional>
#include <string_view>

#include "PointCloudTypes.hpp"

namespace adapters
{

std::optional<perception::StampedPointCloud> to_core_point_cloud(
    const foxglove::PointCloud& message);

std::shared_ptr<foxglove::PointCloud> to_foxglove_point_cloud(
    const perception::StampedPointCloud& cloud, std::string_view frame_id);

}
