#pragma once

#include <cstdint>
#include <vector>

#include "RigidTransform2D.hpp"
#include "common/SlamInterfaces.hpp"

namespace slam
{
enum class PlannerLandmarkState
{
    Pending,
    Optimized,
};

struct PlannerLandmark
{
    std::uint64_t landmark_id{};
    transforms::Point2D position_map_m{};
    ConeColor color{ConeColor::Unknown};
    double color_confidence{};

    PlannerLandmarkState state{PlannerLandmarkState::Pending};
};

struct PlannerMap
{
    std::uint64_t sequence{};
    std::int64_t timestamp_ns{};
    std::vector<PlannerLandmark> landmarks{};
};
}  // namespace slam
