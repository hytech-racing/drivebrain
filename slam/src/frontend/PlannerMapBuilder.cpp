#include "frontend/PlannerMapBuilder.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace slam::frontend
{
namespace
{
bool finite_position(const transforms::Point2D& p)
{
    return std::isfinite(p.x_m) && std::isfinite(p.y_m);
}

bool valid_color(const LandmarkColorEstimate& color_estimate)
{
    return std::isfinite(color_estimate.color_confidence) &&
           color_estimate.color_confidence >= 0.0 &&
           color_estimate.color_confidence <= 1.0;
}

LandmarkColorEstimate color_estimate_for_id(
    const std::uint64_t landmark_id,
    const std::unordered_map<std::uint64_t, ColorEvidence>& evidence_by_id)
{
    const auto it = evidence_by_id.find(landmark_id);
    if (it == evidence_by_id.end())
    {
        return LandmarkColorEstimate{};
    }

    return estimate_color(it->second);
}
}  // namespace

PlannerMap PlannerMapBuilder::build(
    const std::uint64_t sequence, const std::int64_t timestamp_ns,
    const MapState& map_state,
    const std::vector<PendingPlannerLandmark>& pending_landmarks,
    const std::unordered_map<std::uint64_t, ColorEvidence>&
        color_evidence_by_landmark_id) const
{
    PlannerMap planner_map;

    planner_map.sequence = sequence;
    planner_map.timestamp_ns = timestamp_ns;

    std::unordered_set<std::uint64_t> optimized_ids;

    for (const MapLandmark& landmark : map_state.landmarks)
    {
        if (!finite_position(landmark.position_map_m))
        {
            continue;
        }

        const LandmarkColorEstimate color_estimate = color_estimate_for_id(
            landmark.landmark_id, color_evidence_by_landmark_id);

        if (!valid_color(color_estimate))
        {
            continue;
        }

        optimized_ids.insert(landmark.landmark_id);
        planner_map.landmarks.push_back(PlannerLandmark{
            landmark.landmark_id,
            landmark.position_map_m,
            color_estimate.color,
            color_estimate.color_confidence,
            PlannerLandmarkState::Optimized,
        });
    }

    for (const PendingPlannerLandmark& pending : pending_landmarks)
    {
        if (optimized_ids.count(pending.landmark_id) > 0U ||
            !finite_position(pending.position_odom_m) ||
            !valid_color(pending.color_estimate))
        {
            continue;
        }

        planner_map.landmarks.push_back(PlannerLandmark{
            pending.landmark_id,
            map_state.pose_map_from_odom.transform_point(
                pending.position_odom_m),
            pending.color_estimate.color,
            pending.color_estimate.color_confidence,
            PlannerLandmarkState::Pending,
        });
    }

    std::sort(planner_map.landmarks.begin(), planner_map.landmarks.end(),
              [](const PlannerLandmark& a, const PlannerLandmark& b)
              { return a.landmark_id < b.landmark_id; });

    return planner_map;
}

}  // namespace slam::frontend
