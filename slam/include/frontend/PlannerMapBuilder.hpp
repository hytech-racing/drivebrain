#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "common/PlannerMap.hpp"
#include "common/SlamInterfaces.hpp"
#include "frontend/LandmarkColor.hpp"
#include "frontend/SlamFrontendTypes.hpp"

namespace slam::frontend
{
class PlannerMapBuilder
{
   public:
    [[nodiscard]] PlannerMap build(
        std::uint64_t sequence, std::int64_t timestamp_ns,
        const MapState& map_state,
        const std::vector<PendingPlannerLandmark>& pending_landmarks,
        const std::unordered_map<std::uint64_t, ColorEvidence>&
            color_evidence_by_landmark_id) const;
};
}  // namespace slam::frontend
