#include "common/LatestPlannerMap.hpp"

#include <utility>

namespace slam
{

void LatestPlannerMap::store(PlannerMap map)
{
    std::scoped_lock lock(_mutex);
    _latest = std::move(map);
}

std::optional<PlannerMap> LatestPlannerMap::latest() const
{
    std::scoped_lock lock(_mutex);
    return _latest;
}

}  // namespace slam
