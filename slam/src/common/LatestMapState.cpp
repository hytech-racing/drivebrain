#include "common/LatestMapState.hpp"

#include <utility>

namespace slam
{

void LatestMapState::store(MapState state)
{
    std::scoped_lock lock(_mutex);
    _latest = std::move(state);
}

std::optional<MapState> LatestMapState::latest() const
{
    std::scoped_lock lock(_mutex);
    return _latest;
}

}  // namespace slam
