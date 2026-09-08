#include "LatestEstimate.hpp"

namespace estimation
{

void LatestEstimate::store(StateEstimate estimate)
{
    std::scoped_lock lock(_mutex);
    _latest = std::move(estimate);
}

std::optional<StateEstimate> LatestEstimate::latest() const
{
    std::scoped_lock lock(_mutex);
    return _latest;
}

}  // namespace estimation
