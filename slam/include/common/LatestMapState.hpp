#pragma once

#include <mutex>
#include <optional>

#include "common/SlamInterfaces.hpp"

namespace slam
{
class LatestMapState
{
   public:
    void store(MapState state);

    [[nodiscard]]
    std::optional<MapState> latest() const;

   private:
    mutable std::mutex _mutex;
    std::optional<MapState> _latest;
};
}  // namespace slam
