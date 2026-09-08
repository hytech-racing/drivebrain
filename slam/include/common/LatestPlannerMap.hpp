#pragma once

#include <mutex>
#include <optional>

#include "common/PlannerMap.hpp"

namespace slam
{

class LatestPlannerMap
{
   public:
    void store(PlannerMap map);

    [[nodiscard]] std::optional<PlannerMap> latest() const;

   private:
    mutable std::mutex _mutex;
    std::optional<PlannerMap> _latest;
};

}  // namespace slam
