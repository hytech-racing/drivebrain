#pragma once

#include <mutex>
#include <optional>

#include "StateEstimate.hpp"

namespace estimation
{

class LatestEstimate
{
   public:
    void store(StateEstimate estimate);

    std::optional<StateEstimate> latest() const;

   private:
    mutable std::mutex _mutex;
    std::optional<StateEstimate> _latest;
};

}  // namespace estimation
