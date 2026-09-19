#include <StateTracker.hpp>

#include <vector>

#include <Mathematics/NaturalCubicSpline.h>

namespace planning {
class VelocityPlanner {
public:
    void tick();
    size_t getHorizonPointIndex(std::vector<core::xy_vec<float>> path);
private:
    std::vector<core::xy_vec<float>> loadPathFromCsv(const std::string& filename);

};
} // namespace planning