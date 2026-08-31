#ifndef PURE_PURSUIT_MATH_H
#define PURE_PURSUIT_MATH_H

#include <StateTracker.hpp>
#include <vector>

namespace control {
namespace driverless {


class PurePursuitMath {
    /** 
     * Calculates the intersection points of the circle of radius lookahead_distance centered at vehicle_pos with the polyline path
     * @param an ordered set of points representing the path to follow starting with the point closest to the vehicle and to its front (map frame)
     * @param vehicle_pos The current position of the vehicle (map frame)
     * @param lookahead_distance Parameter for pure pursuit
     * @return A vector of 0-N intersection points.
     */
    std::vector<core::xy_vec<float>> getGoalPointCandidates(std::vector<core::xy_vec<float>> path, core::xy_vec<float> vehicle_pos, float lookahead_distance);

    /**
     * Selects the most appropriate goal point from a set of candidates based on the vehicle's position and heading.
     * @param goal_point_candidates A vector of potential goal points.
     * @param vehicle_pos The current position of the vehicle (map frame).
     * @param vehicle_heading The current heading of the vehicle (map frame).
     * @return The selected goal point.
     */
    core::xy_vec<float> selectGoalPoint(std::vector<core::xy_vec<float>> goal_point_candidates, core::xy_vec<float> vehicle_pos, core::xy_vec<float> vehicle_heading);

    /**
    Utility function to get the heading of the vehicle utilizing quaternion data from the vehicle state. The heading is represented as a 2D unit vector
    * @param vehicle_state The current state of the vehicle.
    * @return The heading of the vehicle.
    */
    core::xy_vec<float> getVehicleHeading(core::VehicleState vehicle_state);
};

}
}

#endif // PURE_PURSUIT_MATH_H