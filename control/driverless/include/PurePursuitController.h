#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include "autonomy_msgs.pb.h"
#include <StateTracker.hpp>
#include <Controller.hpp>

#include <memory>
#include <vector>

#include <autonomy_msgs.pb.h>

namespace control {
namespace driverless {


class PurePursuitController : public Controller<core::ControllerOutput, core::VehicleState> {
public:
struct LoggingData {
    std::vector<core::xy_vec<float>> path;
    core::xy_vec<float> vehicle_pos;
    core::xy_vec<float> target_point;
    float curvature;
    float steering_command;
};

bool init();

core::ControllerOutput step_controller(const core::VehicleState& in) override;

/** 
    * Calculates the intersection points of the circle of radius lookahead_distance centered at vehicle_pos with the polyline path
    * @param an ordered set of points representing the path to follow starting with the point closest to the vehicle and to its front (map frame)
    * @param vehicle_pos The current position of the vehicle (map frame)
    * @param lookahead_distance Parameter for pure pursuit
    * @return A vector of 0-N intersection points.
    */
std::vector<core::xy_vec<float>> getGoalPointCandidates(const std::vector<core::xy_vec<float>>& path, core::xy_vec<float> vehicle_pos, float lookahead_distance);

    /**
     * Selects the most appropriate goal point from a set of candidates based on the vehicle's position and heading.
     * @param goal_point_candidates A vector of potential goal points.
     * @param vehicle_pos The current position of the vehicle (map frame).
     * @param vehicle_heading The current heading of the vehicle (map frame).
     * @return The selected goal point.
     */
    core::xy_vec<float> selectGoalPoint(const std::vector<core::xy_vec<float>>& goal_point_candidates, core::xy_vec<float> vehicle_pos, core::xy_vec<float> vehicle_heading);

    /**
    Utility function to get the heading of the vehicle utilizing quaternion data from the vehicle state. The heading is represented as a 2D unit vector
    * @param vehicle_state The current state of the vehicle.
    * @return The heading of the vehicle.
    */


    float getCurvature(core::xy_vec<float> vehicle_pos, core::xy_vec<float> vehicle_heading, core::xy_vec<float> goal_point);

    float getSteeringCommand(float curvature, float wheelbase);

    std::vector<core::xy_vec<float>> loadPathFromCsv(const std::string& filename);

    std::shared_ptr<hytech_msgs::PlannerVisualization> getLoggingData() const;

    void setLoggingData(const LoggingData& data) {
        logging_data_ = data;
    }

    float get_dt_sec() override {
        return 0.1f; // Assuming a control loop of 10 Hz
    }
private:
    LoggingData logging_data_;
    float lookahead_distance_{2.5f};
    float wheelbase_{1.0f};
    float k_p_speed_to_lookahead_{1.0f};
};

}
}

#endif // PURE_PURSUIT_CONTROLLER_H