#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include "autonomy_msgs.pb.h"
#include <StateTracker.hpp>
#include <Controller.hpp>

#include <memory>
#include <vector>


namespace control {
namespace driverless {


class PurePursuitController {
    public:
        struct LoggingData {
            std::vector<core::xy_vec<float>> path;
            core::xy_vec<float> vehicle_pos;
            core::xy_vec<float> target_point;
            float curvature;
            float steering_command;
        };

        bool init();

        std::optional<float> step_controller(const core::VehicleState& in, std::vector<core::xy_vec<float>> path, float& curvature_in);

        /** 
            * Calculates the intersection points of the circle of radius lookahead_distance centered at vehicle_pos with the polyline path
            * @param path an ordered set of points representing the path to follow starting with the point closest to the vehicle and to its front (map frame)
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
        * Returns the path, vehicle position, target point, curvature, and steering command.
        * @return A shared pointer to a PlannerVisualization message containing the logging data from a local struct
        */
        std::shared_ptr<hytech_msgs::PlannerVisualization> getLoggingData() const;

        /**
        * Sets the logging data that will be sent to xcel later.
        * @param data The logging data to set.
        */
        void setLoggingData(const LoggingData& data) {
            logging_data_ = data;
        }

        float get_dt_sec() {
            return 0.01f; // currently stepping at 100Hz, but this should be configurable
        }

        /**
        * Returns the last calculated curvature from the pure pursuit algorithm.
        * @return The curvature.
        */
        float getCurvature() const {
            return curvature_;
        }

    private:
        LoggingData logging_data_;
        float lookahead_distance_{2.5f};
        float wheelbase_{1.53f};
        float curvature_;
        /** 
        Get an arc of constant curvature from current point to target whose tangent is instantaneous heading of vehicle.
        * @param vehicle_pos The current position of the vehicle (map frame)
        * @param vehicle_heading The current heading of the vehicle (map frame)
        * @param goal_point The target point to reach (map frame)
        * @return The curvature of the arc connecting the vehicle's position to the goal point.
        */
        float getCurvature(core::xy_vec<float> vehicle_pos, core::xy_vec<float> vehicle_heading, core::xy_vec<float> goal_point);

        /**
        Converts the curvature to a steering command based on bicycle model.
        * @param curvature The curvature of the path.
        * @param wheelbase The wheelbase of the vehicle.
        * @return The steering command.
        */
        float getSteeringCommand(float curvature, float wheelbase);
        
};

} // driverless namespace
} // control namespace

#endif // PURE_PURSUIT_CONTROLLER_H