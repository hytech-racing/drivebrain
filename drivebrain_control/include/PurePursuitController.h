#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include <StateTracker.hpp>
#include <Controller.hpp>
#include <FoxgloveServer.hpp>
#include "dv_msgs.pb.h"

#include <mutex>
#include <optional>
#include <vector>

#include <boost/signals2.hpp>


namespace control {
namespace driverless {


struct PurePursuitInput {
    const core::VehicleState& state;
    const std::vector<core::xy_vec<float>>& path; // map frame
};

struct PurePursuitOutput {
    std::optional<float> column_angle_deg; // nullopt if no valid steering command
    float curvature;                       // signed, 1/m, for the velocity planner
};

class PurePursuitController : public Controller<PurePursuitOutput, PurePursuitInput> {
    public:
        // lookahead_distance: fixed lookahead used by step_controller (m)
        // wheelbase: bicycle model wheelbase (m)
        // lookahead_base, lookahead_gain_s: dynamic lookahead = lookahead_base + lookahead_gain_s * v (m, s)
        struct config {
            float lookahead_distance;
            float wheelbase;
            float lookahead_base;
            float lookahead_gain_s;
        };

        bool init();

        PurePursuitOutput step_controller(const PurePursuitInput& in) override;

        /** 
            * Calculates the intersection points of the circle of radius lookahead_distance centered at vehicle_pos with the polyline path
            * @param path an ordered set of points representing the path to follow starting with the point closest to the vehicle and to its front (map frame)
            * @param vehicle_pos The current position of the vehicle (map frame)
            * @param lookahead_distance Parameter for pure pursuit
            * @return A vector of 0-N intersection points.
        */
        std::vector<core::xy_vec<float>> get_goal_point_candidates(const std::vector<core::xy_vec<float>>& path, core::xy_vec<float> vehicle_pos, float lookahead_distance);

        /**
        * Selects the most appropriate goal point from a set of candidates based on the vehicle's position and heading.
        * @param goal_point_candidates A vector of potential goal points.
        * @param vehicle_pos The current position of the vehicle (map frame).
        * @param vehicle_heading The current heading of the vehicle (map frame).
        * @return The selected goal point.
        */
        std::optional<size_t> select_goal_point(const std::vector<core::xy_vec<float>>& path, const std::vector<core::xy_vec<float>>& goal_point_candidates, core::xy_vec<float> vehicle_pos, core::xy_vec<float> vehicle_heading);

        /* Diagnostics of the most recent step_controller() */
        const dv_msgs::PurePursuitDebug& debug() const { return _debug; }

        float get_dt_sec() override {
            return 0.004f; // currently stepping at 250Hz, but this should be configurable
        }

    private:
        void _handle_param_updates(const std::unordered_map<std::string, core::DBParam> &new_param_map);

        std::mutex _config_mutex;
        config _config{};
        boost::signals2::scoped_connection _param_connection;

        dv_msgs::PurePursuitDebug _debug;

        /**
        Get an arc of constant curvature from current point to target whose tangent is instantaneous heading of vehicle.
        * @param vehicle_pos The current position of the vehicle (map frame)
        * @param vehicle_heading The current heading of the vehicle (map frame)
        * @param goal_point The target point to reach (map frame)
        * @param lookahead_distance Parameter for pure pursuit
        * @return The curvature of the arc connecting the vehicle's position to the goal point.
        */
        float get_curvature(core::xy_vec<float> vehicle_pos, core::xy_vec<float> vehicle_heading, core::xy_vec<float> goal_point, float lookahead_distance);


        // TODO: Dynamic lookahead
        /**
        Calculates dynamic lookahead distance using vehicle velocity - needs to be updated to use sensor data 
        (ideally sensoric) for true vehicle velocity, currently using sim position and delta t
         */
        // float dynamic_lookahead(float prev_vehicle_x, float prev_vehicle_y, float dt);
        // // need dynamic lookahead distance
        // float _wheel_radius = 2.302f;


        // estimator for yaw-to-column angle conversion
        const estimation::SteeringEstimator _steering_estimator{};
        
};

} // driverless namespace
} // control namespace

#endif // PURE_PURSUIT_CONTROLLER_H
