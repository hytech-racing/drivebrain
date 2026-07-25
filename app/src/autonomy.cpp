#include "autonomy.hpp"

#include <spdlog/spdlog.h>

#include <FoxgloveServer.hpp>
#include <PathPlanner.hpp>
#include <ReferencePath.hpp>
#include <StateTracker.hpp>
#include <Telemetry.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <variant>
#include <vector>

#include "PurePursuitController.hpp"

namespace core
{

Autonomy::~Autonomy() { stop(); }

void Autonomy::start()
{
    if (_running.exchange(true))
    {
        return;
    }

#if HOOTL_ENABLED
    auto path_param = FoxgloveServer::instance().get_param<std::string>(
        "DriverlessSIL/reference_path_csv");
    auto lookahead_param = FoxgloveServer::instance().get_param<float>(
        "DriverlessSIL/lookahead_m");
    auto target_speed_param = FoxgloveServer::instance().get_param<float>(
        "DriverlessSIL/target_speed_mps");

    if (!lookahead_param)
    {
        spdlog::error("Missing DriverlessSIL/lookahead_m, defaulting to 3.0 m");

        _lookahead_m = 3.0f;
    }
    else
    {
        _lookahead_m = *lookahead_param;
    }

    if (!target_speed_param)
    {
        spdlog::error(
            "Missing DriverlessSIL/target_speed_mps, defaulting to 3.0 m/s");
        _target_speed_mps = 3.0f;
    }
    else
    {
        _target_speed_mps = *target_speed_param;
    }

    if (!path_param)
    {
        spdlog::error("Missing DriverlessSIL/reference_path_csv");
    }
    else
    {
        std::optional<planning::ReferencePath> loaded_path =
            planning::load_reference_path_csv(*path_param);

        if (!loaded_path)
        {
            spdlog::error("Failed to load SIL reference path from {}",
                          *path_param);
        }
        else
        {
            _sil_reference_path = std::move(*loaded_path);

            _sil_reference_path_loaded = true;

            spdlog::info("Loaded SIL reference path: {} points, {:.2f} m",
                         _sil_reference_path.points.size(),
                         _sil_reference_path.length_m);

            std::vector<xyz_vec<float>> render_points;

            render_points.reserve(_sil_reference_path.points.size() + 1);
            for (const planning::ReferencePathPoint& point :
                 _sil_reference_path.points)
            {
                render_points.emplace_back(
                    xyz_vec<float>{static_cast<float>(point.x_m),
                                   static_cast<float>(point.y_m), 0.0f});
            }

            if (!render_points.empty())
            {
                render_points.push_back(render_points.front());
            }

            _sil_reference_path_render_points = std::move(render_points);
        }
    }
#endif

    _slam.start();
    _thread = std::thread(&Autonomy::_run, this);
    spdlog::info("Autonomy stack started");
}

void Autonomy::stop()
{
    if (!_running.exchange(false))
    {
        return;
    }
    if (_thread.joinable())
    {
        _thread.join();
    }
    _slam.stop();
    spdlog::info("Autonomy stack stopped");
}

// bool Autonomy::is_valid()
// {
//     auto dv = StateTracker::instance().dv_state();
//     return dv.lidar_is_valid && dv.path && !dv.path->empty();
// }

bool Autonomy::is_valid()
{
#if HOOTL_ENABLED
    return StateTracker::instance().simulation_ground_truth().second &&
           _sil_reference_path_loaded;
#else
    auto dv = StateTracker::instance().dv_state();
    return dv.lidar_is_valid && dv.path && !dv.path->empty();
#endif
}

// TODO: Pure pursuit impl should be invoked here. Hardcoded for now to verify
// the command path from drivebrain through to the sim's vehicle dynamics.
ControllerOutput Autonomy::command(const VehicleState& vehicle_state)
{
    const auto [truth, truth_valid] =
        StateTracker::instance().simulation_ground_truth();

    const ControllerOutput safe_output{

        TorqueControlOut{veh_vec<float>{0.0, 0.0, 0.0, 0.0}}, 0.0};

    if (!truth_valid || !_sil_reference_path_loaded)
    {
        return safe_output;
    }

    const float vx_b_target = _target_speed_mps;

    const float cy = std::cos(truth.yaw_world_rad);
    const float sy = std::sin(truth.yaw_world_rad);

    const float vx_b = truth.vx_world_mps * cy + truth.vy_world_mps * sy;

    const float error = vx_b_target - vx_b;

    const float Kp = 3.0f;

    const float torque = std::clamp(Kp * error, -2.0f, 2.0f);

    TorqueControlOut torque_out;
    torque_out.desired_torques_nm = {torque, torque, torque, torque};

    const std::optional<planning::PathProjection> projection =
        planning::project_onto_path(
            planning::Point2D{truth.x_world_m, truth.y_world_m},
            _sil_reference_path);

    if (!projection)
    {
        return safe_output;
    }

    const std::optional<planning::ReferencePathPoint> target =
        planning::interpolate_at_s(_sil_reference_path,
                                   projection->point.s_m + _lookahead_m);

    if (!target)
    {
        return safe_output;
    }

    const std::optional<control::PurePursuitResult> pure_pursuit =
        control::compute_pure_pursuit(
            control::PurePursuitInput{
                control::Point2D{truth.x_world_m, truth.y_world_m},
                truth.yaw_world_rad,
                control::Point2D{target->x_m, target->y_m}},
            control::PurePursuitParams{1.53, 20.0});

    if (!pure_pursuit)
    {
        return safe_output;
    }

    ControllerOutput out;
    // out.out = std::monostate{};
    out.out = torque_out;
    out.desired_steering_deg = pure_pursuit->steering_rad * 180.0 / M_PI;

    return out;
}

void Autonomy::_run()
{
    auto next_tick = std::chrono::steady_clock::now();
    std::shared_ptr<const foxglove::PointCloud> last_scan;

    while (_running)
    {
        next_tick += _period;

        auto dv = StateTracker::instance().dv_state();

        if (dv.lidar_is_valid && dv.lidar_cloud != last_scan)
        {
            last_scan = dv.lidar_cloud;

            // TODO: cone classifier needs to be invoked here
            auto path = planning::plan_path(
                *StateTracker::instance().dv_state().cone_observations);
            // render_path(path, "planned_path", "lidar");
            StateTracker::instance().set_dv_path(
                std::make_shared<const std::vector<xyz_vec<float>>>(
                    std::move(path)));
        }

        const auto now = std::chrono::steady_clock::now();

        if (_sil_reference_path_loaded &&
            now - _last_sil_path_publish >= std::chrono::seconds(1))
        {
            render_path(_sil_reference_path_render_points, "sil_reference_path",
                        "map");

            _last_sil_path_publish = now;
        }

        const auto [truth, truth_valid] =
            StateTracker::instance().simulation_ground_truth();

        if (truth_valid && _sil_reference_path_loaded)
        {
            const planning::Point2D vehicle_position_world{truth.x_world_m,
                                                           truth.y_world_m};

            const auto projection = planning::project_onto_path(
                vehicle_position_world, _sil_reference_path);

            if (projection)
            {
                std::vector<xyz_vec<float>> cross_track_line{
                    {
                        static_cast<float>(truth.x_world_m),
                        static_cast<float>(truth.y_world_m),
                        0.2f,
                    },
                    {
                        static_cast<float>(projection->point.x_m),
                        static_cast<float>(projection->point.y_m),
                        0.2f,
                    },
                };

                render_path(cross_track_line, "sil_path_projection", "map");

                const double target_s_m = projection->point.s_m + _lookahead_m;

                const std::optional<planning::ReferencePathPoint> target =
                    planning::interpolate_at_s(_sil_reference_path, target_s_m);

                if (target)
                {
                    std::vector<xyz_vec<float>> target_line{
                        {
                            static_cast<float>(truth.x_world_m),
                            static_cast<float>(truth.y_world_m),
                            0.2f,
                        },
                        {
                            static_cast<float>(target->x_m),
                            static_cast<float>(target->y_m),
                            0.2f,
                        },
                    };

                    render_path(target_line, "sil_target_point", "map");
                }
            }
        }

        std::this_thread::sleep_until(next_tick);
    }
}

}  // namespace core
