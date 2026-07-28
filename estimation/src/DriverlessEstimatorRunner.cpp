#include "DriverlessEstimatorRunner.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <utility>
#include <variant>
#include <vector>

#include "Telemetry.hpp"
#include "dv_msgs.pb.h"

namespace estimation
{
namespace
{

double wrap_to_pi(double theta_rad)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;

    theta_rad = std::fmod(theta_rad + kPi, kTwoPi);
    if (theta_rad < 0.0)
    {
        theta_rad += kTwoPi;
    }
    return theta_rad - kPi;
}

}  // namespace

DriverlessEstimatorRunner::DriverlessEstimatorRunner(
    std::shared_ptr<LatestEstimate> latest_estimate,
    std::shared_ptr<transforms::TransformBuffer> transform_buffer,
    const EkfParams& ekf_params, const GssSensorConfig& gss_sensor_config,
    bool publish_full_telemetry)
    : _latest_estimate(latest_estimate),
      _transform_buffer(transform_buffer),
      _publish_full_telemetry(publish_full_telemetry),
      _ekf(ekf_params, gss_sensor_config, _transform_buffer->base_to_gss())
{
    InternalEstimatorState initial_state;
    initial_state.x.setZero();

    StateCovariance initial_covariance;
    initial_covariance.setZero();
    initial_covariance(StateIndex::X_ODOM, StateIndex::X_ODOM) =
        ekf_params.initial_position_std_m * ekf_params.initial_position_std_m;

    initial_covariance(StateIndex::Y_ODOM, StateIndex::Y_ODOM) =
        ekf_params.initial_position_std_m * ekf_params.initial_position_std_m;

    initial_covariance(StateIndex::YAW_ODOM, StateIndex::YAW_ODOM) =
        ekf_params.initial_yaw_std_rad * ekf_params.initial_yaw_std_rad;

    initial_covariance(StateIndex::VX_BODY, StateIndex::VX_BODY) =
        ekf_params.initial_velocity_std_mps *
        ekf_params.initial_velocity_std_mps;
    initial_covariance(StateIndex::VY_BODY, StateIndex::VY_BODY) =
        ekf_params.initial_velocity_std_mps *
        ekf_params.initial_velocity_std_mps;

    _ekf.initialize(initial_state, initial_covariance);
}

DriverlessEstimatorRunner::~DriverlessEstimatorRunner() { stop(); }

void DriverlessEstimatorRunner::start()
{
    {
        std::scoped_lock lock(_queue_mutex);
        if (_running)
        {
            return;
        }
        _running = true;
    }

    _thread = std::thread(&DriverlessEstimatorRunner::_run, this);
}

void DriverlessEstimatorRunner::stop()
{
    if (!_running)
    {
        return;
    }

    {
        std::scoped_lock lock(_queue_mutex);
        _running = false;
    }

    _queue_cv.notify_one();

    if (_thread.joinable())
    {
        _thread.join();
    }
}

bool DriverlessEstimatorRunner::_enqueue_event(EstimatorEvent event)
{
    {
        std::scoped_lock lock(_queue_mutex, _stats_mutex);

        if (ImuMeasurement* imu = std::get_if<ImuMeasurement>(&event))
        {
            if (imu->timestamp_ns < _stats.latest_imu_timestamp_ns)
            {
                _stats.out_of_order_measurements++;
                return false;
            }
        }
        else if (GssMeasurement* gss = std::get_if<GssMeasurement>(&event))
        {
            if (gss->timestamp_ns < _stats.latest_gss_timestamp_ns)
            {
                _stats.out_of_order_measurements++;
                return false;
            }
        }

        if (_queue.size() >= kMaximumQueueSize)
        {
            _stats.queue_drops++;
            return false;
        }

        if (ImuMeasurement* imu = std::get_if<ImuMeasurement>(&event))
        {
            _stats.imu_enqueued++;
            _stats.latest_imu_timestamp_ns = imu->timestamp_ns;
        }
        else if (GssMeasurement* gss = std::get_if<GssMeasurement>(&event))
        {
            _stats.gss_enqueued++;
            _stats.latest_gss_timestamp_ns = gss->timestamp_ns;
        }

        _queue.push_back(std::move(event));
        _stats.current_queue_depth = _queue.size();
        _stats.maximum_queue_depth =
            std::max(_stats.maximum_queue_depth, _stats.current_queue_depth);
    }

    _queue_cv.notify_one();
    return true;
}

bool DriverlessEstimatorRunner::enqueue(ImuMeasurement measurement)
{
    return _enqueue_event(std::move(measurement));
}

bool DriverlessEstimatorRunner::enqueue(GssMeasurement measurement)
{
    return _enqueue_event(std::move(measurement));
}

EstimatorQueueStats DriverlessEstimatorRunner::stats() const
{
    std::scoped_lock lock(_queue_mutex, _stats_mutex);
    EstimatorQueueStats stats = _stats;
    stats.current_queue_depth = _queue.size();
    return stats;
}

void DriverlessEstimatorRunner::_run()
{
    while (true)
    {
        EstimatorEvent event;
        bool is_imu = false;
        bool is_gss = false;

        {
            std::unique_lock lock(_queue_mutex);

            _queue_cv.wait(lock,
                           [this]() { return !_running || !_queue.empty(); });

            if (!_running && _queue.empty())
            {
                break;
            }

            event = std::move(_queue.front());
            _queue.pop_front();

            is_imu = std::holds_alternative<ImuMeasurement>(event);
            is_gss = std::holds_alternative<GssMeasurement>(event);

            {
                std::scoped_lock stats_lock(_stats_mutex);
                _stats.current_queue_depth = _queue.size();
            }
        }

        const bool processed = _process_event(event);

        {
            std::scoped_lock stats_lock(_stats_mutex);

            if (processed && is_imu)
            {
                _stats.imu_processed++;
            }

            if (processed && is_gss)
            {
                _stats.gss_processed++;
            }
        }
    }
}

bool DriverlessEstimatorRunner::_process_event(const EstimatorEvent& event)
{
    std::uint64_t event_timestamp_ns = 0;
    if (const auto* imu = std::get_if<ImuMeasurement>(&event))
    {
        event_timestamp_ns = imu->timestamp_ns;
    }
    else if (const auto* gss = std::get_if<GssMeasurement>(&event))
    {
        event_timestamp_ns = gss->timestamp_ns;
    }

    const std::uint64_t filter_timestamp_ns = _ekf.filter_timestamp_ns();
    if (filter_timestamp_ns != 0 && event_timestamp_ns < filter_timestamp_ns)
    {
        std::scoped_lock lock(_stats_mutex);
        _stats.out_of_order_measurements++;
        return false;
    }

    if (std::holds_alternative<ImuMeasurement>(event))
    {
        const auto imu = std::get<ImuMeasurement>(event);

        if (_previous_imu_stamp_ns)
        {
            const double dt_s = static_cast<double>(imu.timestamp_ns -
                                                    *_previous_imu_stamp_ns) /
                                1e9;

            _ekf.predict(imu, dt_s);
        }
        else
        {
            _previous_imu_stamp_ns = imu.timestamp_ns;
            return true;
        }

        _previous_imu_stamp_ns = imu.timestamp_ns;
    }

    if (std::holds_alternative<GssMeasurement>(event))
    {
        const auto gss = std::get<GssMeasurement>(event);

        GssMeasurementEigen gss_eigen;
        gss_eigen.vx_sensor_mps() = gss.vx_sensor_flu_mps;
        gss_eigen.vy_sensor_mps() = gss.vy_sensor_flu_mps;
        gss_eigen.timestamp_ns = gss.timestamp_ns;

        _ekf.update_gss_speed(gss_eigen);
    }

    const StateEstimate state_estimate = _ekf.state_estimate();

    _publish_estimate(state_estimate, _publish_full_telemetry);

    return true;
}

void DriverlessEstimatorRunner::_publish_estimate(const StateEstimate& estimate,
                                                  bool publish_full_telemetry)
{
    if (_latest_estimate)
    {
        _latest_estimate->store(estimate);
    }

    if (!publish_full_telemetry)
    {
        return;
    }

    const transforms::RigidTransform2D T_odom_base{
        estimate.x_odom_m, estimate.y_odom_m, estimate.yaw_odom_rad};

    _transform_buffer->insert_T_odom_base(estimate.timestamp_ns, T_odom_base);

    core::publish_transform("map", "odom", estimate.timestamp_ns,
                            transforms::RigidTransform2D{0.0, 0.0, 0.0});
    core::publish_transform("odom", "base_link", estimate.timestamp_ns,
                            T_odom_base);
    core::publish_transform("base_link", "imu", estimate.timestamp_ns,
                            _transform_buffer->base_to_imu());
    core::publish_transform("base_link", "gss", estimate.timestamp_ns,
                            _transform_buffer->base_to_gss());
    core::publish_transform("base_link", "lidar", estimate.timestamp_ns,
                            _transform_buffer->base_to_lidar(), 0.15);

    auto msg = std::make_shared<dv_msgs::DriverlessStateEstimate>();

    msg->set_initialized(estimate.initialized);
    msg->set_timestamp_ns(estimate.timestamp_ns);

    msg->set_x_odom_m(estimate.x_odom_m);
    msg->set_y_odom_m(estimate.y_odom_m);
    msg->set_yaw_odom_rad(estimate.yaw_odom_rad);

    msg->set_vx_vehicle_flu_mps(estimate.vx_vehicle_flu_mps);
    msg->set_vy_vehicle_flu_mps(estimate.vy_vehicle_flu_mps);
    msg->set_yaw_rate_vehicle_flu_radps(estimate.yaw_rate_vehicle_flu_radps);

    core::log("/estimation/driverless_state", msg);

    _publish_odom_trail(estimate);
}

void DriverlessEstimatorRunner::_publish_odom_trail(
    const StateEstimate& estimate)
{
    bool should_add_point = _odom_trail.empty();
    if (!_odom_trail.empty())
    {
        const auto& last_point = _odom_trail.back();
        const double dx_m = estimate.x_odom_m - last_point[0];
        const double dy_m = estimate.y_odom_m - last_point[1];
        const double distance_m = std::hypot(dx_m, dy_m);
        const double yaw_delta_rad =
            _last_odom_trail_point_yaw_rad
                ? std::abs(wrap_to_pi(estimate.yaw_odom_rad -
                                      *_last_odom_trail_point_yaw_rad))
                : 0.0;

        should_add_point = distance_m >= kOdomTrailPointDistanceThresholdM ||
                           yaw_delta_rad >= kOdomTrailPointYawThresholdRad;
    }

    if (should_add_point)
    {
        _odom_trail.push_back({static_cast<float>(estimate.x_odom_m),
                               static_cast<float>(estimate.y_odom_m), 0.05F});
        _last_odom_trail_point_yaw_rad = estimate.yaw_odom_rad;
        while (_odom_trail.size() > kMaximumOdomTrailPoints)
        {
            _odom_trail.pop_front();
        }
    }

    if (_last_odom_trail_publish_stamp_ns &&
        estimate.timestamp_ns < *_last_odom_trail_publish_stamp_ns)
    {
        return;
    }

    if (_last_odom_trail_publish_stamp_ns &&
        estimate.timestamp_ns - *_last_odom_trail_publish_stamp_ns <
            kOdomTrailPublishPeriodNs)
    {
        return;
    }

    _last_odom_trail_publish_stamp_ns = estimate.timestamp_ns;

    std::vector<core::xyz_vec<float>> path;
    path.reserve(_odom_trail.size());
    for (const auto& point : _odom_trail)
    {
        path.push_back(core::xyz_vec<float>{point[0], point[1], point[2]});
    }

    core::render_path_at("/viz/odom_trail", path, "odom_trail", "odom",
                         estimate.timestamp_ns, 0.05F);
}

}  // namespace estimation
