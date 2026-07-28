#pragma once

#include "StateTracker.hpp"
#include <chrono>
#include <MCAPLogger.hpp>
#include <FoxgloveServer.hpp>
#include <foxglove/SceneUpdate.pb.h>
#include "dv_msgs.pb.h"
#include <cmath>
#include <cstdint>
#include <foxglove/FrameTransform.pb.h>
#include "RigidTransform2D.hpp"

namespace core {

/**
 * Global method for simultaneously logging to an MCAP and streaming to the Foxglove client.
 *
 * @msg The protobuf message to log and stream.
*/
inline void log(const std::string& topic,
                std::shared_ptr<const google::protobuf::Message> msg) {
    if (!msg) {
        return;
    }
    MCAPLogger::instance().log_msg(topic, msg);
    FoxgloveServer::instance().send_live_telem_msg(topic, msg);
}

inline void log(std::shared_ptr<const google::protobuf::Message> msg) {
    if (!msg) {
        return;
    }
    log(msg->GetDescriptor()->name(), msg);
}

inline void set_timestamp(google::protobuf::Timestamp* timestamp,
                          std::uint64_t timestamp_ns)
{
    timestamp->set_seconds(
        static_cast<int64_t>(timestamp_ns / 1'000'000'000ULL));
    timestamp->set_nanos(
        static_cast<int32_t>(timestamp_ns % 1'000'000'000ULL));
}

/**
 * Renders DV cones in Foxglove's 3D panel for viz
 *
 * @param cones the cones to log
 * @param id The identifier of the cone map (used to distinguish between ground truth and estimated cone maps)
 * @param frame_id The frame the cone positions are expressed in.
 */
inline void render_cones(std::shared_ptr<dv_msgs::Cones> cones, std::string id, std::string frame_id = "map") {
    log(cones);

    auto scene = std::make_shared<foxglove::SceneUpdate>();
    auto* entity = scene->add_entities();
    entity->set_frame_id(frame_id);
    entity->set_id(id);
    entity->mutable_timestamp()->set_seconds(static_cast<int64_t>(cones->timestamp_us() / 1000000));
    entity->mutable_timestamp()->set_nanos(static_cast<int32_t>((cones->timestamp_us() % 1000000) * 1000));

    for (const auto& cone : cones->cones()) {
        auto* cyl = entity->add_cylinders();
        auto* pos = cyl->mutable_pose()->mutable_position();
        pos->set_x(cone.position().x());
        pos->set_y(cone.position().y());
        pos->set_z(0.165);
        cyl->mutable_pose()->mutable_orientation()->set_w(1.0);
        cyl->mutable_size()->set_x(0.23);
        cyl->mutable_size()->set_y(0.23);
        cyl->mutable_size()->set_z(0.33);
        cyl->set_bottom_scale(1.0);
        cyl->set_top_scale(0.25);
        auto* color = cyl->mutable_color();
        color->set_a(1.0);
        switch (cone.color()) {
            case dv_msgs::Cones::BLUE:         color->set_r(0.0); color->set_g(0.3);  color->set_b(1.0); break;
            case dv_msgs::Cones::YELLOW:       color->set_r(1.0); color->set_g(0.85); color->set_b(0.0); break;
            case dv_msgs::Cones::ORANGE_SMALL: color->set_r(1.0); color->set_g(0.45); color->set_b(0.0); break;
            case dv_msgs::Cones::ORANGE_BIG:   color->set_r(1.0); color->set_g(0.25); color->set_b(0.0); break;
            default:                           color->set_r(0.5); color->set_g(0.5);  color->set_b(0.5); break;
        }
    }

    log(scene);
}

/**
 * Renders a vehicle pose in Foxglove's 3D panel as a car-sized box
 *
 * @param pose the pose to render
 * @param id The identifier of the pose (used to distinguish e.g. "gt_pose" from "slam_pose")
 * @param frame_id The frame the pose is expressed in.
 */
inline void render_pose(std::shared_ptr<hytech_msgs::pose> pose, std::string id, std::string frame_id = "map") {
    auto scene = std::make_shared<foxglove::SceneUpdate>();
    auto* entity = scene->add_entities();
    entity->set_frame_id(frame_id);
    entity->set_id(id);

    auto* box = entity->add_cubes();
    auto* pos = box->mutable_pose()->mutable_position();
    pos->set_x(pose->position().x());
    pos->set_y(pose->position().y());
    pos->set_z(pose->position().z() + 0.2);
    auto* ori = box->mutable_pose()->mutable_orientation();
    ori->set_x(pose->orientation().x());
    ori->set_y(pose->orientation().y());
    ori->set_z(pose->orientation().z());
    ori->set_w(pose->orientation().w());
    box->mutable_size()->set_x(2.4);
    box->mutable_size()->set_y(1.2);
    box->mutable_size()->set_z(0.4);
    auto* color = box->mutable_color();
    color->set_r(0.86);
    color->set_g(0.2);
    color->set_b(0.2);
    color->set_a(0.3);

    log(scene);
}

/**
 * Renders a planned path in Foxglove's 3D panel as a line
 *
 * @param path the path points to render, in order
 * @param id The identifier of the path (used to distinguish e.g. a midline from a raceline)
 * @param frame_id The frame the path points are expressed in.
 */
inline void render_path(const std::vector<xyz_vec<float>>& path, std::string id, std::string frame_id = "map") {
    auto scene = std::make_shared<foxglove::SceneUpdate>();
    auto* entity = scene->add_entities();
    entity->set_frame_id(frame_id);
    entity->set_id(id);

    /* Foxglove resolves frame_id through the transform tree at this timestamp, so an
       unset one lands at 1970 and renders at the origin. frame_locked keeps the entity
       following its frame as the car moves instead of snapshotting where it was. */
    auto now = std::chrono::system_clock::now().time_since_epoch();
    entity->mutable_timestamp()->set_seconds(std::chrono::duration_cast<std::chrono::seconds>(now).count());
    entity->mutable_timestamp()->set_nanos(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000);
    entity->set_frame_locked(true);

    auto* line = entity->add_lines();
    line->set_type(foxglove::LinePrimitive::LINE_STRIP);
    line->mutable_pose()->mutable_orientation()->set_w(1.0);
    line->set_thickness(0.1);
    for (const auto& point : path) {
        auto* p = line->add_points();
        p->set_x(point.x);
        p->set_y(point.y);
        p->set_z(point.z);
    }
    auto* color = line->mutable_color();
    color->set_r(0.0);
    color->set_g(0.9);
    color->set_b(0.5);
    color->set_a(1.0);

    log(scene);
}

inline void render_path_at(
    const std::string& topic,
    const std::vector<xyz_vec<float>>& path,
    const std::string& id,
    const std::string& frame_id,
    std::uint64_t timestamp_ns,
    float thickness = 0.1F)
{
    auto scene = std::make_shared<foxglove::SceneUpdate>();
    auto* entity = scene->add_entities();
    entity->set_frame_id(frame_id);
    entity->set_id(id);
    set_timestamp(entity->mutable_timestamp(), timestamp_ns);
    entity->set_frame_locked(true);

    auto* line = entity->add_lines();
    line->set_type(foxglove::LinePrimitive::LINE_STRIP);
    line->mutable_pose()->mutable_orientation()->set_w(1.0);
    line->set_thickness(thickness);
    for (const auto& point : path) {
        auto* p = line->add_points();
        p->set_x(point.x);
        p->set_y(point.y);
        p->set_z(point.z);
    }
    auto* color = line->mutable_color();
    color->set_r(0.0);
    color->set_g(0.9);
    color->set_b(0.5);
    color->set_a(1.0);

    log(topic, scene);
}

inline void publish_transform(
    const std::string& parent_frame_id,
    const std::string& child_frame_id,
    std::uint64_t timestamp_ns,
    const transforms::RigidTransform2D& transform,
    double z_m = 0.0
)
{
    auto tf = std::make_shared<foxglove::FrameTransform>();

    tf->set_parent_frame_id(parent_frame_id);
    tf->set_child_frame_id(child_frame_id);

    set_timestamp(tf->mutable_timestamp(), timestamp_ns);

    tf->mutable_translation()->set_x(transform.x_m);
    tf->mutable_translation()->set_y(transform.y_m);
    tf->mutable_translation()->set_z(z_m);

    const double half_yaw = 0.5 * transform.yaw_rad;

    tf->mutable_rotation()->set_x(0.0);
    tf->mutable_rotation()->set_y(0.0);
    tf->mutable_rotation()->set_z(std::sin(half_yaw));
    tf->mutable_rotation()->set_w(std::cos(half_yaw));

    log(tf);
}

inline std::uint64_t system_time_ns()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

}
