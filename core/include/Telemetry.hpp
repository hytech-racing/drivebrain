#pragma once

#include "StateTracker.hpp"
#include <MCAPLogger.hpp>
#include <FoxgloveServer.hpp>
#include <foxglove/SceneUpdate.pb.h>
#include "dv_msgs.pb.h"

namespace core {

/**
 * Global method for simultaneously logging to an MCAP and streaming to the Foxglove client.
 *
 * @msg The protobuf message to log and stream.
*/
inline void log(std::shared_ptr<google::protobuf::Message> msg) {
    MCAPLogger::instance().log_msg(msg);
    FoxgloveServer::instance().send_live_telem_msg(msg);
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
}
