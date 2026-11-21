#pragma once

#include <array>
#include <cmath>

#include "StateTracker.hpp"
#include <MCAPLogger.hpp>
#include <FoxgloveServer.hpp>
#include <foxglove/SceneUpdate.pb.h>
#include <foxglove/FrameTransform.pb.h>
#include <Geometry.hpp>
#include "dv_msgs.pb.h"

namespace core {

/**
 * Global method for simultaneously logging to an MCAP and streaming to the Foxglove client.
 *
 * @msg The protobuf message to log and stream.
*/
inline void log(std::shared_ptr<const google::protobuf::Message> msg, const std::string& topic = {}) {
    if (!msg) {
        return;
    }
    const std::string& t = topic.empty() ? msg->GetDescriptor()->name() : topic;
    MCAPLogger::instance().log_msg(msg, t);
    FoxgloveServer::instance().send_live_telem_msg(msg, t);
}

/**
 * Logs the given protobuf message to the MCAP file only.
 * 
 * @msg The protobuf message to log to the MCAP file only.
 */
inline void log_mcap_only(std::shared_ptr<const google::protobuf::Message> msg, const std::string &topic = "") {
    MCAPLogger::instance().log_msg(msg, topic);
}


/**
 * Logs the given protobuf message to the Foxglove client only.
 * 
 * @msg The protobuf message to log to the Foxglove client only.
 */
inline void log_foxglove_only(std::shared_ptr<const google::protobuf::Message> msg, const std::string &topic = "") {
    FoxgloveServer::instance().send_live_telem_msg(msg, topic);
}

/**
 * Publishes a parent <- child transform on the tf topic
 *
 * @param parent_frame_id the parent frame
 * @param child_frame_id the child frame
 * @param timestamp_ns host time the transform is valid at
 * @param T_parent_child pose of the child frame expressed in the parent frame
 */
inline void publish_transform(const std::string& parent_frame_id, const std::string& child_frame_id,
                              uint64_t timestamp_ns, const Eigen::Isometry3d& T_parent_child) {
    auto tf = std::make_shared<foxglove::FrameTransform>();
    tf->mutable_timestamp()->set_seconds(static_cast<int64_t>(timestamp_ns / 1000000000ULL));
    tf->mutable_timestamp()->set_nanos(static_cast<int32_t>(timestamp_ns % 1000000000ULL));
    tf->set_parent_frame_id(parent_frame_id);
    tf->set_child_frame_id(child_frame_id);

    const Eigen::Vector3d t = T_parent_child.translation();
    tf->mutable_translation()->set_x(t.x());
    tf->mutable_translation()->set_y(t.y());
    tf->mutable_translation()->set_z(t.z());

    const Eigen::Quaterniond q(T_parent_child.linear());
    tf->mutable_rotation()->set_x(q.x());
    tf->mutable_rotation()->set_y(q.y());
    tf->mutable_rotation()->set_z(q.z());
    tf->mutable_rotation()->set_w(q.w());

    log(tf, "tf");
}

/**
 * Renders DV cones in Foxglove's 3D panel for viz
 *
 * @param cones the cones to log
 * @param id The identifier of the cone map (used to distinguish between ground truth and estimated cone maps)
 * @param frame_id The frame the cone positions are expressed in.
 */
inline void render_cones(std::shared_ptr<dv_msgs::Cones> cones, std::string id, std::string frame_id = "map") {
    log(cones, id);

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
        pos->set_z(cone.position().z());
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

        // SLAM landmarks carry an id; they are drawn faded until the optimizer has placed them
        if (cone.landmark_id()) {
            color->set_a(cone.optimized() ? 1.0 : 0.5);
            auto* text = entity->add_texts();
            auto* text_pos = text->mutable_pose()->mutable_position();
            text_pos->set_x(cone.position().x());
            text_pos->set_y(cone.position().y());
            text_pos->set_z(cone.position().z() + 0.5);
            text->mutable_pose()->mutable_orientation()->set_w(1.0);
            text->set_billboard(true);
            text->set_font_size(12.0);
            text->set_scale_invariant(true);
            text->mutable_color()->set_r(1.0);
            text->mutable_color()->set_g(1.0);
            text->mutable_color()->set_b(1.0);
            text->mutable_color()->set_a(0.9);
            text->set_text(std::to_string(cone.landmark_id()));
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
 * @param rgb Box colour. Default red (the SLAM pose); the sim's ground truth is drawn blue so the two can be told apart.
 */
inline void render_pose(std::shared_ptr<hytech_msgs::pose> pose, std::string id, std::string frame_id = "map", std::array<float, 3> rgb = {0.86f, 0.2f, 0.2f}) {
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
    color->set_r(rgb[0]);
    color->set_g(rgb[1]);
    color->set_b(rgb[2]);
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
inline void render_path(const std::vector<xyz_vec<float>>& path, std::string id, std::string frame_id = "map",
                        std::array<double, 4> rgba = {0.0, 0.9, 0.5, 1.0}) {
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
    color->set_r(rgba[0]);
    color->set_g(rgba[1]);
    color->set_b(rgba[2]);
    color->set_a(rgba[3]);

    log(scene);
}

/**
 * Renders a planned path in Foxglove's 3D panel as a flat triangle strip
 * 
 * @param path the path points to render, in order
 * @param id The identifier of the path (used to distinguish e.g. a midline from a raceline)
 * @param frame_id The frame the path points are expressed in.
 */
inline void render_thick_path(const std::vector<xyz_vec<float>>& path, std::string id, std::string frame_id = "map",
                        std::array<double, 4> rgba = {0.0, 0.9, 0.5, 1.0}, float thickness = 1.5) {
    auto scene = std::make_shared<foxglove::SceneUpdate>();
    auto* entity = scene->add_entities();
    entity->set_frame_id(frame_id);
    entity->set_id(id);

    auto now = std::chrono::system_clock::now().time_since_epoch();
    entity->mutable_timestamp()->set_seconds(std::chrono::duration_cast<std::chrono::seconds>(now).count());
    entity->mutable_timestamp()->set_nanos(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000);
    entity->set_frame_locked(true);

    if (path.size() < 2) {
        log(scene);
        return; //atleast 2 points needed to draw path
    }

    auto* line = entity->add_triangles();
    line->mutable_pose()->mutable_orientation()->set_w(1.0);

    const float half_thick = thickness/2;
    const float hue_shift_freq = 720;

    int left_vertex_index = 0;
    int right_vertex_index = 1;

    float cur_hue = 0;

    auto add_tri = [&](int i1, int i2, int i3) { 
        line->add_indices(i1);
        line->add_indices(i2);
        line->add_indices(i3);
    };
    /*
    auto add_color = [&](float hue) { 
        auto* c = line->add_colors();

        //covert hue to rbg
        float r = 0;
        float g = 0;
        float b = 0;
        hue = std::fmod(hue, 300);
        int group = std::floor(hue/60);
        float match = hue/60.0 - group;
        switch (group) {
            case 0:
                r = 1;
                g = match;
                break;
            case 1:
                r = 1-match;
                g = 1;
                break;
            case 2:
                g = 1;
                b = match;
                break;
            case 3:
                g = 1-match;
                b = 1;
                break;
            case 4:
                b = 1;
                r = match;
                break;
        }

        c->set_r(r);
        c->set_b(g);
        c->set_g(b);
        c->set_a(rgba[3]);
    };
    */
    auto add_vertex = [&](float x, float y, float z) { 
        auto* p = line->add_points();
        p->set_x(x);
        p->set_y(y);
        p->set_z(z);
        //add_color(cur_hue);
    };
    auto add_vertex_left = [&](float x, float y, float z) { 
        add_vertex(x,y,z);
        add_tri(left_vertex_index, right_vertex_index, line->points_size()-1);
        left_vertex_index = line->points_size()-1;
    };
    auto add_vertex_right = [&](float x, float y, float z) { 
        add_vertex(x,y,z);
        add_tri(left_vertex_index, right_vertex_index, line->points_size()-1);
        right_vertex_index = line->points_size()-1;
    };

    for (int i = 0; i < path.size(); i++)
    {
        if (i == 0) {
            float dx = path[i+1].x - path[i].x;
            float dy = path[i+1].y - path[i].y;
            float mag = std::sqrt(dx*dx + dy*dy);
            dx *= half_thick/mag;
            dy *= half_thick/mag;

            add_vertex(path[i].x-dy, path[i].y+dx, path[i].z);
            add_vertex(path[i].x+dy, path[i].y-dx, path[i].z);

            cur_hue += hue_shift_freq*mag;

            continue;
        } else if (i == path.size()-1) {
            float dx = path[i].x - path[i-1].x;
            float dy = path[i].y - path[i-1].y;
            float mag = std::sqrt(dx*dx + dy*dy);
            dx *= half_thick/mag;
            dy *= half_thick/mag;

            add_vertex_left(path[i].x-dy, path[i].y+dx, path[i].z);
            add_vertex_right(path[i].x+dy, path[i].y-dx, path[i].z);

            continue;
        }

        const auto& prev = path[i-1];
        const auto& cur = path[i];
        const auto& next = path[i+1];

        float prev_dx = cur.x-prev.x;
        float prev_dy = cur.y-prev.y;
        float next_dx = next.x-cur.x;
        float next_dy = next.y-cur.y;

        float prev_mag = std::sqrt(prev_dx*prev_dx + prev_dy*prev_dy);
        float next_mag = std::sqrt(next_dx*next_dx + next_dy*next_dy);

        float offset1_x = -half_thick*prev_dy/prev_mag;
        float offset1_y = half_thick*prev_dx/prev_mag;
        float offset2_x = -half_thick*next_dy/next_mag;
        float offset2_y = half_thick*next_dx/next_mag;


        float ax, ay, bx, by; //the coodinates on the side we calc the intersection for

        //check the direction the path curves,
        //for the side that curves inwards, find the intersect of the path edges, place a single point there
        //on the side we curve away from, add an extra triangle to fill were the elbo breaks
        float delta_det = prev_dx*next_dy - prev_dy*next_dx; //positive means curve left, negative curve right

        if (std::abs(delta_det) < 0.1*prev_mag*next_mag) { //parallel case, or if we're close enoguh to being paralell
            add_vertex_left(path[i].x+offset1_x, path[i].y+offset1_y, path[i].z);
            add_vertex_right(path[i].x-offset1_x, path[i].y-offset1_y, path[i].z);
            continue;
        } else if (delta_det > 0) {
            ax = prev.x + offset1_x;
            ay = prev.y + offset1_y;
            bx = next.x + offset2_x;
            by = next.y + offset2_y;
        } else {
            ax = prev.x - offset1_x;
            ay = prev.y - offset1_y;
            bx = next.x - offset2_x;
            by = next.y - offset2_y;
        }
        
        float delta_prev_det = prev_dy*ax - prev_dx*ay;
        float delta_next_det = next_dy*bx - next_dx*by;
        float intersect_x = (prev_dx*delta_next_det - next_dx*delta_prev_det) / delta_det;
        float intersect_y = (prev_dy*delta_next_det - next_dy*delta_prev_det) / delta_det;

        float elbow_offset_x = offset1_x + offset2_x;
        float elbow_offset_y = offset1_y + offset2_y;
        float elbow_offset_mag = std::sqrt(elbow_offset_x*elbow_offset_x + elbow_offset_y*elbow_offset_y);
        elbow_offset_x *= half_thick/elbow_offset_mag;
        elbow_offset_y *= half_thick/elbow_offset_mag;

        //order which we add points depends on bend direction, since we want the left vertex to have lower index
        if (delta_det > 0) {
            add_vertex_left(intersect_x, intersect_y, cur.z);
            add_vertex_right(cur.x - offset1_x, cur.y - offset1_y, cur.z);
            add_vertex_right(cur.x - elbow_offset_x, cur.y - elbow_offset_y, cur.z);
            add_vertex_right(cur.x - offset2_x, cur.y - offset2_y, cur.z);
        } else {
            add_vertex_right(intersect_x, intersect_y, cur.z);
            add_vertex_left(cur.x + offset1_x, cur.y + offset1_y, cur.z);
            add_vertex_left(cur.x + elbow_offset_x, cur.y + elbow_offset_y, cur.z);
            add_vertex_left(cur.x + offset2_x, cur.y + offset2_y, cur.z);
        }

        cur_hue += hue_shift_freq*prev_mag;
    }
    
    /*
    auto* color = line->mutable_color();
    color->set_r(rgba[0]);
    color->set_g(rgba[1]);
    color->set_b(rgba[2]);
    color->set_a(rgba[3]);
    */
    

    log(scene);
}

/**
 * Renders line segments in Foxglove's 3D panel, one per consecutive pair of points
 *
 * @param points segment endpoints, in pairs
 * @param id The identifier of the segment set (e.g. "slam_factors")
 * @param frame_id The frame the points are expressed in.
 */
inline void render_segments(const std::vector<xyz_vec<float>>& points, std::string id, std::string frame_id = "map") {
    auto scene = std::make_shared<foxglove::SceneUpdate>();
    auto* entity = scene->add_entities();
    entity->set_frame_id(frame_id);
    entity->set_id(id);
    auto now = std::chrono::system_clock::now().time_since_epoch();
    entity->mutable_timestamp()->set_seconds(std::chrono::duration_cast<std::chrono::seconds>(now).count());
    entity->mutable_timestamp()->set_nanos(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000);
    entity->set_frame_locked(true);

    auto* line = entity->add_lines();
    line->set_type(foxglove::LinePrimitive::LINE_LIST);
    line->mutable_pose()->mutable_orientation()->set_w(1.0);
    line->set_thickness(0.02);
    for (const auto& point : points) {
        auto* p = line->add_points();
        p->set_x(point.x);
        p->set_y(point.y);
        p->set_z(point.z);
    }
    // muted so the factor web sits behind the cones and the trail
    auto* color = line->mutable_color();
    color->set_r(0.55);
    color->set_g(0.55);
    color->set_b(0.55);
    color->set_a(0.35);

    log(scene);
}
}
