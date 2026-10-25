#include "ConeClassifier.hpp"
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <spdlog/spdlog.h>
#include "FoxgloveServer.hpp"
#include "LidarFilter.hpp"
#include "Telemetry.hpp"

namespace perception {
namespace {

    const Eigen::Matrix3f R_LIDAR_FLU = (Eigen::Matrix3f() <<
         0, 1, 0,
        -1, 0, 0,
         0, 0, 1).finished();

    const Eigen::Matrix3f R_FLU_CAM = (Eigen::Matrix3f() <<
         0,  0, 1,
        -1,  0, 0,
         0, -1, 0).finished();

    std::vector<float> parse_list(const std::string& text, const std::string& key) {
        const std::regex re("'" + key + "'\\s*:\\s*\\[([^\\]]*)\\]");
        std::smatch match;
        if (!std::regex_search(text, match, re)) throw std::runtime_error("missing '" + key + "'");

        std::vector<float> values;
        std::stringstream ss(match[1].str());
        std::string token;
        while (std::getline(ss, token, ',')) {
            if (token.find_first_not_of(" \t\n") != std::string::npos) values.push_back(std::stof(token));
        }
        return values;
    }

    Eigen::Isometry3f camera_from_lidar(const std::string& camera) {
        float yaw_deg = core::FoxgloveServer::instance().get_param<float>("ConeClassifier/" + camera + "/yaw_deg").value_or(0.0f);
        float pitch_deg = core::FoxgloveServer::instance().get_param<float>("ConeClassifier/" + camera + "/pitch_deg").value_or(0.0f);
        float roll_deg = core::FoxgloveServer::instance().get_param<float>("ConeClassifier/" + camera + "/roll_deg").value_or(0.0f);
        float forward_m = core::FoxgloveServer::instance().get_param<float>("ConeClassifier/" + camera + "/forward_m").value_or(0.0f);
        float left_m = core::FoxgloveServer::instance().get_param<float>("ConeClassifier/" + camera + "/left_m").value_or(0.0f);
        float up_m = core::FoxgloveServer::instance().get_param<float>("ConeClassifier/" + camera + "/up_m").value_or(0.0f);

        const auto deg = [](float d) { return d * static_cast<float>(M_PI) / 180.0f; };

        const Eigen::Matrix3f R_mount = (Eigen::AngleAxisf(deg(yaw_deg), Eigen::Vector3f::UnitZ())
                                       * Eigen::AngleAxisf(deg(pitch_deg), Eigen::Vector3f::UnitY())
                                       * Eigen::AngleAxisf(deg(roll_deg), Eigen::Vector3f::UnitX())).toRotationMatrix();

        Eigen::Isometry3f T_lidar_cam = Eigen::Isometry3f::Identity();
        T_lidar_cam.linear() = R_LIDAR_FLU * R_mount * R_FLU_CAM;
        T_lidar_cam.translation() = R_LIDAR_FLU * Eigen::Vector3f(forward_m, left_m, up_m);
        return T_lidar_cam.inverse();
    }

    /* Same colors as core::render_cones, gray for UNKNOWN */
    void set_cone_color(foxglove::Color& color, dv_msgs::Cones::ConeColor cone_color, double alpha) {
        color.set_a(alpha);
        switch (cone_color) {
            case dv_msgs::Cones::BLUE:         color.set_r(0.0); color.set_g(0.3);  color.set_b(1.0); break;
            case dv_msgs::Cones::YELLOW:       color.set_r(1.0); color.set_g(0.85); color.set_b(0.0); break;
            case dv_msgs::Cones::ORANGE_SMALL: color.set_r(1.0); color.set_g(0.45); color.set_b(0.0); break;
            case dv_msgs::Cones::ORANGE_BIG:   color.set_r(1.0); color.set_g(0.25); color.set_b(0.0); break;
            default:                           color.set_r(0.5); color.set_g(0.5);  color.set_b(0.5); break;
        }
    }

    // A detection placed on the ground plane, in the LiDAR frame
    struct CameraCone {
        Eigen::Vector2f xy;
        dv_msgs::Cones::ConeColor color;
        float score;
    };

    /**
     * Places each detection by casting a ray through the bottom center of its box (where the cone meets the ground)
     * and intersecting it with the ground plane. Detections that don't hit the ground within MAX_DETECTION_RANGE_M
     * are dropped.
     */
    std::vector<CameraCone> place_detections(const CameraModel& cam, const Eigen::Isometry3f& T_cam_lidar,
                                             const std::vector<core::ConeDetection>& dets) {
        const Eigen::Isometry3f T_lidar_cam = T_cam_lidar.inverse();
        const Eigen::Vector3f origin = T_lidar_cam.translation();
        const float ground_z = -SENSOR_HEIGHT_ABOVE_GROUND_M;

        std::vector<CameraCone> placed;
        for (const auto& d : dets) {
            const Eigen::Vector2f xy = cam.unproject(Eigen::Vector2f((d.x0 + d.x1) / 2, d.y1));
            const Eigen::Vector3f dir = T_lidar_cam.linear() * Eigen::Vector3f(xy.x(), xy.y(), 1.0f);
            if (dir.z() >= 0) continue; // at or above the horizon, never reaches the ground
            const Eigen::Vector3f ground = origin + (ground_z - origin.z()) / dir.z() * dir;
            if (std::hypot(ground.x(), ground.y()) > MAX_DETECTION_RANGE_M) continue;
            placed.push_back({ground.head<2>(), d.color, d.score});
        }
        return placed;
    }

    /**
     * Logs the camera-placed cones as dv_msgs::Cones on <camera>/cones and draws them translucent in the 3D panel
     * as entity <camera>_cones, to compare against where the LiDAR found the cones.
     */
    void render_camera_cones(const std::string& camera, const std::vector<CameraCone>& placed, uint64_t timestamp_us, const std::string& frame_id) {
        auto cones = std::make_shared<dv_msgs::Cones>();
        cones->set_timestamp_us(timestamp_us);
        for (const auto& p : placed) {
            auto* cone = cones->add_cones();
            cone->set_color(p.color);
            cone->mutable_position()->set_x(p.xy.x());
            cone->mutable_position()->set_y(p.xy.y());
            cone->mutable_position()->set_z(-SENSOR_HEIGHT_ABOVE_GROUND_M + CONE_HEIGHT_M / 2);
        }
        core::log(cones, camera + "/cones");

        auto scene = std::make_shared<foxglove::SceneUpdate>();
        auto* entity = scene->add_entities();
        entity->set_frame_id(frame_id);
        entity->set_id(camera + "_cones");
        entity->mutable_timestamp()->set_seconds(static_cast<int64_t>(timestamp_us / 1000000));
        entity->mutable_timestamp()->set_nanos(static_cast<int32_t>((timestamp_us % 1000000) * 1000));
        for (const auto& cone : cones->cones()) {
            auto* cyl = entity->add_cylinders();
            auto* pos = cyl->mutable_pose()->mutable_position();
            pos->set_x(cone.position().x());
            pos->set_y(cone.position().y());
            pos->set_z(cone.position().z());
            cyl->mutable_pose()->mutable_orientation()->set_w(1.0);
            cyl->mutable_size()->set_x(CONE_WIDTH_M);
            cyl->mutable_size()->set_y(CONE_WIDTH_M);
            cyl->mutable_size()->set_z(CONE_HEIGHT_M);
            cyl->set_bottom_scale(1.0);
            cyl->set_top_scale(0.25);
            set_cone_color(*cyl->mutable_color(), cone.color(), 0.5);
        }
        core::log(scene);
    }

} // END OF ANONYMOUS NAMESPACE

    Eigen::Vector2f CameraModel::unproject(const Eigen::Vector2f& px) const {
        // Fixed-point inversion of the OpenCV8 rational distortion, as in OpenCV's undistortPoints
        const auto [k1, k2, p1, p2, k3, k4, k5, k6] = dist;
        const float xd = (px.x() - cx) / fx, yd = (px.y() - cy) / fy;
        float x = xd, y = yd;
        for (int iter = 0; iter < 20; ++iter) {
            const float r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
            const float inv_radial = (1 + k4 * r2 + k5 * r4 + k6 * r6) / (1 + k1 * r2 + k2 * r4 + k3 * r6);
            const float dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
            const float dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
            x = (xd - dx) * inv_radial;
            y = (yd - dy) * inv_radial;
        }
        return Eigen::Vector2f(x, y);
    }

    CameraModel load_camera_model(const std::string& path) {
        std::ifstream file(path);
        if (!file) throw std::runtime_error("failed to open " + path);
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string text = buffer.str();
        text = text.substr(0, text.find("'optimization_inputs'"));

        if (text.find("LENSMODEL_OPENCV8") == std::string::npos) throw std::runtime_error("only LENSMODEL_OPENCV8 is supported: " + path);
        const auto intrinsics = parse_list(text, "intrinsics");
        const auto size = parse_list(text, "imagersize");
        if (intrinsics.size() != 12 || size.size() != 2) throw std::runtime_error("unexpected intrinsics layout: " + path);

        CameraModel cam;
        cam.fx = intrinsics[0];  cam.fy = intrinsics[1];
        cam.cx = intrinsics[2];  cam.cy = intrinsics[3];
        std::copy(intrinsics.begin() + 4, intrinsics.end(), cam.dist.begin());
        cam.width = static_cast<int>(size[0]);
        cam.height = static_cast<int>(size[1]);

        cam.cx = cam.width - 1 - cam.cx;
        cam.cy = cam.height - 1 - cam.cy;
        cam.dist[2] = -cam.dist[2];
        cam.dist[3] = -cam.dist[3];
        return cam;
    }

    ConeClassifier::ConeClassifier(const std::string& intrinsics_dir) {
        namespace fs = std::filesystem;
        if (!fs::is_directory(intrinsics_dir)) {
            spdlog::warn("[ConeClassifier] intrinsics directory '{}' not found, cones will not be projected", intrinsics_dir);
            return;
        }

        for (const auto& entry : fs::directory_iterator(intrinsics_dir)) {
            if (entry.path().extension() != ".cameramodel") continue;
            const std::string stem = entry.path().stem().string();
            const std::string name = "blackfly_" + stem.substr(0, stem.find('-'));
            try {
                _cameras[name] = load_camera_model(entry.path().string());
                spdlog::info("[ConeClassifier] loaded intrinsics for {} (fx {:.1f})", name, _cameras[name].fx);
            } catch (const std::exception& e) {
                spdlog::error("[ConeClassifier] skipping {}: {}", entry.path().string(), e.what());
            }
        }
    }

    std::shared_ptr<dv_msgs::Cones> ConeClassifier::classify(std::shared_ptr<dv_msgs::Cones> cones,
                                                             const std::map<std::string, std::shared_ptr<const std::vector<core::ConeDetection>>>& detections,
                                                             const std::string& frame_id) {
        // Score of the detection that colored each cone, so the more confident camera wins when both see it
        std::vector<float> best_score(cones->cones_size(), 0.0f);

        for (const auto& [name, dets] : detections) {
            const auto cam_it = _cameras.find(name);
            if (cam_it == _cameras.end() || !dets) continue;
            const std::vector<CameraCone> placed = place_detections(cam_it->second, camera_from_lidar(name), *dets);
            render_camera_cones(name, placed, cones->timestamp_us(), frame_id);

            // (distance, cone, detection) for every pair within the match distance
            std::vector<std::tuple<float, int, int>> pairs;
            for (int i = 0; i < cones->cones_size(); ++i) {
                const auto& pos = cones->cones(i).position();
                for (int j = 0; j < static_cast<int>(placed.size()); ++j) {
                    const float dist = (placed[j].xy - Eigen::Vector2f(pos.x(), pos.y())).norm();
                    if (dist < MATCH_DISTANCE_M) pairs.emplace_back(dist, i, j);
                }
            }

            // Greedy one-to-one matching, closest pairs first
            std::sort(pairs.begin(), pairs.end());
            std::vector<bool> cone_used(cones->cones_size()), det_used(placed.size());
            for (const auto& [dist, i, j] : pairs) {
                if (cone_used[i] || det_used[j]) continue;
                cone_used[i] = det_used[j] = true;
                if (placed[j].score > best_score[i]) {
                    best_score[i] = placed[j].score;
                    cones->mutable_cones(i)->set_color(placed[j].color);
                    cones->mutable_cones(i)->set_color_confidence(placed[j].score);
                }
            }
        }

        return cones;
    }

}
