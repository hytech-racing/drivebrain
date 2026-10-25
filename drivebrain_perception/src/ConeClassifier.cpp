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

} // END OF ANONYMOUS NAMESPACE

    std::optional<Eigen::Vector2f> CameraModel::project(const Eigen::Vector3f& p_cam) const {
        if (p_cam.z() < MIN_PROJECTION_DEPTH_M) return std::nullopt;
        const float x = p_cam.x() / p_cam.z();
        const float y = p_cam.y() / p_cam.z();

        const float u = fx * x + cx, v = fy * y + cy;
        if (u < -0.5f * width || u > 1.5f * width || v < -0.5f * height || v > 1.5f * height) return std::nullopt;

        const auto [k1, k2, p1, p2, k3, k4, k5, k6] = dist;
        const float r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
        const float radial = (1 + k1 * r2 + k2 * r4 + k3 * r6) / (1 + k4 * r2 + k5 * r4 + k6 * r6);
        const float xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        const float yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
        return Eigen::Vector2f(fx * xd + cx, fy * yd + cy);
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
                                                             const std::map<std::string, std::shared_ptr<const std::vector<core::ConeDetection>>>& detections) {
        // Score of the detection that colored each cone, so the more confident camera wins when both see it
        std::vector<float> best_score(cones->cones_size(), 0.0f);

        for (const auto& [name, dets] : detections) {
            const auto cam_it = _cameras.find(name);
            if (cam_it == _cameras.end() || !dets) continue;
            const CameraModel& cam = cam_it->second;
            const Eigen::Isometry3f T_cam_lidar = camera_from_lidar(name);

            // (cost, cone, detection) for every plausible pair
            std::vector<std::tuple<float, int, int>> pairs;
            for (int i = 0; i < cones->cones_size(); ++i) {
                const auto& pos = cones->cones(i).position();
                const Eigen::Vector3f p_cam = T_cam_lidar * Eigen::Vector3f(pos.x(), pos.y(), -SENSOR_HEIGHT_ABOVE_GROUND_M + CONE_HEIGHT_M / 2);
                const auto px = cam.project(p_cam);
                if (!px) continue;
                const float expected_height = cam.fy * CONE_HEIGHT_M / p_cam.z();

                for (int j = 0; j < static_cast<int>(dets->size()); ++j) {
                    const auto& d = (*dets)[j];
                    const float height = d.y1 - d.y0;
                    const float ratio = height / expected_height;
                    if (height <= 0 || ratio > MAX_SIZE_RATIO || ratio < 1 / MAX_SIZE_RATIO) continue;
                    const float cost = (Eigen::Vector2f((d.x0 + d.x1) / 2, (d.y0 + d.y1) / 2) - *px).norm() / height;
                    if (cost < MATCH_GATE) pairs.emplace_back(cost, i, j);
                }
            }

            // Greedy one-to-one matching, cheapest pairs first
            std::sort(pairs.begin(), pairs.end());
            std::vector<bool> cone_used(cones->cones_size()), det_used(dets->size());
            for (const auto& [cost, i, j] : pairs) {
                if (cone_used[i] || det_used[j]) continue;
                cone_used[i] = det_used[j] = true;
                const auto& d = (*dets)[j];
                if (d.score > best_score[i]) {
                    best_score[i] = d.score;
                    cones->mutable_cones(i)->set_color(d.color);
                }
            }
        }
        return cones;
    }

}
