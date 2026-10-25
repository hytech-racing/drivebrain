#include "ConeClassifier.hpp"
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <foxglove/ImageAnnotations.pb.h>
#include <spdlog/spdlog.h>
#if JETSON_ENABLED
#include <opencv2/imgproc.hpp>
#endif
#include "FoxgloveServer.hpp"
#include "LidarFilter.hpp"
#include "Telemetry.hpp"

namespace perception {
namespace {

    constexpr std::array<dv_msgs::Cones::ConeColor, 4> CLASS_COLORS{
        dv_msgs::Cones::BLUE, dv_msgs::Cones::YELLOW, dv_msgs::Cones::ORANGE_SMALL, dv_msgs::Cones::UNKNOWN};

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

    std::optional<std::array<float, 4>> project_cone(const CameraModel& cam, const Eigen::Isometry3f& T_cam_lidar, const hytech_msgs::xyz_vector& position) {
        // Square of fixed metric size around the cone's mid-height, shifted (not clipped) to stay inside the image
        const Eigen::Vector3f center(position.x(), position.y(), -SENSOR_HEIGHT_ABOVE_GROUND_M + CONE_HEIGHT_M / 2);
        const Eigen::Vector3f p_cam = T_cam_lidar * center;
        const auto px = cam.project(p_cam);
        if (!px || px->x() < 0 || px->x() >= cam.width || px->y() < 0 || px->y() >= cam.height) return std::nullopt;

        const float side = cam.fy * PATCH_SIZE_M / p_cam.z();
        if (side > cam.width || side > cam.height) return std::nullopt;
        const float x0 = std::clamp(px->x() - side / 2, 0.0f, cam.width - side);
        const float y0 = std::clamp(px->y() - side / 2, 0.0f, cam.height - side);
        return std::array<float, 4>{x0, y0, x0 + side, y0 + side};
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
#if JETSON_ENABLED
        try {
            _detector = std::make_unique<comms::Detector>("/home/hytech/engines/cone_color.engine");
        } catch (const std::exception& e) {
            spdlog::error("[ConeClassifier] cone coloring disabled: {}", e.what());
        }
#endif

        namespace fs = std::filesystem;
        fs::path dir = intrinsics_dir;
        if (!fs::is_directory(dir) && dir.is_relative()) {
            std::error_code ec;
            const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
            if (!ec) dir = exe.parent_path() / intrinsics_dir;
        }
        if (!fs::is_directory(dir)) {
            spdlog::warn("[ConeClassifier] intrinsics directory '{}' not found, cones will not be projected", intrinsics_dir);
            return;
        }

        for (const auto& entry : fs::directory_iterator(dir)) {
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

    std::shared_ptr<dv_msgs::Cones> ConeClassifier::classify(std::shared_ptr<dv_msgs::Cones> cones, const std::map<std::string, std::shared_ptr<const foxglove::RawImage>>& frames) {
        if (frames.empty()) {
            spdlog::warn("[ConeClassifier] no camera frames available, no cone_boxes will be published");
        }
        for (const auto& [name, frame] : frames) {
            const auto cam_it = _cameras.find(name);
            if (cam_it == _cameras.end() || !frame) {
                // The frame's frame_id must match a loaded intrinsics key (blackfly_<serial>).
                spdlog::warn("[ConeClassifier] no intrinsics for camera frame '{}' (loaded {} cameras), skipping",
                             name, _cameras.size());
                continue;
            }
             const CameraModel& cam = cam_it->second;
            const Eigen::Isometry3f T_cam_lidar = camera_from_lidar(name);

            // boxes of the cones in view, by cone index
            std::vector<std::pair<int, std::array<float, 4>>> boxes;
            for (int i = 0; i < cones->cones_size(); ++i) {
                if (const auto box = project_cone(cam, T_cam_lidar, cones->cones(i).position())) boxes.emplace_back(i, *box);
            }

#if JETSON_ENABLED
            if (_detector && !boxes.empty() && frame->encoding() == "bayer_bggr8") {
                const cv::Mat bayer(frame->height(), frame->width(), CV_8UC1, const_cast<char*>(frame->data().data()), frame->step());
                cv::Mat rgb;
                cv::cvtColor(bayer, rgb, cv::COLOR_BayerBGGR2RGB);

                std::vector<cv::Mat> patches;
                for (const auto& [i, box] : boxes) {
                    const auto [x0, y0, x1, y1] = box;
                    cv::Mat patch;
                    cv::resize(rgb(cv::Rect(cv::Point(x0, y0), cv::Point(x1, y1))), patch, cv::Size(PATCH_PX, PATCH_PX), 0, 0, cv::INTER_AREA);
                    patches.push_back(patch);
                }

                const auto detections = _detector->classify(patches);
                for (std::size_t k = 0; k < detections.size(); ++k) {
                    auto* cone = cones->mutable_cones(boxes[k].first);
                    if (detections[k].score > cone->color_confidence()) {
                        cone->set_color(CLASS_COLORS.at(detections[k].class_id));
                        cone->set_color_confidence(detections[k].score);
                    }
                }
            }
#endif

            auto annotations = std::make_shared<foxglove::ImageAnnotations>();
            for (const auto& [i, box] : boxes) {
                const auto [x0, y0, x1, y1] = box;

                auto* outline = annotations->add_points();
                *outline->mutable_timestamp() = frame->timestamp();
                outline->set_type(foxglove::PointsAnnotation::LINE_LOOP);
                const std::array<std::pair<float, float>, 4> corners{{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
                for (const auto& [x, y] : corners) {
                    auto* p = outline->add_points();
                    p->set_x(x);
                    p->set_y(y);
                }
                auto* color = outline->mutable_outline_color();
                color->set_r(1.0);  color->set_g(1.0);  color->set_b(1.0);  color->set_a(1.0);
                outline->set_thickness(2);
            }
            core::log(annotations, name + "/cone_boxes");
        }
        return cones;
    }

}
