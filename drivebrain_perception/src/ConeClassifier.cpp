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

#if HOOTL_ENABLED
    namespace {
        // Max distance between a detection and a ground-truth cone for them to be the same cone
        constexpr float GROUND_TRUTH_GATE_M = 1.0f;
    }

    // No cameras in sim
    std::optional<Eigen::Vector2f> CameraModel::project(const Eigen::Vector3f&) const {
        return std::nullopt;
    }

    CameraModel load_camera_model(const std::string&) {
        return {};
    }
#endif

    ConeClassifier::ConeClassifier(const std::string& intrinsics_dir) {

#if HOOTL_ENABLED
// No-Op
#endif
    }

    std::shared_ptr<dv_msgs::Cones> ConeClassifier::classify(std::shared_ptr<dv_msgs::Cones> cones) {
#if HOOTL_ENABLED
        // Sim stands in for a perfect camera: each detection takes the color of the nearest ground-truth cone
        const auto ground_truth = core::StateTracker::instance().dv_state().ground_truth_cones;
        if (!ground_truth) {
            static bool warned = false;
            if (!warned) {
                spdlog::warn("ConeClassifier: no ground-truth cones from the sim yet, leaving cones uncolored");
                warned = true;
            }
            return cones;
        }

        // Ground-truth car pose from the sim, same map frame as the ground-truth cones
        const auto vehicle_state = core::StateTracker::instance().vehicle_state().first;
        const auto& pos = vehicle_state.vehicle_position_map_frame;
        const auto& heading = vehicle_state.vehicle_heading_map_frame_unit_vector;
        const Eigen::Isometry3d& T_base_lidar = core::StateTracker::instance().T_base_lidar();

        for (auto& cone : *cones->mutable_cones()) {
            // os_sensor -> base_link -> map
            const Eigen::Vector3d p_base = T_base_lidar * Eigen::Vector3d(cone.position().x(), cone.position().y(), cone.position().z());
            const float x = pos.x + heading.x * p_base.x() - heading.y * p_base.y();
            const float y = pos.y + heading.y * p_base.x() + heading.x * p_base.y();

            const dv_msgs::Cones::Cone* nearest = nullptr;
            float nearest_dist_sq = GROUND_TRUTH_GATE_M * GROUND_TRUTH_GATE_M;
            for (const auto& truth : ground_truth->cones()) {
                const float dx = truth.position().x() - x;
                const float dy = truth.position().y() - y;
                const float dist_sq = dx * dx + dy * dy;
                if (dist_sq < nearest_dist_sq) {
                    nearest_dist_sq = dist_sq;
                    nearest = &truth;
                }
            }

            if (nearest) {
                cone.set_color(nearest->color());
                cone.set_color_confidence(1.0f);
            }
        }
#endif
        return cones;
    }
}
