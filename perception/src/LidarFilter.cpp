#include "LidarFilter.hpp"

namespace perception {
namespace {

    struct Cell {
        std::vector<std::size_t> point_indices;
    };

    int flatten_index(int radial_index, int angular_index, int num_angular_bins) {
        return radial_index * num_angular_bins + angular_index;
    }

    /**
     * Removes ground points from the input point cloud.
     * Points are classified as ground or non-ground based on a grid-based height estimation.
     * 
     * @param in The input point cloud.
     * @return A new point cloud containing only non-ground points.
     */
    std::shared_ptr<const foxglove::PointCloud> remove_ground(const foxglove::PointCloud& in) {

        static bool logged = false;
        if (!logged) {
            logged = true;
            spdlog::info("stride {}, {} bytes", in.point_stride(), in.data().size());
            for (const auto& f : in.fields())
                spdlog::info("  {} offset {} type {}", f.name(), f.offset(), static_cast<int>(f.type()));
        }

        const auto scan = core::points(in);

        // Determine the grid size
        int num_angular_bins = std::ceil((GRID_MAX_THETA_RAD - GRID_MIN_THETA_RAD) / GRID_ANGULAR_BIN_SIZE_RAD);
        int num_radial_bins = std::ceil((GRID_MAX_RANGE_M - GRID_MIN_RANGE_M) / GRID_RADIAL_BIN_SIZE_M);
        std::vector<Cell> cells(num_radial_bins * num_angular_bins);

        // Assign points to cells
        for (std::size_t i = 0; i < scan.size(); ++i) {
            const auto& point = scan[i];
            const float ri = (std::hypot(point.x, point.y) - GRID_MIN_RANGE_M) / GRID_RADIAL_BIN_SIZE_M;
            const float ti = (std::atan2(point.y, point.x) - GRID_MIN_THETA_RAD) / GRID_ANGULAR_BIN_SIZE_RAD;
            if (ri < 0 || ri >= num_radial_bins || ti < 0 || ti >= num_angular_bins) continue;
            cells[flatten_index(static_cast<int>(ri), static_cast<int>(ti), num_angular_bins)].point_indices.push_back(i);
        }

        // Estimate the ground height for each cell by looking at the nth pensicile and classify points as ground or non-ground
        std::vector<float> z;
        std::vector<core::LidarPoint> non_ground;
        non_ground.reserve(scan.size());

        for (auto& cell : cells) {
            z.clear();
            for (auto i : cell.point_indices) z.push_back(scan[i].z);

            float ground = FLAT_GROUND_Z_MAX_M;
            // Get nth percentile of the z-values to estimate the ground height
            if (!z.empty() && z.size() >= MIN_POINTS_PER_CELL) {
                const auto nth = static_cast<std::ptrdiff_t>(GROUND_PERCENTILE * (z.size()-1));
                std::nth_element(z.begin(), z.begin() + nth, z.end());
                ground = z[nth];
            }

            for (auto i : cell.point_indices) {
                if (scan[i].z > ground + NON_GROUND_HEIGHT_THRESHOLD_M) non_ground.push_back(scan[i]);
            }
        }

        return core::to_cloud(non_ground, in);
    }
} // END OF ANONYMOUS NAMESPACE

    std::shared_ptr<const foxglove::PointCloud> filter_cloud(const foxglove::PointCloud& scan) {
        return remove_ground(scan);
    }

}