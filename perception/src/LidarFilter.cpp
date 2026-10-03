#include "LidarFilter.hpp"

namespace perception {
namespace {

    struct Cell {
        std::vector<std::size_t> point_indices;
    };

    struct Cluster {
        std::vector<std::size_t> point_indices;
    };

    struct ClusterFeatures {

    }

    // Arranges cell key in the format of 64 bits cx (high 32 bits) cy (low 32 bits)
    std::int64_t cell_key(std::int32_t cx, std::int32_t cy) {
        return (static_cast<std::int64_t>(cx) << 32) | static_cast<std::uint32_t>(cy);
    }

    // Flattens a 2D grid index into a 1D array index
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

    /**
     * Runs Euclidean clustering on the non-ground points
     * Uses grids for much faster approach than O(n^2) brute force
     * 
     * @param in the input point cloud containing non-ground points
     */
    std::vector<Cluster> cluster(const foxglove::PointCloud& in) {
        std::vector<Cluster> clusters;
    
        const auto filtered_points = core::points(in);
        const std::size_t n = filtered_points.size();

        if (n == 0 || CLUSTER_TOLERANCE_M <= 0.0f) return clusters;

        const double tol_sq = CLUSTER_TOLERANCE_M * CLUSTER_TOLERANCE_M;
        const double inv_cell = 1.0 / CLUSTER_TOLERANCE_M;

        // Goes from point index -> cell coordinates
        std::vector<std::int32_t> cx(n), cy(n);
        std::unordered_map<std::int64_t, std::vector<int>> points_in_cell;
        points_in_cell.reserve(n);

        for (std::size_t i = 0; i < n; ++i) {
            cx[i] = static_cast<std::int32_t>(std::floor(filtered_points[i].x * inv_cell));
            cy[i] = static_cast<std::int32_t>(std::floor(filtered_points[i].y * inv_cell));
            const std::int64_t key = cell_key(cx[i], cy[i]);
            points_in_cell[key].push_back(static_cast<int>(i));
        }

        std::vector<std::uint8_t> visited(n, 0);
        std::size_t raw_clusters = 0, too_small = 0, too_large = 0; 
        std::queue<std::size_t> queue;

        for (std::size_t i = 0; i < n; ++i) {
            if (visited[i]) continue;
            Cluster cluster;
            ++raw_clusters;
            visited[i] = 1;
            queue.push(i);

            while (!queue.empty()) {
                const std::size_t cur = queue.front();
                queue.pop();
                cluster.point_indices.push_back(cur);

                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        auto it = points_in_cell.find(cell_key(cx[cur] + dx, cy[cur] + dy));
                        if (it == points_in_cell.end()) continue;
                        for (const int j : it->second) {
                            if (visited[j]) continue;
                            const double ddx = filtered_points[j].x - filtered_points[cur].x;
                            const double ddy = filtered_points[j].y - filtered_points[cur].y;
                            const double dist_sq = ddx * ddx + ddy * ddy;
                            if (dist_sq <= tol_sq) {
                                visited[j] = 1;
                                queue.push(j);
                            }
                        }
                    }
                }
            }

            const std::size_t cluster_size = cluster.point_indices.size();
            if (cluster_size < MIN_CLUSTER_SIZE) { ++too_small; continue; }
            if (cluster_size > MAX_CLUSTER_SIZE) { ++too_large; continue; }
            clusters.push_back(std::move(cluster));

        }

        std::cout << "Raw clusters: " << raw_clusters << ", Too small: " << too_small << ", Too large: " << too_large << ", Valid clusters: " << clusters.size() << std::endl;

        return clusters;
    }

    std::vector<ClusterFeatures> extract_features(const std::vector<Cluster>& clusters, const foxglove::PointCloud& points) {
        std::vector<ClusterFeatures> features;

        return features;
    }
    

} // END OF ANONYMOUS NAMESPACE

    std::shared_ptr<const foxglove::PointCloud> filter_cloud(const foxglove::PointCloud& scan) {
        std::shared_ptr<const foxglove::PointCloud> filtered_scan = remove_ground(scan);
        auto clusters = cluster(*filtered_scan);
        return filtered_scan;
    }

}