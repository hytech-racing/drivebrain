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
        core::LidarPoint centroid{0, 0, 0}, min{0, 0, 0}, max{0, 0, 0};
        float width_x_m = 0, width_y_m = 0, height_z_m = 0; 
        float top_height_m = 0;  // cluster top above the local ground estimate
        float max_horizontal_width_m = 0, range_m = 0; 
        std::size_t num_points = 0;
    };

    /* Estimated ground height per polar cell, from remove_ground */
    struct GroundGrid {
        int num_radial_bins = 0, num_angular_bins = 0;
        std::vector<float> ground_z;

        /* Cell index of a point, or -1 outside the grid */
        int cell(float x, float y) const {
            const float ri = (std::hypot(x, y) - GRID_MIN_RANGE_M) / GRID_RADIAL_BIN_SIZE_M;
            const float ti = (std::atan2(x, -y) - GRID_MIN_THETA_RAD) / GRID_ANGULAR_BIN_SIZE_RAD;
            if (ri < 0 || ri >= num_radial_bins || ti < 0 || ti >= num_angular_bins) return -1;
            return static_cast<int>(ri) * num_angular_bins + static_cast<int>(ti);
        }

        float at(float x, float y) const {
            const int i = cell(x, y);
            return i < 0 ? -SENSOR_HEIGHT_ABOVE_GROUND_M : ground_z[i];
        }
    };

    // Arranges cell key in the format of 64 bits cx (high 32 bits) cy (low 32 bits)
    std::int64_t cell_key(std::int32_t cx, std::int32_t cy) {
        return (static_cast<std::int64_t>(cx) << 32) | static_cast<std::uint32_t>(cy);
    }

    /**
     * Removes ground points from the input point cloud.
     * Points are classified as ground or non-ground based on a grid-based height estimation.
     * 
     * @param in The input point cloud.
     * @param grid Receives the ground height estimated for each cell.
     * @return A new point cloud containing only non-ground points.
     */
    std::shared_ptr<const foxglove::PointCloud> remove_ground(const foxglove::PointCloud& in, GroundGrid& grid) {
        const auto scan = core::points(in);

        // Determine the grid size
        grid.num_angular_bins = std::ceil((GRID_MAX_THETA_RAD - GRID_MIN_THETA_RAD) / GRID_ANGULAR_BIN_SIZE_RAD);
        grid.num_radial_bins = std::ceil((GRID_MAX_RANGE_M - GRID_MIN_RANGE_M) / GRID_RADIAL_BIN_SIZE_M);
        std::vector<Cell> cells(grid.num_radial_bins * grid.num_angular_bins);
        grid.ground_z.assign(cells.size(), -SENSOR_HEIGHT_ABOVE_GROUND_M);

        // Assign points to cells (grid angle measured around car-forward, os_sensor -y)
        for (std::size_t i = 0; i < scan.size(); ++i) {
            const int c = grid.cell(scan[i].x, scan[i].y);
            if (c >= 0) cells[c].point_indices.push_back(i);
        }

        // Estimate the ground height for each cell by looking at the nth pensicile and classify points as ground or non-ground
        std::vector<float> z;
        std::vector<core::LidarPoint> non_ground;
        non_ground.reserve(scan.size());

        for (std::size_t c = 0; c < cells.size(); ++c) {
            const auto& cell = cells[c];
            z.clear();
            for (auto i : cell.point_indices) z.push_back(scan[i].z);

            // Get nth percentile of the z-values to estimate the ground height
            if (!z.empty() && z.size() >= MIN_POINTS_PER_CELL) {
                const auto nth = static_cast<std::ptrdiff_t>(GROUND_PERCENTILE * (z.size()-1));
                std::nth_element(z.begin(), z.begin() + nth, z.end());
                grid.ground_z[c] = z[nth];
            }
            const float ground = grid.ground_z[c];

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

        // std::cout << "Raw clusters: " << raw_clusters << ", Too small: " << too_small << ", Too large: " << too_large << ", Valid clusters: " << clusters.size() << std::endl;

        return clusters;
    }

    /**
     * Extract features from the given clusters and the filtered point cloud.
     * Returns a vector of ClusterFeatures containing centroid, bounding box, and other metrics for each cluster.
     * 
     * @param clusters The vector of clusters to extract features from.
     * @param filtered_cloud The filtered point cloud corresponding to the clusters.
     * @param grid The ground height estimates, for heights relative to the local ground.
     * @return A vector of ClusterFeatures for each cluster.
     */
    std::vector<ClusterFeatures> extract_features(const std::vector<Cluster>& clusters, const foxglove::PointCloud& filtered_cloud, const GroundGrid& grid) {
        const auto pts = core::points(filtered_cloud); 
        std::vector<ClusterFeatures> features;
        features.reserve(clusters.size());

        for (const auto& cluster : clusters) {
            if (cluster.point_indices.empty()) continue;

            ClusterFeatures feature;
            const auto& first = pts[cluster.point_indices.front()];
            feature.min = feature.max = first;
            
            // Get centroid and bounding box for the cluster
            for (const auto& idx : cluster.point_indices) {
                const auto& pt = pts[idx];
                feature.centroid.x += pt.x;  feature.centroid.y += pt.y;  feature.centroid.z += pt.z;
                feature.min.x = std::min(feature.min.x, pt.x);  feature.max.x = std::max(feature.max.x, pt.x);
                feature.min.y = std::min(feature.min.y, pt.y);  feature.max.y = std::max(feature.max.y, pt.y);
                feature.min.z = std::min(feature.min.z, pt.z);  feature.max.z = std::max(feature.max.z, pt.z);
            }

            feature.num_points = cluster.point_indices.size();
            const float n = static_cast<float>(feature.num_points);
            feature.centroid = {feature.centroid.x / n, feature.centroid.y / n, feature.centroid.z / n};

            feature.width_x_m = feature.max.x - feature.min.x;
            feature.width_y_m = feature.max.y - feature.min.y;
            feature.height_z_m = feature.max.z - feature.min.z;
            feature.top_height_m = feature.max.z - grid.at(feature.centroid.x, feature.centroid.y);
            feature.max_horizontal_width_m = std::max(feature.width_x_m, feature.width_y_m);
            feature.range_m = std::hypot(feature.centroid.x, feature.centroid.y);

            features.push_back(std::move(feature));
        }

        return features;
    }

    std::shared_ptr<dv_msgs::Cones> extract_cones(const std::vector<ClusterFeatures>& features, const foxglove::PointCloud& filtered_cloud) {
        auto cones = std::make_shared<dv_msgs::Cones>();

        for (const ClusterFeatures& feature : features) {
            // Determine the range category of the cluster (near, mid, far) and create lambda for returning values based on range
            const bool near = feature.range_m < NEAR_RANGE_M;
            const bool mid = !near && feature.range_m <= MID_RANGE_M;
            const bool far = !near && !mid;
            const auto by_range = [&](auto n, auto m, auto f) { return near ? n : mid ? m : f; };

            const std::size_t min_points = by_range(NEAR_MIN_CONE_POINTS, MID_MIN_CONE_POINTS, FAR_MIN_CONE_POINTS);
            const double min_height = by_range(NEAR_MIN_CONE_HEIGHT_M, MID_MIN_CONE_HEIGHT_M, FAR_MIN_CONE_HEIGHT_M);
            const double max_width = by_range(NEAR_MAX_CONE_WIDTH_M, MID_MAX_CONE_WIDTH_M, FAR_MAX_CONE_WIDTH_M);
            const double small_width = std::max(std::min(feature.width_x_m, feature.width_y_m), MIN_WIDTH_FOR_ELONGATION_M);
            const double elongation = feature.max_horizontal_width_m / small_width;

            // TODO better debug information for why a cluster is being skipped 
            if (feature.range_m > MAX_DETECTION_RANGE_M) continue;
            else if (feature.num_points < min_points) continue;
            else if (feature.height_z_m < min_height) continue;
            else if (feature.height_z_m > MAX_CONE_HEIGHT_M) continue;
            else if (feature.max_horizontal_width_m > max_width) continue;
            else if (elongation > MAX_ELONGATION) continue;
            else if (feature.top_height_m > MAX_CONE_TOP_HEIGHT_M) continue;

            // Add the dv cone
            auto* cone = cones->add_cones();

            cone->set_color(dv_msgs::Cones::UNKNOWN);

            auto* position = cone->mutable_position();
            position->set_x(feature.centroid.x);
            position->set_y(feature.centroid.y);
            position->set_z(feature.centroid.z);
        }
        
        return cones;
    }
    

} // END OF ANONYMOUS NAMESPACE

    std::shared_ptr<dv_msgs::Cones> filter_cloud(const foxglove::PointCloud& scan) {
        GroundGrid ground;
        std::shared_ptr<const foxglove::PointCloud> filtered_scan = remove_ground(scan, ground);
        auto clusters = cluster(*filtered_scan);
        auto features = extract_features(clusters, *filtered_scan, ground);
        auto cones = extract_cones(features, *filtered_scan);
        const auto& ts = scan.timestamp();
        uint64_t microseconds = (static_cast<uint64_t>(ts.seconds()) * 1'000'000ULL) 
                          + (static_cast<uint64_t>(ts.nanos()) / 1'000ULL);
        cones->set_timestamp_us(microseconds);
        return cones;
    }

}