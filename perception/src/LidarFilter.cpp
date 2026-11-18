
#include "LidarFilter.hpp"
#include <limits>

namespace perception {
namespace {

    struct Cell {
        std::vector<std::size_t> point_indices;
    };

    struct Cluster {
        std::vector<std::size_t> point_indices;
    };

    struct GroundPoint {
        float range; // horizontal distance from the sensor
        float z;
    };

    struct GroundLine {
        float slope = 0.0f;
        float intercept = -SENSOR_HEIGHT_ABOVE_GROUND_M;
        bool valid = false; // defines whether the regression succeeded

        float height_at(float range) const {
            return slope*range + intercept;
        }
    };

    struct GroundSegment {
        GroundLine line;
        float start_range = 0.0f;
        float end_range = 0.0f;

        float height_at(float range) const {
            return line.height_at(range);
        }
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
        std::vector<std::uint8_t> valid;

        /* Cell index of a point, or -1 outside the grid */
        int cell(float x, float y) const {
            const float ri = (std::hypot(x, y) - GRID_MIN_RANGE_M) / GRID_RADIAL_BIN_SIZE_M;
            const float ti = (std::atan2(x, -y) - GRID_MIN_THETA_RAD) / GRID_ANGULAR_BIN_SIZE_RAD;
            if (!std::isfinite(ri) || !std::isfinite(ti)) return -1;
            if (ri < 0 || ri >= num_radial_bins || ti < 0 || ti >= num_angular_bins) return -1;
            return static_cast<int>(ri) * num_angular_bins + static_cast<int>(ti);
        }

        float at(float x, float y) const {
            const int i = cell(x, y);
            return i < 0 || !valid[i] ? -SENSOR_HEIGHT_ABOVE_GROUND_M : ground_z[i];
        }
    };

    // Arranges cell key in the format of 64 bits cx (high 32 bits) cy (low 32 bits)
    std::int64_t cell_key(std::int32_t cx, std::int32_t cy) {
        const std::uint64_t x = static_cast<std::uint32_t>(cx);
        const std::uint64_t y = static_cast<std::uint32_t>(cy);
        return static_cast<std::int64_t>((x << 32) | y);
    }

    /**
     * Performs least squares regression on the points that are entered.
     *
     * @param points The points of data that the line is being drawn between.
     * @return A groundline that represents the line of best fit for that group of points.
     */
    GroundLine fit_ground_line(const std::vector<GroundPoint>& points){
        GroundLine line;

        if(points.size() < MIN_GROUND_LINE_POINTS) {
            return line;
        }

        double sum_r = 0.0;
        double sum_z = 0.0;
        double sum_rr = 0.0;
        double sum_rz = 0.0;

        for(const auto& point: points) {
            sum_r += point.range;
            sum_z += point.z;
            sum_rr += static_cast<double>(point.range) * point.range;
            sum_rz += static_cast<double>(point.range) * point.z;
        }

        const double n = static_cast<double>(points.size());
        const double denominator = n * sum_rr - sum_r * sum_r;

        if(std::abs(denominator) < 1e-9) {
            return line;
        }

        const double slope = (n * sum_rz - sum_r * sum_z) / denominator;

        if(!std::isfinite(slope) || std::abs(slope) > MAX_GROUND_SLOPE) {
            return line;
        }

        line.slope = static_cast<float>(slope);
        line.intercept = static_cast<float>((sum_z - slope * sum_r) / n);
        line.valid = std::isfinite(line.intercept);

        return line;
    }

    /**
     * Checks whether a point is suitable for starting a ground segment.
     *
     * @param point The candidate ground point.
     * @return Whether the point is close to the expected initial ground height.
     */
    bool valid_ground_start(const GroundPoint& point) {
        return std::isfinite(point.range) && std::isfinite(point.z)
            && std::abs(point.z + SENSOR_HEIGHT_ABOVE_GROUND_M) <= MAX_GROUND_START_HEIGHT_ERROR_M;
    }

    /**
     * Creates an initial ground line using two candidate points.
     *
     * @param first The first ground point.
     * @param second The second ground point.
     * @return The initial line if the slope and spacing are acceptable.
     */
    GroundLine initial_ground_line(const GroundPoint& first, const GroundPoint& second) {
        GroundLine line;

        const float gap = second.range - first.range;

        if (gap <= 0.0f || gap > MAX_GROUND_SEGMENT_GAP_M) return line;

        const float slope = (second.z - first.z) / gap;

        if (!std::isfinite(slope) || std::abs(slope) > MAX_GROUND_SLOPE) return line;

        if (gap > GROUND_LONG_GAP_M &&
            std::abs(second.z - first.z) > MAX_GROUND_LONG_GAP_HEIGHT_CHANGE_M) return line;

        line.slope = slope;
        line.intercept = first.z - slope * first.range;
        line.valid = std::isfinite(line.intercept);

        return line;
    }

    /**
     * Checks whether all points agree with a fitted ground line.
     *
     * @param points The points belonging to the segment.
     * @param line The fitted line.
     * @return Whether the line satisfies the fitting error threshold.
     */
    bool valid_ground_fit(const std::vector<GroundPoint>& points, const GroundLine& line) {
        if (!line.valid) return false;

        for (const auto& point : points) {
            const float error = std::abs(point.z - line.height_at(point.range));
            if (error > MAX_GROUND_FIT_ERROR_M) return false;
        }

        return true;
    }

    /**
     * Performs least squares regression on ground points in different segments.
     *
     * @param points The lowest points from the radial bins in one angular sector.
     * @return The accepted ground line segments.
     */
    std::vector<GroundSegment> fit_ground_segments(const std::vector<GroundPoint>& points){
        std::vector<GroundSegment> segments;
        std::vector<GroundPoint> current;
        GroundLine current_line;

        auto save_segment = [&]() {
            if (current.size() < MIN_GROUND_LINE_POINTS) return;

            const GroundLine line = fit_ground_line(current);

            if (!valid_ground_fit(current, line)) return;

            const float start_range = current.front().range;
            const float start_height = line.height_at(start_range);

            if (segments.empty() &&
                std::abs(start_height + SENSOR_HEIGHT_ABOVE_GROUND_M) > MAX_GROUND_START_HEIGHT_ERROR_M) {
                return;
            }

            if (!segments.empty()) {
                const auto& previous = segments.back();
                const float gap = start_range - previous.end_range;

                if (gap <= MAX_GROUND_SEGMENT_GAP_M) {
                    const float expected = previous.height_at(start_range);
                    if (std::abs(start_height - expected) > MAX_GROUND_CONTINUITY_ERROR_M) return;
                }
            }

            segments.push_back({line,current.front().range,current.back().range});
        };

        for (const auto& point : points) {
            if (!std::isfinite(point.range) || !std::isfinite(point.z)) continue;

            if (current.empty()) {
                if (!segments.empty() || valid_ground_start(point)) current.push_back(point);
                continue;
            }

            const float gap = point.range - current.back().range;

            if (gap <= 0.0f) continue;

            if (gap > MAX_GROUND_SEGMENT_GAP_M) {
                save_segment();
                current.clear();
                current_line = GroundLine();

                if (!segments.empty() || valid_ground_start(point)) current.push_back(point);
                continue;
            }

            if (current.size() == 1) {
                const GroundLine line = initial_ground_line(current.front(), point);

                if (line.valid) {
                    current.push_back(point);
                    current_line = line;
                } else {
                    current.clear();
                    if (!segments.empty() || valid_ground_start(point)) current.push_back(point);
                }

                continue;
            }

            const float point_error = std::abs(point.z - current_line.height_at(point.range));

            if (point_error > MAX_GROUND_POINT_LINE_DISTANCE_M) {
                if (point.z > current_line.height_at(point.range)) {
                    continue;
                }

                save_segment();
                current.clear();
                current_line = GroundLine();

                if (!segments.empty() || valid_ground_start(point)) current.push_back(point);
                continue;
            }

            if (gap > GROUND_LONG_GAP_M &&
                std::abs(point.z - current.back().z) > MAX_GROUND_LONG_GAP_HEIGHT_CHANGE_M) {
                save_segment();
                current.clear();
                current_line = GroundLine();

                if (!segments.empty() || valid_ground_start(point)) current.push_back(point);
                continue;
            }

            std::vector<GroundPoint> trial = current;
            trial.push_back(point);

            const GroundLine trial_line = fit_ground_line(trial);

            if (!valid_ground_fit(trial, trial_line)) {
                save_segment();
                current.clear();
                current_line = GroundLine();

                if (!segments.empty() || valid_ground_start(point)) current.push_back(point);
                continue;
            }

            current.push_back(point);
            current_line = trial_line;
        }

        if (!current.empty()) {
            save_segment();
        }

        return segments;
    }

    /**
     * Finds a suitable ground segment in the current or neighboring angular sectors.
     *
     * @param segments Ground segments for every angular sector.
     * @param angular_bin The angular sector being searched.
     * @param range The radial distance of the point.
     * @return The closest supported ground segment, or nullptr if none exists.
     */
    const GroundSegment* find_ground_segment(
        const std::vector<std::vector<GroundSegment>>& segments,
        int angular_bin, float range) {

        const GroundSegment* nearest = nullptr;
        float nearest_distance = std::numeric_limits<float>::infinity();

        for (int offset = -GROUND_NEIGHBOR_SECTOR_SEARCH_BINS;
             offset <= GROUND_NEIGHBOR_SECTOR_SEARCH_BINS; ++offset) {

            const int a = angular_bin + offset;

            if (a < 0 || a >= static_cast<int>(segments.size())) continue;

            for (const auto& segment : segments[a]) {
                const float distance = std::max(segment.start_range - range,
                                                std::max(0.0f,range - segment.end_range));

                if (distance > MAX_GROUND_SEGMENT_GAP_M) continue;

                const float score = distance + std::abs(offset) * GRID_RADIAL_BIN_SIZE_M;

                if (score < nearest_distance) {
                    nearest_distance = score;
                    nearest = &segment;
                }
            }
        }

        return nearest;
    }

    /**
     * Removes ground points from the input point cloud.
     * Uses Himmelsbach-style piecewise linear ground segmentation.
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
        grid.valid.assign(cells.size(), 0);

        // Assign points to cells (grid angle measured around car-forward, os_sensor -y)
        for (std::size_t i = 0; i < scan.size(); ++i) {
            const int c = grid.cell(scan[i].x, scan[i].y);
            if (c >= 0) cells[c].point_indices.push_back(i);
        }

        std::vector<core::LidarPoint> non_ground;
        non_ground.reserve(scan.size());

        std::vector<std::vector<GroundSegment>> all_segments(grid.num_angular_bins);

        // Fit ground segments independently for every angular sector.
        for (int a = 0; a < grid.num_angular_bins; ++a) {
            std::vector<GroundPoint> lowest_points;

            // Select the lowest point in each radial bin.
            for (int r = 0; r < grid.num_radial_bins; ++r) {
                const int c = r * grid.num_angular_bins + a;
                const auto& indices = cells[c].point_indices;

                if (indices.size() < MIN_POINTS_PER_CELL) continue;

                const auto lowest = *std::min_element(
                    indices.begin(),
                    indices.end(),
                    [&](std::size_t i, std::size_t j) {return scan[i].z < scan[j].z;}
                );

                lowest_points.push_back({std::hypot(scan[lowest].x, scan[lowest].y),scan[lowest].z});
            }

            // Fit multiple ground segments.
            all_segments[a] = fit_ground_segments(lowest_points);
        }

        // Estimate ground height in each radial cell.
        for (int a = 0; a < grid.num_angular_bins; ++a) {
            for (int r = 0; r < grid.num_radial_bins; ++r) {
                const int c = r * grid.num_angular_bins + a;
                const float range = GRID_MIN_RANGE_M + (r + 0.5f) * GRID_RADIAL_BIN_SIZE_M;

                const GroundSegment* nearest = find_ground_segment(all_segments, a, range);

                if (nearest != nullptr) {
                    grid.ground_z[c] = nearest->height_at(range);
                    grid.valid[c] = 1;
                }
            }
        }

        // Classify points using the fitted ground segments.
        for (std::size_t c = 0; c < cells.size(); ++c) {
            const int a = static_cast<int>(c) % grid.num_angular_bins;

            for (const auto i : cells[c].point_indices) {
                const float range = std::hypot(scan[i].x, scan[i].y);
                const GroundSegment* nearest = find_ground_segment(all_segments, a, range);

                // Preserve points when no reliable ground model is available.
                if (nearest == nullptr) {
                    non_ground.push_back(scan[i]);
                    continue;
                }

                const float ground = nearest->height_at(range);

                if (scan[i].z > ground + NON_GROUND_HEIGHT_THRESHOLD_M) {
                    non_ground.push_back(scan[i]);
                }
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
        cones->set_timestamp_us(google::protobuf::util::TimeUtil::TimestampToMicroseconds(scan.timestamp()));
        return cones;
    }

}
