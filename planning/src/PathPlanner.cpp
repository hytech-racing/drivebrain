#include "PathPlanner.hpp"
#include "StateTracker.hpp"

namespace planning {

void PathPlanner::update(const dv_msgs::Cones& cones) {
    delaunay_.reset();
    coords_.clear();

    if (cones.cones_size() < 3) {
        return;
    }

    coords_.reserve(cones.cones_size() * 2);
    for (const auto& cone : cones.cones()) {
        coords_.push_back(cone.position().x());
        coords_.push_back(cone.position().y());
    }
    delaunay_.emplace(delaunator::Delaunator(coords_));
}

bool PathPlanner::isCrossingEdge(const dv_msgs::Cones& cones, const size_t edge_index) const {
    if (!delaunay_.has_value()) {
        return false;
    }
    const delaunator::Delaunator& delaunay = delaunay_.value();
    std::size_t curr_edge_vertex_A = delaunay.triangles[edge_index];
    std::size_t curr_edge_vertex_B = delaunay.triangles[delaunay.halfedges[edge_index]];

    auto vertex_A_color = cones.cones().at(curr_edge_vertex_A).color();
    auto vertex_B_color = cones.cones().at(curr_edge_vertex_B).color();
    
    bool is_crossing_edge = (vertex_A_color == dv_msgs::Cones_ConeColor_BLUE && vertex_B_color == dv_msgs::Cones_ConeColor_YELLOW) ||
        (vertex_A_color == dv_msgs::Cones_ConeColor_YELLOW && vertex_B_color == dv_msgs::Cones_ConeColor_BLUE);
    return is_crossing_edge;
}

core::xy_vec<float> PathPlanner::getEdgeMidpoint(size_t edge_index) const {
    if (!delaunay_.has_value()) {
        return {};
    }
    const delaunator::Delaunator& delaunay = delaunay_.value();
    std::size_t curr_edge_vertex_A = delaunay.triangles[edge_index];
    std::size_t curr_edge_vertex_B = delaunay.triangles[delaunay.halfedges[edge_index]];

    const float mx = (delaunay.coords[2*curr_edge_vertex_A] + delaunay.coords[2* curr_edge_vertex_B]) / (2.0);
    const float my = (delaunay.coords[2*curr_edge_vertex_A + 1] + delaunay.coords[2* curr_edge_vertex_B + 1]) / (2.0);
    return {mx, my};
}

inline core::xy_vec<double> PathPlanner::getVertexCoords(const size_t vertex_index) const {
    if (!delaunay_.has_value()) {
        return {};
    }
    return {delaunay_.value().coords[2*vertex_index], delaunay_.value().coords[2*vertex_index + 1]};
}

size_t PathPlanner::getEnclosingTriangleIndex(const hytech_msgs::pose& vehicle_pose) const {
    if (!delaunay_.has_value()) {
        return std::numeric_limits<size_t>::max();
    }
    const delaunator::Delaunator& delaunay = delaunay_.value();
    
    size_t enclosing_triangle_index = std::numeric_limits<size_t>::max();

    for (std::size_t i = 0; i < delaunay.triangles.size(); i += 3) {
      // select vertices
      const std::size_t p0 = delaunay.triangles[i];
      const std::size_t p1 = delaunay.triangles[i + 1];
      const std::size_t p2 = delaunay.triangles[i + 2];

      // fetch vertices' coordinates
      const VertexCoords2D v0 = getVertexCoords(p0);
      const VertexCoords2D v1 = getVertexCoords(p1);
      const VertexCoords2D v2 = getVertexCoords(p2);

      // vehicle pose
      const VertexCoords2D vehicle_pos{vehicle_pose.position().x(), vehicle_pose.position().y()};


      // sign of cross product of edges and vertex with current pos has to be the same for all three edges of the triangle
      const double cp0 = cross_product({v1.x - v0.x, v1.y - v0.y}, {vehicle_pos.x - v0.x, vehicle_pos.y - v0.y});
      const double cp1 = cross_product({v2.x - v1.x, v2.y - v1.y}, {vehicle_pos.x - v1.x, vehicle_pos.y - v1.y});
      const double cp2 = cross_product({v0.x - v2.x, v0.y - v2.y}, {vehicle_pos.x - v2.x, vehicle_pos.y - v2.y});

      // The vehicle is inside the triangle formed by p0, p1, and p2
      if (cp0 * cp1 * cp2 >= 0) {
        enclosing_triangle_index = i;
        break; 
      }      
    }
    return enclosing_triangle_index;
}

size_t PathPlanner::getFirstPathEdge(const hytech_msgs::pose& vehicle_pose, const std::vector<size_t>& enclosing_crosstrack_edges) const {
    if (!delaunay_.has_value() || enclosing_crosstrack_edges.empty()) {
        return std::numeric_limits<size_t>::max();
    }
    const delaunator::Delaunator& delaunay = delaunay_.value();

    if (enclosing_crosstrack_edges.size() == 1) {
        return enclosing_crosstrack_edges[0];
    }

    // get current heading as a unit vector
    const auto& q = vehicle_pose.orientation();
    const float yaw = std::atan2(
        2.0f * (q.w() * q.z() + q.x() * q.y()),
        1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z())
    );
    const float heading_x = std::cos(yaw);
    const float heading_y = std::sin(yaw);



   // get edge midpoints
    std::size_t edge_1_vertex_A = delaunay.triangles[enclosing_crosstrack_edges[0]];
    std::size_t edge_1_vertex_B = delaunay.triangles[delaunay.halfedges[enclosing_crosstrack_edges[0]]];
    core::xy_vec<float> edge_1_midpoint = getEdgeMidpoint(enclosing_crosstrack_edges[0]);

    std::size_t edge_2_vertex_A = delaunay.triangles[enclosing_crosstrack_edges[1]];
    std::size_t edge_2_vertex_B = delaunay.triangles[delaunay.halfedges[enclosing_crosstrack_edges[1]]];
    core::xy_vec<float> edge_2_midpoint = getEdgeMidpoint(enclosing_crosstrack_edges[1]);


    // get vector from midpoint 1 to 2
    float midpoints_vec_x = edge_2_midpoint.x - edge_1_midpoint.x;
    float midpoints_vec_y = edge_2_midpoint.y - edge_1_midpoint.y;

    // vector from vehicle pose to midpoint
    float vec_to_mid_x = edge_1_midpoint.x - vehicle_pose.position().x();
    float vec_to_mid_y = edge_1_midpoint.y - vehicle_pose.position().y();

    // dot product them
    float dot_product = midpoints_vec_x * heading_x + midpoints_vec_y * heading_y;
    // if dot product is positive, midpoint 2 is in front of midpoint 1, so we keep midpoint 2 and discard midpoint 1

    if (dot_product > 0) {
        return enclosing_crosstrack_edges[1];
    } 
    return enclosing_crosstrack_edges[0];
}
 
std::vector<core::xyz_vec<float>> PathPlanner::plan_path(const dv_msgs::Cones& cones, const hytech_msgs::pose& vehicle_pose) {
    update(cones); // Update the Delaunay triangulation with the new cone positions
    std::vector<core::xyz_vec<float>> path_points{};
    if (!delaunay_.has_value()) { // not enough cones to triangulate caused the update to fail
      return path_points; 
    }

    const delaunator::Delaunator& delaunay = delaunay_.value();
    static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
    // Select only cross-track edges (one blue and one yellow cone) and add their midpoint to the path_points vector
    for (std::size_t i = 0; i < delaunay.triangles.size(); i++) {
      
      // If the edge is a boundary (-1) or a twin edge already iterated over, skip it
      if (delaunay.halfedges[i] == invalid_index || delaunay.halfedges[i] > i) {
        continue;
      }

      if (isCrossingEdge(cones, i)) {
        core::xy_vec<float> midpoint = getEdgeMidpoint(i);
        path_points.push_back({midpoint.x, midpoint.y, 0.0f});
      }
    }


    size_t enclosing_triangle_index = getEnclosingTriangleIndex(vehicle_pose);
    
   

    // find which edges of the enclosing triangle are cross-track edges (one blue and one yellow cone)
    std::vector<size_t> enclosing_crosstrack_edges{};
    if (enclosing_triangle_index != static_cast<size_t>(-1)) {
      std::vector<size_t> enclosing_triangle_edges = {enclosing_triangle_index, enclosing_triangle_index + 1, enclosing_triangle_index + 2};
      for (auto edge: enclosing_crosstrack_edges) {
        if (isCrossingEdge(cones, edge)) {
          enclosing_crosstrack_edges.push_back(edge);
        }
      }
    }
    return path_points;

    /* Below is a hardcoded, randomized set of points that renders a small path in Foxglove.
      The logic for Delauany triangulation should live in here (and operate on @cones). The end result should be a vector of xyz_vec.
      (In all cases, z should always be zero, since this is 2D Delauany. it's not like the car is gonna fly or anything.)
    */
    // static thread_local std::mt19937 rng{std::random_device{}()};
    // std::uniform_real_distribution<float> lateral(-3.0f, 3.0f);


    // return {
    //   {0.0f, 0.0f, 0.0f},
    //   {3.0f, lateral(rng), 0.0f},
    //   {6.0f, lateral(rng), 0.0f},
    //   {9.0f, lateral(rng), 0.0f}
    // };
  }

} // namespace planning