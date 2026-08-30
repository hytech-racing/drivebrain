#pragma once

#include <optional>
#include <random>
#include <vector>
#include <algorithm>

#include <StateTracker.hpp>
#include "dv_msgs.pb.h"
#include <delaunator.hpp>

// left side = blue
// right side = yellow


namespace planning {

class PathPlanner {
  using VertexCoords2D = core::xy_vec<double>;

public:
  // TODO: come up with something for orange cones (exit and entry lanes)
  // Returns the next set of coordinates for the car to follow as a path
  std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones, const hytech_msgs::pose& vehicle_pose);
  void update(const dv_msgs::Cones& cones);

private:
  bool isCrossingEdge(const dv_msgs::Cones& cones, size_t edge_index) const;
  core::xy_vec<float> getEdgeMidpoint(size_t edge_index) const;
  core::xy_vec<double> getVertexCoords(const size_t vertex_index) const;
  size_t getEnclosingTriangleIndex(const hytech_msgs::pose& vehicle_pose) const;
  size_t getFirstPathEdge(const hytech_msgs::pose& vehicle_pose, const std::vector<size_t>& enclosing_crosstrack_edges) const;
  std::vector<core::xyz_vec<float>> order_path(const std::vector<core::xyz_vec<float>>& path_points, const hytech_msgs::pose& vehicle_pose) const;

  inline double cross_product(const core::xy_vec<double>& a, const core::xy_vec<double>& b) const {
    return a.x * b.y - a.y * b.x;
  }

  std::optional<delaunator::Delaunator> delaunay_;
  std::vector<double> coords_;

};

}