
#pragma once

#include <cstddef>
#include <vector>

#include "ClusterFeatures.hpp"
#include "PointCloudTypes.hpp"
namespace perception
{

struct ConeFilterParams
{
    double max_detection_range_m{25.0};

    double near_range_m{7.5};
    double mid_range_m{15.0};

    std::size_t near_min_cone_points{7};
    std::size_t mid_min_cone_points{5};
    std::size_t far_min_cone_points{3};

    double near_min_cone_height_m{0.06};
    double mid_min_cone_height_m{0.02};
    double far_min_cone_height_m{0.0};

    double near_max_cone_width_m{0.30};
    double mid_max_cone_width_m{0.25};
    double far_max_cone_width_m{0.10};

    double max_cone_height_m{0.5};

    double max_elongation_ratio{4.0};
    double min_width_for_elongation_m{0.03};

    double near_accepted_confidence{0.9};
    double mid_accepted_confidence{0.5};
    double far_accepted_confidence{0.25};
};

enum class ConeRejectionReason
{
    None,
    TooFar,
    TooFewPoints,
    TooShort,
    TooTall,
    TooWide,
    TooElongated
};

struct ConeCandidate
{
    PointXYZI position;
    double confidence{};
    ClusterFeatures feature;
};

struct RejectedCluster
{
    ClusterFeatures features;
    ConeRejectionReason reason{ConeRejectionReason::None};
};

struct ConeFilterResult
{
    std::vector<ConeCandidate> candidates;
    std::vector<RejectedCluster> rejected;
};

ConeFilterResult filter_cone_candidates(
    const std::vector<ClusterFeatures>& cluster_features,
    const ConeFilterParams& params);

}  // namespace perception
