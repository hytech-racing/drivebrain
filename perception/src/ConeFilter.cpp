
#include "ConeFilter.hpp"

#include <algorithm>

namespace perception
{

std::size_t choose_min_points_by_range(const double range,
                                       const ConeFilterParams& params);
double choose_min_height_by_range(const double range,
                                  const ConeFilterParams& params);
double choose_max_width_by_range(const double range,
                                 const ConeFilterParams& params);
double choose_confidence_by_range(const double range,
                                  const ConeFilterParams& params);

double compute_elongation(const ClusterFeatures& feature,
                          const ConeFilterParams& params);

ConeFilterResult filter_cone_candidates(
    const std::vector<ClusterFeatures>& cluster_features,
    const ConeFilterParams& params)
{
    ConeFilterResult result;

    for (const ClusterFeatures& feature : cluster_features)
    {
        if (feature.range_m > params.max_detection_range_m)
        {
            result.rejected.emplace_back(
                RejectedCluster{feature, ConeRejectionReason::TooFar});
            continue;
        }

        const std::size_t min_points =
            choose_min_points_by_range(feature.range_m, params);
        const double min_height =
            choose_min_height_by_range(feature.range_m, params);
        const double max_width =
            choose_max_width_by_range(feature.range_m, params);

        if (feature.num_points < min_points)
        {
            result.rejected.emplace_back(
                RejectedCluster{feature, ConeRejectionReason::TooFewPoints});
            continue;
        }

        if (feature.height_z_m < min_height)
        {
            result.rejected.emplace_back(
                RejectedCluster{feature, ConeRejectionReason::TooShort});
            continue;
        }

        if (feature.height_z_m > params.max_cone_height_m)
        {
            result.rejected.emplace_back(
                RejectedCluster{feature, ConeRejectionReason::TooTall});
            continue;
        }

        if (feature.max_horizontal_width_m > max_width)
        {
            result.rejected.emplace_back(
                RejectedCluster{feature, ConeRejectionReason::TooWide});
            continue;
        }

        const double elongation = compute_elongation(feature, params);

        if (elongation > params.max_elongation_ratio)
        {
            result.rejected.emplace_back(
                RejectedCluster{feature, ConeRejectionReason::TooElongated});
            continue;
        }

        const double confidence =
            choose_confidence_by_range(feature.range_m, params);

        ConeCandidate candidate{feature.centroid, confidence, feature};
        result.candidates.push_back(candidate);
    }

    return result;
}

std::size_t choose_min_points_by_range(const double range,
                                       const ConeFilterParams& params)
{
    if (range <= params.near_range_m)
    {
        return params.near_min_cone_points;
    }

    if (range <= params.mid_range_m)
    {
        return params.mid_min_cone_points;
    }

    return params.far_min_cone_points;
}

double choose_min_height_by_range(const double range,
                                  const ConeFilterParams& params)
{
    if (range <= params.near_range_m)
    {
        return params.near_min_cone_height_m;
    }

    if (range <= params.mid_range_m)
    {
        return params.mid_min_cone_height_m;
    }

    return params.far_min_cone_height_m;
}

double choose_max_width_by_range(const double range,
                                 const ConeFilterParams& params)
{
    if (range <= params.near_range_m)
    {
        return params.near_max_cone_width_m;
    }

    if (range <= params.mid_range_m)
    {
        return params.mid_max_cone_width_m;
    }

    return params.far_max_cone_width_m;
}

double choose_confidence_by_range(const double range,
                                  const ConeFilterParams& params)
{
    if (range <= params.near_range_m)
    {
        return params.near_accepted_confidence;
    }

    if (range <= params.mid_range_m)
    {
        return params.mid_accepted_confidence;
    }

    return params.far_accepted_confidence;
}

double compute_elongation(const ClusterFeatures& feature,
                          const ConeFilterParams& params)
{
    const double small_width =
        std::max(std::min(feature.width_x_m, feature.width_y_m),
                 params.min_width_for_elongation_m);

    const double large_width = std::max(feature.width_x_m, feature.width_y_m);

    return large_width / small_width;
}

}  // namespace perception
