#include <gtest/gtest.h>

#include <ConeFilter.hpp>

#include <algorithm>
#include <vector>

namespace perception
{
namespace
{

ClusterFeatures make_features(const double range_m,
                              const std::size_t num_points,
                              const double width_x_m, const double width_y_m,
                              const double height_z_m)
{
    ClusterFeatures features;

    features.centroid.x = static_cast<float>(range_m);
    features.centroid.y = 0.0F;
    features.centroid.z = static_cast<float>(height_z_m * 0.5);

    features.bbox.min.x = static_cast<float>(range_m - width_x_m * 0.5);
    features.bbox.max.x = static_cast<float>(range_m + width_x_m * 0.5);

    features.bbox.min.y = static_cast<float>(-width_y_m * 0.5);
    features.bbox.max.y = static_cast<float>(width_y_m * 0.5);

    features.bbox.min.z = 0.0F;
    features.bbox.max.z = static_cast<float>(height_z_m);

    features.num_points = num_points;

    features.width_x_m = width_x_m;
    features.width_y_m = width_y_m;
    features.height_z_m = height_z_m;
    features.max_horizontal_width_m = std::max(width_x_m, width_y_m);
    features.range_m = range_m;

    return features;
}

ConeFilterParams make_test_params()
{
    ConeFilterParams params;

    params.max_detection_range_m = 10.0;

    params.near_range_m = 3.0;
    params.mid_range_m = 8.0;

    params.near_min_cone_points = 6;
    params.mid_min_cone_points = 3;
    params.far_min_cone_points = 3;

    params.near_min_cone_height_m = 0.06;
    params.mid_min_cone_height_m = 0.02;
    params.far_min_cone_height_m = 0.0;

    params.max_cone_height_m = 0.50;
    params.near_max_cone_width_m = 0.50;
    params.mid_max_cone_width_m = 0.50;
    params.far_max_cone_width_m = 0.50;

    params.max_elongation_ratio = 3.0;
    params.min_width_for_elongation_m = 0.03;

    params.near_accepted_confidence = 1.0;
    params.mid_accepted_confidence = 0.7;
    params.far_accepted_confidence = 0.4;

    return params;
}

TEST(ConeFilterTest, EmptyInputReturnsEmptyResult)
{
    const ConeFilterParams params = make_test_params();

    const std::vector<ClusterFeatures> features;

    const ConeFilterResult result = filter_cone_candidates(features, params);

    EXPECT_TRUE(result.candidates.empty());
    EXPECT_TRUE(result.rejected.empty());
}

TEST(ConeFilterTest, AcceptsCompactNearCone)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(2.0, 8, 0.20, 0.18, 0.20));

    const ConeFilterResult result = filter_cone_candidates(features, params);

    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_TRUE(result.rejected.empty());

    const ConeCandidate& candidate = result.candidates.at(0);

    EXPECT_NEAR(candidate.position.x, 2.0, 1e-6);
    EXPECT_NEAR(candidate.position.y, 0.0, 1e-6);
    EXPECT_NEAR(candidate.confidence, params.near_accepted_confidence, 1e-9);
    EXPECT_EQ(candidate.feature.num_points, 8U);
}

TEST(ConeFilterTest, RejectsClusterBeyondMaxDetectionRange)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(10.5, 8, 0.20, 0.20, 0.20));

    const ConeFilterResult result = filter_cone_candidates(features, params);

    EXPECT_TRUE(result.candidates.empty());
    ASSERT_EQ(result.rejected.size(), 1U);
    EXPECT_EQ(result.rejected.at(0).reason, ConeRejectionReason::TooFar);
}

TEST(ConeFilterTest, RejectsNearSparseCluster)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(2.0, 5, 0.20, 0.20, 0.20));

    const ConeFilterResult result = filter_cone_candidates(features, params);

    EXPECT_TRUE(result.candidates.empty());
    ASSERT_EQ(result.rejected.size(), 1U);
    EXPECT_EQ(result.rejected.at(0).reason,
              ConeRejectionReason::TooFewPoints);
}

TEST(ConeFilterTest, AcceptsMidRangeSparseCone)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(6.0, 3, 0.18, 0.16, 0.03));

    const ConeFilterResult result = filter_cone_candidates(features, params);

    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_TRUE(result.rejected.empty());

    EXPECT_NEAR(result.candidates.at(0).confidence,
                params.mid_accepted_confidence, 1e-9);
}

TEST(ConeFilterTest, AcceptsFarWeakHeightCone)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(9.0, 3, 0.18, 0.16, 0.005));

    const ConeFilterResult result = filter_cone_candidates(features, params);

    ASSERT_EQ(result.candidates.size(), 1U);
    EXPECT_TRUE(result.rejected.empty());

    EXPECT_NEAR(result.candidates.at(0).confidence,
                params.far_accepted_confidence, 1e-9);
}

TEST(ConeFilterTest, RejectsTooShortNearCluster)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(2.0, 8, 0.20, 0.18, 0.03));

    const ConeFilterResult result = filter_cone_candidates(features, params);
    EXPECT_TRUE(result.candidates.empty());
    ASSERT_EQ(result.rejected.size(), 1U);
    EXPECT_EQ(result.rejected.at(0).reason, ConeRejectionReason::TooShort);
}

TEST(ConeFilterTest, RejectsTooTallCluster)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(2.0, 8, 0.20, 0.20, 0.80));

    const ConeFilterResult result = filter_cone_candidates(features, params);
    EXPECT_TRUE(result.candidates.empty());
    ASSERT_EQ(result.rejected.size(), 1U);
    EXPECT_EQ(result.rejected.at(0).reason, ConeRejectionReason::TooTall);
}

TEST(ConeFilterTest, RejectsTooWideCluster)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(4.0, 8, 0.80, 0.20, 0.20));

    const ConeFilterResult result = filter_cone_candidates(features, params);
    EXPECT_TRUE(result.candidates.empty());
    ASSERT_EQ(result.rejected.size(), 1U);
    EXPECT_EQ(result.rejected.at(0).reason, ConeRejectionReason::TooWide);
}

TEST(ConeFilterTest, RejectsElongatedCluster)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(4.0, 8, 0.45, 0.05, 0.20));

    const ConeFilterResult result = filter_cone_candidates(features, params);
    EXPECT_TRUE(result.candidates.empty());
    ASSERT_EQ(result.rejected.size(), 1U);
    EXPECT_EQ(result.rejected.at(0).reason,
              ConeRejectionReason::TooElongated);
}

TEST(ConeFilterTest, PreservesRejectedClusterFeatures)
{
    const ConeFilterParams params = make_test_params();

    const ClusterFeatures rejected_features =
        make_features(4.0, 8, 0.80, 0.20, 0.20);

    std::vector<ClusterFeatures> features;
    features.push_back(rejected_features);

    const ConeFilterResult result = filter_cone_candidates(features, params);
    ASSERT_EQ(result.rejected.size(), 1U);

    EXPECT_EQ(result.rejected.at(0).reason, ConeRejectionReason::TooWide);
    EXPECT_EQ(result.rejected.at(0).features.num_points,
              rejected_features.num_points);
    EXPECT_NEAR(result.rejected.at(0).features.range_m,
                rejected_features.range_m, 1e-9);
    EXPECT_NEAR(result.rejected.at(0).features.max_horizontal_width_m,
                rejected_features.max_horizontal_width_m, 1e-9);
}

TEST(ConeFilterTest, HandlesMixedAcceptedAndRejectedClusters)
{
    const ConeFilterParams params = make_test_params();

    std::vector<ClusterFeatures> features;
    features.push_back(make_features(2.0, 8, 0.20, 0.18, 0.20));
    features.push_back(make_features(4.0, 8, 0.80, 0.20, 0.20));
    features.push_back(make_features(6.0, 3, 0.18, 0.16, 0.03));
    features.push_back(make_features(11.0, 8, 0.20, 0.18, 0.20));

    const ConeFilterResult result = filter_cone_candidates(features, params);
    ASSERT_EQ(result.candidates.size(), 2U);
    ASSERT_EQ(result.rejected.size(), 2U);

    EXPECT_EQ(result.rejected.at(0).reason, ConeRejectionReason::TooWide);
    EXPECT_EQ(result.rejected.at(1).reason, ConeRejectionReason::TooFar);
}

}  // namespace
}  // namespace perception
