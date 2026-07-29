#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "frontend/SlamFrontend.hpp"

namespace slam::frontend
{
namespace
{

constexpr double kTolerance = 1e-9;

SlamFrontendParams make_test_params()
{
    SlamFrontendParams params;
    params.optimized_association_gate_m = 1.0;
    params.local_track_association_gate_m = 0.5;
    params.minimum_observations_to_confirm = 3U;
    params.tentative_track_max_age_ns = 1'000'000'000LL;
    params.pending_track_max_age_ns = 2'000'000'000LL;
    params.minimum_detection_confidence = 0.0;
    return params;
}

void expect_association(const AcceptedAssociation& actual,
                        const std::size_t expected_detection_index,
                        const std::size_t expected_target_index,
                        const double expected_residual_m)
{
    EXPECT_EQ(actual.valid_detection_index, expected_detection_index);
    EXPECT_EQ(actual.target_view_index, expected_target_index);
    EXPECT_NEAR(actual.residual_m, expected_residual_m, kTolerance);
}

TEST(SlamFrontendAssociationTest, EmptyInputsReturnNoAssociations)
{
    const SlamFrontend frontend(make_test_params());

    EXPECT_TRUE(frontend.associate_points_one_to_one(
                            {}, {transforms::Point2D{1.0, 0.0}}, 1.0)
                    .empty());

    EXPECT_TRUE(frontend.associate_points_one_to_one(
                            {transforms::Point2D{1.0, 0.0}}, {}, 1.0)
                    .empty());
}

TEST(SlamFrontendAssociationTest, GateBoundaryIsInclusive)
{
    const SlamFrontend frontend(make_test_params());

    const std::vector<AcceptedAssociation> associations =
        frontend.associate_points_one_to_one(
            {transforms::Point2D{0.0, 0.0}},
            {transforms::Point2D{0.3, 0.4}}, 0.5);

    ASSERT_EQ(associations.size(), 1U);
    expect_association(associations.front(), 0U, 0U, 0.5);
}

TEST(SlamFrontendAssociationTest, ClosestOneToOnePairsWin)
{
    const SlamFrontend frontend(make_test_params());

    const std::vector<AcceptedAssociation> associations =
        frontend.associate_points_one_to_one(
            {transforms::Point2D{0.0, 0.0}, transforms::Point2D{0.4, 0.0},
             transforms::Point2D{5.0, 0.0}},
            {transforms::Point2D{0.1, 0.0}, transforms::Point2D{5.2, 0.0}},
            1.0);

    ASSERT_EQ(associations.size(), 2U);
    expect_association(associations.at(0), 0U, 0U, 0.1);
    expect_association(associations.at(1), 2U, 1U, 0.2);
}

}  // namespace
}  // namespace slam::frontend
