#include <gtest/gtest.h>

#include <cmath>

#include "frontend/LandmarkColor.hpp"

namespace slam::frontend
{
namespace
{

TEST(LandmarkColorTest, UnknownAndInvalidObservationsAreIgnored)
{
    ColorEvidence evidence;

    update_color_evidence(evidence, ConeColor::Unknown, 1.0);
    update_color_evidence(evidence, ConeColor::Blue, 0.0);
    update_color_evidence(evidence, ConeColor::Yellow, -0.1);
    update_color_evidence(evidence, ConeColor::OrangeSmall, NAN);

    const LandmarkColorEstimate estimate = estimate_color(evidence);
    EXPECT_EQ(estimate.color, ConeColor::Unknown);
    EXPECT_DOUBLE_EQ(estimate.color_confidence, 0.0);
}

TEST(LandmarkColorTest, AccumulatesEvidenceByColor)
{
    ColorEvidence evidence;

    update_color_evidence(evidence, ConeColor::Blue, 0.8);
    update_color_evidence(evidence, ConeColor::Yellow, 0.4);
    update_color_evidence(evidence, ConeColor::Blue, 0.9);

    const LandmarkColorEstimate estimate = estimate_color(evidence);
    EXPECT_EQ(estimate.color, ConeColor::Blue);
    EXPECT_NEAR(estimate.color_confidence, 1.7 / 2.1, 1e-9);
}

TEST(LandmarkColorTest, MergesEvidence)
{
    ColorEvidence target;
    target.blue = 0.4;

    ColorEvidence source;
    source.blue = 0.6;
    source.orange_big = 1.0;

    merge_color_evidence(target, source);

    EXPECT_DOUBLE_EQ(target.blue, 1.0);
    EXPECT_DOUBLE_EQ(target.orange_big, 1.0);
}

}  // namespace
}  // namespace slam::frontend
