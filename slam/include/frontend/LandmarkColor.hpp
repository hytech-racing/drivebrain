#pragma once

#include "common/SlamInterfaces.hpp"

namespace slam::frontend
{

struct ColorEvidence
{
    double blue{};
    double yellow{};
    double orange_small{};
    double orange_big{};
};

struct LandmarkColorEstimate
{
    ConeColor color{ConeColor::Unknown};
    double color_confidence{};
};

void update_color_evidence(ColorEvidence& evidence,
                           ConeColor observed_color,
                           double observed_confidence);

void merge_color_evidence(ColorEvidence& target, const ColorEvidence& source);

[[nodiscard]] LandmarkColorEstimate estimate_color(
    const ColorEvidence& evidence);

}  // namespace slam::frontend
