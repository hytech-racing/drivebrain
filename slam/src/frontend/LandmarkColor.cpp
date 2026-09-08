#include "frontend/LandmarkColor.hpp"

#include <algorithm>
#include <cmath>

namespace slam::frontend
{
void update_color_evidence(ColorEvidence& evidence,
                           const ConeColor observed_color,
                           const double observed_confidence)
{
    if (observed_color == ConeColor::Unknown ||
        !std::isfinite(observed_confidence) || observed_confidence <= 0.0)
    {
        return;
    }

    switch (observed_color)
    {
        case ConeColor::Unknown:
            return;
        case ConeColor::Blue:
            evidence.blue += observed_confidence;
            return;
        case ConeColor::Yellow:
            evidence.yellow += observed_confidence;
            return;
        case ConeColor::OrangeSmall:
            evidence.orange_small += observed_confidence;
            return;
        case ConeColor::OrangeBig:
            evidence.orange_big += observed_confidence;
            return;
    }
}

void merge_color_evidence(ColorEvidence& target, const ColorEvidence& source)
{
    target.blue += source.blue;
    target.yellow += source.yellow;
    target.orange_small += source.orange_small;
    target.orange_big += source.orange_big;
}

LandmarkColorEstimate estimate_color(const ColorEvidence& evidence)
{
    const double total = evidence.blue + evidence.yellow + evidence.orange_big +
                         evidence.orange_small;

    if (!std::isfinite(total) || total <= 0.0)
    {
        return LandmarkColorEstimate{};
    }

    ConeColor color = ConeColor::Blue;
    double best = evidence.blue;

    if (evidence.yellow > best)
    {
        color = ConeColor::Yellow;
        best = evidence.yellow;
    }

    if (evidence.orange_small > best)
    {
        color = ConeColor::OrangeSmall;
        best = evidence.orange_small;
    }

    if (evidence.orange_big > best)
    {
        color = ConeColor::OrangeBig;
        best = evidence.orange_big;
    }

    return LandmarkColorEstimate{color, best / total};
}
}  // namespace slam::frontend
