#pragma once

#include <algorithm>
#include <cmath>

namespace control {
class HeadingHold {
public:
    struct Config {
        float kp = 2.0f; // column degrees / heading degree
        float kd = 0.5f; // column degrees / (yaw degree/s)
        float max_column_angle_deg = 15.0f;
        float steering_sign = 1.0f;
    };
    static bool valid(const Config& c) {
        return std::isfinite(c.kp) && c.kp >= 0 && std::isfinite(c.kd) && c.kd >= 0 &&
            std::isfinite(c.max_column_angle_deg) && c.max_column_angle_deg > 0 &&
            c.max_column_angle_deg <= 90 && std::abs(c.steering_sign) == 1.0f;
    }
    void reset(double heading_deg) { _reference_deg = heading_deg; }
    float step(const Config& c, double heading_deg, float yaw_rate_deg_s) const {
        const float error = static_cast<float>(std::remainder(heading_deg - _reference_deg, 360.0));
        return std::clamp(-c.steering_sign * (c.kp * error + c.kd * yaw_rate_deg_s),
                          -c.max_column_angle_deg, c.max_column_angle_deg);
    }
private:
    double _reference_deg = 0.0;
};
}
