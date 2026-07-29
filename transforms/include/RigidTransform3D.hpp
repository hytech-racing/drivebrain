#pragma once
#include <cmath>

namespace transforms
{

struct Quaternion
{
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] Quaternion multiply(const Quaternion& q) const noexcept
    {
        double rw = w * q.w - x * q.x - y * q.y - z * q.z;
        double rx = w * q.x + x * q.w + y * q.z - z * q.y;
        double ry = w * q.y - x * q.z + y * q.w + z * q.x;
        double rz = w * q.z + x * q.y - y * q.x + z * q.w;

        double len = std::sqrt(rw * rw + rx * rx + ry * ry + rz * rz);
        if (len > 1e-9)
        {
            rw /= len;
            rx /= len;
            ry /= len;
            rz /= len;
        }

        return Quaternion{rw, rx, ry, rz};
    }

    [[nodiscard]] Quaternion conjugate() const noexcept
    {
        return Quaternion{w, -x, -y, -z};
    }
};

struct Pose3D
{
    double x_m{};
    double y_m{};
    double z_m{};
    Quaternion q{};

    [[nodiscard]] Pose3D compose(const Pose3D& other) const noexcept
    {
        double num1 = q.x * 2.0;
        double num2 = q.y * 2.0;
        double num3 = q.z * 2.0;
        double num4 = q.x * num1;
        double num5 = q.y * num2;
        double num6 = q.z * num3;
        double num7 = q.x * num2;
        double num8 = q.x * num3;
        double num9 = q.y * num3;
        double num10 = q.w * num1;
        double num11 = q.w * num2;
        double num12 = q.w * num3;

        double rot_x = (1.0 - (num5 + num6)) * other.x_m +
                       (num7 - num12) * other.y_m + (num8 + num11) * other.z_m;
        double rot_y = (num7 + num12) * other.x_m +
                       (1.0 - (num4 + num6)) * other.y_m +
                       (num9 - num10) * other.z_m;
        double rot_z = (num8 - num11) * other.x_m + (num9 + num10) * other.y_m +
                       (1.0 - (num4 + num5)) * other.z_m;

        double new_x = x_m + rot_x;
        double new_y = y_m + rot_y;
        double new_z = z_m + rot_z;

        Quaternion new_q = q.multiply(other.q);

        return Pose3D{new_x, new_y, new_z, new_q};
    }

    [[nodiscard]] Pose3D inverse() const noexcept
    {
        Quaternion inv_q = q.conjugate();

        double num1 = inv_q.x * 2.0;
        double num2 = inv_q.y * 2.0;
        double num3 = inv_q.z * 2.0;
        double num4 = inv_q.x * num1;
        double num5 = inv_q.y * num2;
        double num6 = inv_q.z * num3;
        double num7 = inv_q.x * num2;
        double num8 = inv_q.x * num3;
        double num9 = inv_q.y * num3;
        double num10 = inv_q.w * num1;
        double num11 = inv_q.w * num2;
        double num12 = inv_q.w * num3;

        double inv_x = -((1.0 - (num5 + num6)) * x_m + (num7 - num12) * y_m +
                         (num8 + num11) * z_m);
        double inv_y = -((num7 + num12) * x_m + (1.0 - (num4 + num6)) * y_m +
                         (num9 - num10) * z_m);
        double inv_z = -((num8 - num11) * x_m + (num9 + num10) * y_m +
                         (1.0 - (num4 + num5)) * z_m);

        return Pose3D{inv_x, inv_y, inv_z, inv_q};
    }
};
}  // namespace transforms
