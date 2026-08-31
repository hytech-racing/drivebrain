#include <PurePursuitMath.cpp>

namespace control {
namespace driverless {

core::xy_vec<float> PurePursuitMath::getVehicleHeading(core::VehicleState vehicle_state) {
    const auto& q = vehicle_pose.orientation();
    const float yaw = std::atan2(
        2.0f * (q.w() * q.z() + q.x() * q.y()),
        1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z())
    );
    const float heading_x = std::cos(yaw);
    const float heading_y = std::sin(yaw);
    return core::xy_vec<float>(heading_x, heading_y);
}


}
}