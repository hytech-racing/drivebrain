#ifndef __LITERALS_H__
#define __LITERALS_H__

using speed_m_s = float;
using torque_nm = float;
using velocity_rpm = float;

namespace constants
{
    constexpr long double GEARBOX_RATIO               = 11.86;
    constexpr long double WHEEL_DIAMETER              = 0.4064; // meters
    constexpr long double RPM_TO_METERS_PER_SECOND    = WHEEL_DIAMETER * 3.1415 / GEARBOX_RATIO / 60.0;
    constexpr long double RPM_TO_KILOMETERS_PER_HOUR  = RPM_TO_METERS_PER_SECOND * 3600.0 / 1000.0;
    constexpr long double METERS_PER_SECOND_TO_RPM    = 1.0 / RPM_TO_METERS_PER_SECOND;
    constexpr long double POWER_LIMIT                 = 63000.0; // in watts of mechanical power

    constexpr long double RPM_TO_RAD_PER_SECOND = 2.0 * 3.1415 / 60.0;
    constexpr long double RAD_PER_SECOND_TO_RPM = 1.0 / RPM_TO_RAD_PER_SECOND;
}

class VehicleSpeed
{
    public: 
        explicit constexpr VehicleSpeed(const long double s) : _meters_per_second(s) {}

        /// @brief get the speed in meters per second
        /// @return float value
        speed_m_s get() { return _meters_per_second; };
    private:
        const speed_m_s _meters_per_second;
};

constexpr VehicleSpeed operator"" _m_s(long double m_s)
{
    return VehicleSpeed{m_s};
}

constexpr VehicleSpeed operator"" _kph(long double kph)
{
    return VehicleSpeed{kph * 0.277778};
}

constexpr VehicleSpeed operator"" _mph(long double mph)
{
    return VehicleSpeed{mph * 0.44704};
}

/// @brief literal to convert vehicle's wheel rpm to @ref VehicleSpeed
/// @param w_rpm wheel rpm
/// @return @ref VehicleSpeed
constexpr VehicleSpeed operator"" _w_rpm(long double w_rpm)
{
    return VehicleSpeed{w_rpm * constants::RPM_TO_METERS_PER_SECOND};
}

constexpr VehicleSpeed operator"" _w_rad_s(long double w_rad_s)
{
    return VehicleSpeed{w_rad_s * constants::RAD_PER_SECOND_TO_RPM * constants::RPM_TO_METERS_PER_SECOND};
}

class AngularSpeed
{
    public:
        explicit constexpr AngularSpeed(const long double rpm) : _rpm(rpm) {}
        velocity_rpm get() {return _rpm; };
    private:
        velocity_rpm _rpm;
};

constexpr AngularSpeed operator"" _rpm(long double rpm)
{
    return AngularSpeed{rpm};
}

constexpr AngularSpeed operator"" _rad_s(long double rad_s)
{
    return AngularSpeed{rad_s * constants::RAD_PER_SECOND_TO_RPM};
}


#endif // __LITERALS_H__