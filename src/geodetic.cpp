#include "cyberwave/geometry/geodetic.hpp"

#include "cyberwave/geometry/quaternion.hpp"
#include "cyberwave/geometry/transform.hpp"
#include "cyberwave/geometry/vector.hpp"

#include <cmath>

namespace cyberwave
{
namespace geometry
{
namespace geo
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 1.57079632679489661923;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

Result<Geodetic> geodetic_failure(const char* detail)
{
    return Result<Geodetic>::failure(ErrorCode::kInvalidGeodetic, "", detail);
}

/// Every entry point that divides by a longitude scale routes through here, so
/// the pole guard is stated once rather than at each quotient.
Result<MetresPerDegree> invertible_scale(const Geodetic& origin)
{
    if (std::fabs(origin.latitude_deg) > kMaxTangentPlaneLatitude)
    {
        return Result<MetresPerDegree>::failure(ErrorCode::kInvalidGeodetic, "",
                                                "origin latitude is too close to a pole for a tangent plane");
    }
    return metres_per_degree(origin.latitude_deg);
}

} // namespace

bool is_finite(const Geodetic& g)
{
    return std::isfinite(g.latitude_deg) && std::isfinite(g.longitude_deg) && std::isfinite(g.altitude_m);
}

Result<Geodetic> validate(const Geodetic& g)
{
    if (!is_finite(g))
    {
        return geodetic_failure("geodetic position has a non-finite component");
    }
    if (g.latitude_deg < -90.0 || g.latitude_deg > 90.0)
    {
        return geodetic_failure("latitude is outside [-90, 90]");
    }
    if (g.longitude_deg < -180.0 || g.longitude_deg > 180.0)
    {
        return geodetic_failure("longitude is outside [-180, 180]");
    }
    return g;
}

Result<GeoAnchor> validate(const GeoAnchor& anchor)
{
    const Result<Geodetic> origin = validate(anchor.origin);
    if (!origin)
    {
        return Result<GeoAnchor>::failure(ErrorCode::kInvalidGeoAnchor, "", origin.error().detail);
    }
    if (std::fabs(anchor.origin.latitude_deg) > kMaxTangentPlaneLatitude)
    {
        return Result<GeoAnchor>::failure(ErrorCode::kInvalidGeoAnchor, "",
                                          "anchor latitude is too close to a pole for a tangent plane");
    }
    if (!std::isfinite(anchor.heading_deg))
    {
        return Result<GeoAnchor>::failure(ErrorCode::kInvalidGeoAnchor, "", "anchor heading is not finite");
    }
    return anchor;
}

Result<GeoPose> normalize(const GeoPose& pose)
{
    const Result<Geodetic> position = validate(pose.position);
    if (!position)
    {
        return Result<GeoPose>::failure(position.error().code, position.error().subject, position.error().detail);
    }
    const Result<Quaternion> orientation = quat::normalize(pose.orientation);
    if (!orientation)
    {
        return Result<GeoPose>::failure(orientation.error().code, orientation.error().subject,
                                        orientation.error().detail);
    }
    return GeoPose{position.value(), orientation.value()};
}

double normalize_longitude_deg(double longitude_deg)
{
    // Returned untouched when it is already in range rather than always run
    // through the modulo: adding 180 and taking it away again costs ~1e-13
    // degrees, which is 1e-8 m of position. Invisible on its own, but enough to
    // break the bit-for-bit agreement the cross-language golden vectors pin. The
    // wrap only has to be exact where it actually does something.
    if (longitude_deg >= -180.0 && longitude_deg < 180.0)
    {
        return longitude_deg;
    }
    if (!std::isfinite(longitude_deg))
    {
        return longitude_deg;
    }
    return std::fmod(std::fmod(longitude_deg + 180.0, 360.0) + 360.0, 360.0) - 180.0;
}

double normalize_bearing_deg(double bearing_deg)
{
    if (bearing_deg >= 0.0 && bearing_deg < 360.0)
    {
        return bearing_deg;
    }
    if (!std::isfinite(bearing_deg))
    {
        return bearing_deg;
    }
    return std::fmod(std::fmod(bearing_deg, 360.0) + 360.0, 360.0);
}

Result<MetresPerDegree> metres_per_degree(double latitude_deg)
{
    if (!std::isfinite(latitude_deg))
    {
        return Result<MetresPerDegree>::failure(ErrorCode::kNonFiniteValue, "", "latitude is not finite");
    }
    if (latitude_deg < -90.0 || latitude_deg > 90.0)
    {
        return Result<MetresPerDegree>::failure(ErrorCode::kInvalidGeodetic, "", "latitude is outside [-90, 90]");
    }

    const double phi = latitude_deg * kDegToRad;
    const double sin_phi = std::sin(phi);
    const double w = std::sqrt(1.0 - kWgs84EccentricitySquared * sin_phi * sin_phi);
    const double meridional = kWgs84SemiMajorAxis * (1.0 - kWgs84EccentricitySquared) / (w * w * w);
    const double prime_vertical = kWgs84SemiMajorAxis / w;
    return MetresPerDegree{kDegToRad * meridional, kDegToRad * prime_vertical * std::cos(phi)};
}

Result<Vector3> enu_between(const Geodetic& origin, const Geodetic& target)
{
    const Result<Geodetic> checked_origin = validate(origin);
    if (!checked_origin)
    {
        return Result<Vector3>::failure(checked_origin.error().code, "origin", checked_origin.error().detail);
    }
    const Result<Geodetic> checked_target = validate(target);
    if (!checked_target)
    {
        return Result<Vector3>::failure(checked_target.error().code, "target", checked_target.error().detail);
    }

    const Result<MetresPerDegree> scale = metres_per_degree(origin.latitude_deg);
    if (!scale)
    {
        return Result<Vector3>::failure(scale.error().code, "origin", scale.error().detail);
    }

    const double north = (target.latitude_deg - origin.latitude_deg) * scale.value().latitude;
    // The difference is wrapped, not just the result: see normalize_longitude_deg.
    const double east = normalize_longitude_deg(target.longitude_deg - origin.longitude_deg) * scale.value().longitude;
    return Vector3{east, north, target.altitude_m - origin.altitude_m};
}

Result<Geodetic> offset(const Geodetic& origin, const Vector3& enu)
{
    const Result<Geodetic> checked = validate(origin);
    if (!checked)
    {
        return checked;
    }
    if (!vec::is_finite(enu))
    {
        return geodetic_failure("ENU offset has a non-finite component");
    }

    const Result<MetresPerDegree> scale = invertible_scale(origin);
    if (!scale)
    {
        return Result<Geodetic>::failure(scale.error().code, "origin", scale.error().detail);
    }

    const double latitude = origin.latitude_deg + enu.y / scale.value().latitude;
    if (latitude < -90.0 || latitude > 90.0)
    {
        // Clamping would invent a position, and a fix that walks over a pole
        // means the offset or the origin is wrong, not that something travelled.
        return geodetic_failure("ENU offset carries the position past a pole");
    }
    return Geodetic{latitude, normalize_longitude_deg(origin.longitude_deg + enu.x / scale.value().longitude),
                    origin.altitude_m + enu.z};
}

Result<double> ground_distance(const Geodetic& a, const Geodetic& b)
{
    const Result<Vector3> enu = enu_between(a, b);
    if (!enu)
    {
        return Result<double>::failure(enu.error().code, enu.error().subject, enu.error().detail);
    }
    return std::hypot(enu.value().x, enu.value().y);
}

Result<Quaternion> enu_to_local_rotation(const GeoAnchor& anchor)
{
    const Result<GeoAnchor> checked = validate(anchor);
    if (!checked)
    {
        return Result<Quaternion>::failure(checked.error().code, checked.error().subject, checked.error().detail);
    }
    // A yaw about +Z, reached through the core rather than as a hand-rolled 2x2:
    // env +Y sits at bearing `heading_deg`, so east lands at that same angle
    // counter-clockwise from env +X.
    return quat::from_yaw(anchor.heading_deg * kDegToRad);
}

Result<Vector3> to_local(const GeoAnchor& anchor, const Geodetic& position)
{
    const Result<Quaternion> rotation = enu_to_local_rotation(anchor);
    if (!rotation)
    {
        return Result<Vector3>::failure(rotation.error().code, rotation.error().subject, rotation.error().detail);
    }
    const Result<Vector3> enu = enu_between(anchor.origin, position);
    if (!enu)
    {
        return enu;
    }
    return quat::rotate_unit(rotation.value(), enu.value());
}

Result<Geodetic> from_local(const GeoAnchor& anchor, const Vector3& position)
{
    const Result<Quaternion> rotation = enu_to_local_rotation(anchor);
    if (!rotation)
    {
        return Result<Geodetic>::failure(rotation.error().code, rotation.error().subject, rotation.error().detail);
    }
    if (!vec::is_finite(position))
    {
        return geodetic_failure("environment position has a non-finite component");
    }
    const Vector3 enu = quat::rotate_unit(quat::conjugate(rotation.value()), position);
    return offset(anchor.origin, enu);
}

Result<Transform> to_local_pose(const GeoAnchor& anchor, const GeoPose& pose)
{
    const Result<GeoPose> checked = normalize(pose);
    if (!checked)
    {
        return Result<Transform>::failure(checked.error().code, checked.error().subject, checked.error().detail);
    }
    const Result<Quaternion> rotation = enu_to_local_rotation(anchor);
    if (!rotation)
    {
        return Result<Transform>::failure(rotation.error().code, rotation.error().subject, rotation.error().detail);
    }
    const Result<Vector3> translation = to_local(anchor, checked.value().position);
    if (!translation)
    {
        return Result<Transform>::failure(translation.error().code, translation.error().subject,
                                          translation.error().detail);
    }
    // body -> ENU -> environment. Left-multiplying is what "then" means for a
    // Hamilton product; see CONVENTIONS.md section 1.
    return Transform{translation.value(), quat::multiply(rotation.value(), checked.value().orientation)};
}

Result<GeoPose> from_local_pose(const GeoAnchor& anchor, const Transform& pose)
{
    const Result<Transform> normalized = tf::normalize(pose);
    if (!normalized)
    {
        return Result<GeoPose>::failure(normalized.error().code, normalized.error().subject, normalized.error().detail);
    }
    const Result<Quaternion> rotation = enu_to_local_rotation(anchor);
    if (!rotation)
    {
        return Result<GeoPose>::failure(rotation.error().code, rotation.error().subject, rotation.error().detail);
    }
    const Result<Geodetic> position = from_local(anchor, normalized.value().translation);
    if (!position)
    {
        return Result<GeoPose>::failure(position.error().code, position.error().subject, position.error().detail);
    }
    return GeoPose{position.value(), quat::multiply(quat::conjugate(rotation.value()), normalized.value().rotation)};
}

Result<Quaternion> quat_from_compass_heading(double heading_deg)
{
    if (!std::isfinite(heading_deg))
    {
        return Result<Quaternion>::failure(ErrorCode::kNonFiniteValue, "", "compass heading is not finite");
    }
    return quat::from_yaw(kHalfPi - heading_deg * kDegToRad);
}

Result<double> to_compass_heading_deg(const Quaternion& enu_orientation)
{
    const Result<double> yaw = quat::to_yaw(enu_orientation);
    if (!yaw)
    {
        return yaw;
    }
    return normalize_bearing_deg(90.0 - yaw.value() * kRadToDeg);
}

Result<Quaternion> quat_from_ned_rpy(double roll, double pitch, double yaw)
{
    if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
    {
        return Result<Quaternion>::failure(ErrorCode::kNonFiniteValue, "", "NED attitude has a non-finite angle");
    }
    return quat::from_rpy(roll, -pitch, kHalfPi - yaw);
}

Result<Rpy> to_ned_rpy(const Quaternion& enu_orientation)
{
    const Result<Rpy> enu = quat::to_rpy(enu_orientation);
    if (!enu)
    {
        return enu;
    }
    // The yaw is wrapped back into (-pi, pi] so the inverse of a wrapped input
    // is the input, rather than drifting a turn per round trip.
    double yaw = kHalfPi - enu.value().yaw;
    yaw = std::fmod(yaw + kPi, 2.0 * kPi);
    if (yaw <= 0.0)
    {
        yaw += 2.0 * kPi;
    }
    return Rpy{enu.value().roll, -enu.value().pitch, yaw - kPi};
}

} // namespace geo
} // namespace geometry
} // namespace cyberwave
