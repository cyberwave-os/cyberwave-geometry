// Named value types shared by every Cyberwave language binding.
//
// Everything here is a plain aggregate: no invariants are enforced by the type
// system, because these cross a C ABI and land in Python dicts, TypeScript
// objects and protobuf messages unchanged. Validation lives in the free
// functions of quaternion.hpp / transform.hpp, which say so in their return
// type.
#ifndef CYBERWAVE_GEOMETRY_TYPES_HPP
#define CYBERWAVE_GEOMETRY_TYPES_HPP

#include <array>

namespace cyberwave
{
namespace geometry
{

/// A quaternion whose norm is below this carries no usable rotation. Treating
/// it as identity would persist a confidently wrong orientation, so the core
/// rejects it and leaves the identity fallback to compatibility adapters.
constexpr double kMinQuaternionNorm = 1e-9;

/// Below this an axis vector cannot be normalized, so the joint is unusable.
constexpr double kMinAxisNorm = 1e-12;

struct Vector3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    static Vector3 zero() { return Vector3{0.0, 0.0, 0.0}; }
};

/// Hamilton quaternion with *named* components.
///
/// The field order in memory is `x, y, z, w` -- the Cyberwave convention,
/// shared with protobuf, ROS and Three.js. `wxyz` (MuJoCo, the legacy backend
/// payloads) is a foreign order that only exists behind the named converters
/// in quaternion.hpp, never as a raw layout.
///
/// Even so, no caller should depend on the layout: the point of naming the
/// components is that both orders round-trip through explicit conversions
/// rather than through a positional array whose meaning you have to guess
/// from context. Brace-initialize with designated initializers.
struct Quaternion
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;

    static Quaternion identity() { return Quaternion{.x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0}; }
};

/// A rigid transform: translation applied after rotation (`p' = R*p + t`).
struct Transform
{
    Vector3 translation;
    Quaternion rotation;

    static Transform identity() { return Transform{Vector3::zero(), Quaternion::identity()}; }
};

/// Rotation about a unit axis by an angle in radians.
struct AxisAngle
{
    Vector3 axis;
    double angle = 0.0;
};

/// Roll/pitch/yaw in radians, fixed-axis XYZ. See CONVENTIONS.md.
struct Rpy
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

/// Row-major 3x3 rotation matrix: `m[row * 3 + col]`.
struct Matrix3
{
    std::array<double, 9> m{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};

    double& at(int row, int col) { return m[static_cast<std::size_t>(row * 3 + col)]; }
    double at(int row, int col) const { return m[static_cast<std::size_t>(row * 3 + col)]; }

    static Matrix3 identity() { return Matrix3{}; }
};

} // namespace geometry
} // namespace cyberwave

#endif // CYBERWAVE_GEOMETRY_TYPES_HPP
