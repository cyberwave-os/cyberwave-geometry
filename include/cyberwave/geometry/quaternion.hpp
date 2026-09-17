// Quaternion algebra and every conversion the monorepo used to reimplement.
//
// Hamilton convention throughout: `multiply(a, b)` is the rotation that applies
// `b` first and then `a`, matching ROS, Eigen and Three.js (and *not* the JPL
// convention some IMU vendors ship).
#ifndef CYBERWAVE_GEOMETRY_QUATERNION_HPP
#define CYBERWAVE_GEOMETRY_QUATERNION_HPP

#include "cyberwave/geometry/errors.hpp"
#include "cyberwave/geometry/types.hpp"

#include <array>

namespace cyberwave
{
namespace geometry
{
namespace quat
{

// --- explicit component-order conversions -----------------------------------
//
// These exist so that no call site anywhere in the monorepo has to encode a
// component order positionally. `xyzw` is what protobuf, ROS and Three.js use;
// `wxyz` is what MuJoCo and the legacy backend payloads use.

Quaternion from_wxyz(double w, double x, double y, double z);
Quaternion from_xyzw(double x, double y, double z, double w);
Quaternion from_wxyz(const std::array<double, 4>& components);
Quaternion from_xyzw(const std::array<double, 4>& components);
std::array<double, 4> to_wxyz(const Quaternion& q);
std::array<double, 4> to_xyzw(const Quaternion& q);

// --- algebra ----------------------------------------------------------------

double squared_norm(const Quaternion& q);
double norm(const Quaternion& q);
bool is_finite(const Quaternion& q);
double dot(const Quaternion& a, const Quaternion& b);

/// Unit quaternion, or kInvalidQuaternion for a non-finite or near-zero input.
Result<Quaternion> normalize(const Quaternion& q);

/// Hamilton product. Total: no input can make this fail.
Quaternion multiply(const Quaternion& a, const Quaternion& b);

Quaternion conjugate(const Quaternion& q);

/// True inverse (`conjugate / norm^2`), so it is correct for a non-unit input
/// too. For a unit quaternion this equals conjugate() but costs a division;
/// prefer conjugate() when the input is known normalized.
Result<Quaternion> inverse(const Quaternion& q);

/// Rotate a vector by an *already normalized* quaternion.
///
/// The precondition is not checked. This is the FK inner loop, and every
/// quaternion reaching it has been normalized by RobotTree::build or by a
/// validating factory below. Use rotate() when the input is unvalidated.
Vector3 rotate_unit(const Quaternion& q, const Vector3& v);

/// Rotate a vector, normalizing the quaternion first.
Result<Vector3> rotate(const Quaternion& q, const Vector3& v);

// --- interpolation ----------------------------------------------------------

/// Above this dot product two rotations are close enough that the slerp
/// denominator (sin of the angle between them) stops being numerically useful,
/// and a normalized linear blend is both cheaper and better conditioned.
constexpr double kSlerpLinearThreshold = 0.9995;

/// Spherical linear interpolation along the *shortest* arc.
///
/// `b` is negated first when the two point into opposite hemispheres: q and -q
/// are the same rotation, so without that the interpolation takes the long way
/// round -- a visible 350-degree spin where 10 degrees was meant. Falls back to
/// nlerp() once the two are within kSlerpLinearThreshold.
///
/// `t` is not clamped: values outside [0, 1] extrapolate, which is occasionally
/// what a caller predicting ahead of a telemetry sample actually wants.
Result<Quaternion> slerp(const Quaternion& a, const Quaternion& b, double t);

/// Normalized linear interpolation, also along the shortest arc.
///
/// Cheaper than slerp and free of its near-parallel conditioning problem, at
/// the cost of non-constant angular velocity across the arc.
Result<Quaternion> nlerp(const Quaternion& a, const Quaternion& b, double t);

// --- conversions ------------------------------------------------------------

/// Rotation of `angle` radians about `axis`. The axis is normalized here, so
/// callers may pass a URDF axis verbatim.
Result<Quaternion> from_axis_angle(const Vector3& axis, double angle);

/// Inverse of from_axis_angle. The identity rotation yields the +X axis with a
/// zero angle rather than an error: the axis of a null rotation is arbitrary,
/// and a caller round-tripping one wants an identity back, not a failure.
Result<AxisAngle> to_axis_angle(const Quaternion& q);

/// Fixed-axis XYZ roll/pitch/yaw, i.e. `qz(yaw) * qy(pitch) * qx(roll)`.
/// This is the URDF/ROS convention. Total: any finite input is a valid
/// rotation. See CONVENTIONS.md for why this is *not* Three.js's default order.
Quaternion from_rpy(double roll, double pitch, double yaw);
Quaternion from_rpy(const Rpy& rpy);

/// Inverse of from_rpy. At a pitch of +/- pi/2 (gimbal lock) roll and yaw are
/// not separable; the whole rotation is reported as yaw with roll zero, which
/// still round-trips through from_rpy to the same quaternion.
Result<Rpy> to_rpy(const Quaternion& q);

/// Rotation about +Z only. Cheaper and exactly reversible against to_yaw().
Quaternion from_yaw(double yaw);

/// Rotation about +Z extracted from a full rotation.
///
/// Exactly `to_rpy(q).yaw`, including at gimbal lock, where the whole rotation
/// is attributed to yaw because roll and yaw are then the same degree of
/// freedom. Named `to_yaw` rather than `yaw` so the pair reads like every other
/// conversion here -- from_rpy/to_rpy, from_axis_angle/to_axis_angle.
Result<double> to_yaw(const Quaternion& q);

/// Row-major rotation matrix. Normalizes first, so a slightly-off unit input
/// still yields an orthonormal matrix.
Result<Matrix3> to_matrix(const Quaternion& q);

/// Inverse of to_matrix. Rejects a matrix that is not right-handed orthonormal
/// within `tolerance` rather than silently returning the nearest rotation --
/// a scaled or mirrored matrix reaching here is a bug upstream, not a rotation.
Result<Quaternion> from_matrix(const Matrix3& m, double tolerance = 1e-6);

} // namespace quat
} // namespace geometry
} // namespace cyberwave

#endif // CYBERWAVE_GEOMETRY_QUATERNION_HPP
