#include "cyberwave/geometry/quaternion.hpp"

#include "cyberwave/geometry/vector.hpp"

#include <cmath>

namespace cyberwave
{
namespace geometry
{
namespace quat
{

namespace
{
/// Below this |cos(pitch)| a fixed-XYZ decomposition is genuinely singular.
///
/// Tested on cos(pitch), never on sin(pitch). The two are the same test
/// analytically, but only one survives double precision: 1e-7 rad from
/// vertical sin(pitch) is 1 - 5e-15, three ulps from the pole's own value and
/// so indistinguishable from it, while hypot(R00, R10) is 1e-7 with its full
/// relative accuracy. Testing sin declared every attitude within ~1.4e-6 rad
/// of vertical singular and discarded a roll the caller could still have had
/// -- a wrong rotation, not a rounding error, for anything that then rebuilds
/// through from_rpy (CYB-3869).
constexpr double kMinCosPitch = 1e-12;
} // namespace

Quaternion from_wxyz(double w, double x, double y, double z) { return Quaternion{.x = x, .y = y, .z = z, .w = w}; }

Quaternion from_xyzw(double x, double y, double z, double w) { return Quaternion{.x = x, .y = y, .z = z, .w = w}; }

Quaternion from_wxyz(const std::array<double, 4>& c) { return Quaternion{.x = c[1], .y = c[2], .z = c[3], .w = c[0]}; }

Quaternion from_xyzw(const std::array<double, 4>& c) { return Quaternion{.x = c[0], .y = c[1], .z = c[2], .w = c[3]}; }

std::array<double, 4> to_wxyz(const Quaternion& q) { return {q.w, q.x, q.y, q.z}; }

std::array<double, 4> to_xyzw(const Quaternion& q) { return {q.x, q.y, q.z, q.w}; }

double squared_norm(const Quaternion& q) { return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z; }

double norm(const Quaternion& q) { return std::sqrt(squared_norm(q)); }

bool is_finite(const Quaternion& q)
{
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

double dot(const Quaternion& a, const Quaternion& b) { return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; }

Result<Quaternion> normalize(const Quaternion& q)
{
    if (!is_finite(q))
    {
        return Result<Quaternion>::failure(ErrorCode::kInvalidQuaternion, "", "quaternion has a non-finite component");
    }
    const double length = norm(q);
    if (length < kMinQuaternionNorm)
    {
        return Result<Quaternion>::failure(ErrorCode::kInvalidQuaternion, "", "quaternion norm is below the minimum");
    }
    return Quaternion{.x = q.x / length, .y = q.y / length, .z = q.z / length, .w = q.w / length};
}

Quaternion multiply(const Quaternion& a, const Quaternion& b)
{
    return Quaternion{
        .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

Quaternion conjugate(const Quaternion& q) { return Quaternion{.x = -q.x, .y = -q.y, .z = -q.z, .w = q.w}; }

Result<Quaternion> inverse(const Quaternion& q)
{
    if (!is_finite(q))
    {
        return Result<Quaternion>::failure(ErrorCode::kInvalidQuaternion, "", "quaternion has a non-finite component");
    }
    const double sq = squared_norm(q);
    if (sq < kMinQuaternionNorm * kMinQuaternionNorm)
    {
        return Result<Quaternion>::failure(ErrorCode::kInvalidQuaternion, "", "quaternion norm is below the minimum");
    }
    return Quaternion{.x = -q.x / sq, .y = -q.y / sq, .z = -q.z / sq, .w = q.w / sq};
}

Vector3 rotate_unit(const Quaternion& q, const Vector3& v)
{
    // v + 2w(u x v) + 2(u x (u x v)), with u the vector part. Two crosses
    // instead of the q*v*q^-1 sandwich: same result, roughly half the
    // multiplies, and no temporary quaternion.
    const double tx = 2.0 * (q.y * v.z - q.z * v.y);
    const double ty = 2.0 * (q.z * v.x - q.x * v.z);
    const double tz = 2.0 * (q.x * v.y - q.y * v.x);
    return Vector3{
        v.x + q.w * tx + (q.y * tz - q.z * ty),
        v.y + q.w * ty + (q.z * tx - q.x * tz),
        v.z + q.w * tz + (q.x * ty - q.y * tx),
    };
}

Result<Vector3> rotate(const Quaternion& q, const Vector3& v)
{
    if (!vec::is_finite(v))
    {
        return Result<Vector3>::failure(ErrorCode::kNonFiniteValue, "", "vector has a non-finite component");
    }
    const Result<Quaternion> unit = normalize(q);
    if (!unit)
    {
        return Result<Vector3>(unit.error());
    }
    return rotate_unit(unit.value(), v);
}

namespace
{

/// Normalize both inputs and orient `b` onto the shortest arc from `a`.
struct ShortestArc
{
    Quaternion a;
    Quaternion b;
    double dot = 0.0;
};

Result<ShortestArc> shortest_arc(const Quaternion& a, const Quaternion& b)
{
    const Result<Quaternion> unit_a = normalize(a);
    if (!unit_a)
    {
        return Result<ShortestArc>(unit_a.error());
    }
    const Result<Quaternion> unit_b = normalize(b);
    if (!unit_b)
    {
        return Result<ShortestArc>(unit_b.error());
    }

    ShortestArc arc;
    arc.a = unit_a.value();
    arc.b = unit_b.value();
    arc.dot = dot(arc.a, arc.b);
    if (arc.dot < 0.0)
    {
        arc.b = Quaternion{.x = -arc.b.x, .y = -arc.b.y, .z = -arc.b.z, .w = -arc.b.w};
        arc.dot = -arc.dot;
    }
    // Rounding can push a legitimately identical pair a hair past 1, and acos
    // of that is NaN.
    if (arc.dot > 1.0)
    {
        arc.dot = 1.0;
    }
    return arc;
}

} // namespace

Result<Quaternion> nlerp(const Quaternion& a, const Quaternion& b, double t)
{
    if (!std::isfinite(t))
    {
        return Result<Quaternion>::failure(ErrorCode::kNonFiniteValue, "", "interpolation parameter is not finite");
    }
    const Result<ShortestArc> arc = shortest_arc(a, b);
    if (!arc)
    {
        return Result<Quaternion>(arc.error());
    }
    const Quaternion& left = arc.value().a;
    const Quaternion& right = arc.value().b;
    return normalize(Quaternion{
        .x = left.x + t * (right.x - left.x),
        .y = left.y + t * (right.y - left.y),
        .z = left.z + t * (right.z - left.z),
        .w = left.w + t * (right.w - left.w),
    });
}

Result<Quaternion> slerp(const Quaternion& a, const Quaternion& b, double t)
{
    if (!std::isfinite(t))
    {
        return Result<Quaternion>::failure(ErrorCode::kNonFiniteValue, "", "interpolation parameter is not finite");
    }
    const Result<ShortestArc> arc = shortest_arc(a, b);
    if (!arc)
    {
        return Result<Quaternion>(arc.error());
    }
    if (arc.value().dot > kSlerpLinearThreshold)
    {
        return nlerp(a, b, t);
    }

    const Quaternion& left = arc.value().a;
    const Quaternion& right = arc.value().b;
    const double theta = std::acos(arc.value().dot);
    const double sin_theta = std::sin(theta);
    const double weight_a = std::sin((1.0 - t) * theta) / sin_theta;
    const double weight_b = std::sin(t * theta) / sin_theta;
    return normalize(Quaternion{
        .x = weight_a * left.x + weight_b * right.x,
        .y = weight_a * left.y + weight_b * right.y,
        .z = weight_a * left.z + weight_b * right.z,
        .w = weight_a * left.w + weight_b * right.w,
    });
}

Result<Quaternion> from_axis_angle(const Vector3& axis, double angle)
{
    if (!std::isfinite(angle))
    {
        return Result<Quaternion>::failure(ErrorCode::kNonFiniteValue, "", "angle is not finite");
    }
    const Result<Vector3> unit = vec::normalize_axis(axis);
    if (!unit)
    {
        return Result<Quaternion>(unit.error());
    }
    const double half = angle * 0.5;
    const double sin_half = std::sin(half);
    return Quaternion{
        .x = unit.value().x * sin_half,
        .y = unit.value().y * sin_half,
        .z = unit.value().z * sin_half,
        .w = std::cos(half),
    };
}

Result<AxisAngle> to_axis_angle(const Quaternion& q)
{
    const Result<Quaternion> unit = normalize(q);
    if (!unit)
    {
        return Result<AxisAngle>(unit.error());
    }
    Quaternion u = unit.value();
    // A quaternion and its negation are the same rotation; picking w >= 0 keeps
    // the reported angle in [0, pi] so round-tripping is stable.
    if (u.w < 0.0)
    {
        u = Quaternion{.x = -u.x, .y = -u.y, .z = -u.z, .w = -u.w};
    }
    const double vector_norm = std::sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
    if (vector_norm < kMinAxisNorm)
    {
        // Identity: the axis is arbitrary. +X with a zero angle round-trips.
        return AxisAngle{Vector3{1.0, 0.0, 0.0}, 0.0};
    }
    const double angle = 2.0 * std::atan2(vector_norm, u.w);
    return AxisAngle{
        Vector3{u.x / vector_norm, u.y / vector_norm, u.z / vector_norm},
        angle,
    };
}

Quaternion from_rpy(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    return Quaternion{
        .x = sr * cp * cy - cr * sp * sy,
        .y = cr * sp * cy + sr * cp * sy,
        .z = cr * cp * sy - sr * sp * cy,
        .w = cr * cp * cy + sr * sp * sy,
    };
}

Quaternion from_rpy(const Rpy& rpy) { return from_rpy(rpy.roll, rpy.pitch, rpy.yaw); }

Result<Rpy> to_rpy(const Quaternion& q)
{
    const Result<Quaternion> unit = normalize(q);
    if (!unit)
    {
        return Result<Rpy>(unit.error());
    }
    const Quaternion u = unit.value();

    // The first column of the rotation matrix. |cos(pitch)| is its length, and
    // yaw is its direction -- see kMinCosPitch for why the singularity is
    // decided on this and not on sin(pitch).
    const double r00 = 1.0 - 2.0 * (u.y * u.y + u.z * u.z);
    const double r10 = 2.0 * (u.w * u.z + u.x * u.y);
    const double cos_pitch = std::hypot(r00, r10);

    // sin(pitch), clamped: rounding can push a legitimately vertical pitch a
    // hair past 1.
    double sin_pitch = 2.0 * (u.w * u.y - u.z * u.x);
    if (sin_pitch > 1.0)
    {
        sin_pitch = 1.0;
    }
    if (sin_pitch < -1.0)
    {
        sin_pitch = -1.0;
    }

    Rpy rpy;
    // atan2 of the pair, never asin(sin_pitch). asin has infinite slope at 1, so
    // a sin_pitch 2e-16 short of it -- what a double round trip through from_rpy
    // produces -- comes back 2e-8 short of pi/2, and rebuilding from that misses
    // the original quaternion by ~7e-9. atan2 stays well conditioned there, and
    // at the pole itself returns pi/2 to within an ulp, so nothing needs
    // snapping.
    rpy.pitch = std::atan2(sin_pitch, cos_pitch);
    if (cos_pitch < kMinCosPitch)
    {
        // Gimbal lock: roll and yaw turn the same axis, so there is no unique
        // split. Attribute the whole rotation to yaw with roll zero, which
        // still reconstructs the original quaternion through from_rpy. With
        // roll pinned to zero the residual is a pure Z turn at either pole, so
        // the same expression serves both.
        rpy.roll = 0.0;
        rpy.yaw = 2.0 * std::atan2(u.z, u.w);
        return rpy;
    }
    rpy.roll = std::atan2(2.0 * (u.w * u.x + u.y * u.z), 1.0 - 2.0 * (u.x * u.x + u.y * u.y));
    rpy.yaw = std::atan2(r10, r00);
    return rpy;
}

Quaternion from_yaw(double yaw)
{
    const double half = yaw * 0.5;
    return Quaternion{.x = 0.0, .y = 0.0, .z = std::sin(half), .w = std::cos(half)};
}

Result<double> to_yaw(const Quaternion& q)
{
    const Result<Quaternion> unit = normalize(q);
    if (!unit)
    {
        return Result<double>(unit.error());
    }
    const Quaternion u = unit.value();

    // Same gimbal-lock branch as to_rpy, on the same quantity: at pitch =
    // +/- pi/2 the planar yaw numerator and denominator both collapse toward
    // zero, so the atan2 below splits the rotation between roll and yaw in a
    // way that is platform/compiler-sensitive. Use the same canonical pole
    // representation as to_rpy instead -- roll zero, the whole remaining
    // rotation in yaw -- since to_yaw(q) has to equal to_rpy(q).yaw everywhere,
    // which includes agreeing on where the pole starts.
    const double r00 = 1.0 - 2.0 * (u.y * u.y + u.z * u.z);
    const double r10 = 2.0 * (u.w * u.z + u.x * u.y);
    if (std::hypot(r00, r10) < kMinCosPitch)
    {
        return 2.0 * std::atan2(u.z, u.w);
    }
    return std::atan2(r10, r00);
}

Result<Matrix3> to_matrix(const Quaternion& q)
{
    const Result<Quaternion> unit = normalize(q);
    if (!unit)
    {
        return Result<Matrix3>(unit.error());
    }
    const Quaternion u = unit.value();
    Matrix3 out;
    out.at(0, 0) = 1.0 - 2.0 * (u.y * u.y + u.z * u.z);
    out.at(0, 1) = 2.0 * (u.x * u.y - u.w * u.z);
    out.at(0, 2) = 2.0 * (u.x * u.z + u.w * u.y);
    out.at(1, 0) = 2.0 * (u.x * u.y + u.w * u.z);
    out.at(1, 1) = 1.0 - 2.0 * (u.x * u.x + u.z * u.z);
    out.at(1, 2) = 2.0 * (u.y * u.z - u.w * u.x);
    out.at(2, 0) = 2.0 * (u.x * u.z - u.w * u.y);
    out.at(2, 1) = 2.0 * (u.y * u.z + u.w * u.x);
    out.at(2, 2) = 1.0 - 2.0 * (u.x * u.x + u.y * u.y);
    return out;
}

namespace
{

Vector3 column(const Matrix3& m, int index) { return Vector3{m.at(0, index), m.at(1, index), m.at(2, index)}; }

} // namespace

Result<Quaternion> from_matrix(const Matrix3& m, double tolerance)
{
    for (double component : m.m)
    {
        if (!std::isfinite(component))
        {
            return Result<Quaternion>::failure(ErrorCode::kNonFiniteValue, "", "matrix has a non-finite component");
        }
    }

    const Vector3 c0 = column(m, 0);
    const Vector3 c1 = column(m, 1);
    const Vector3 c2 = column(m, 2);
    const double checks[6] = {
        vec::squared_norm(c0) - 1.0,
        vec::squared_norm(c1) - 1.0,
        vec::squared_norm(c2) - 1.0,
        vec::dot(c0, c1),
        vec::dot(c0, c2),
        vec::dot(c1, c2),
    };
    for (double deviation : checks)
    {
        if (std::abs(deviation) > tolerance)
        {
            return Result<Quaternion>::failure(ErrorCode::kInvalidRotationMatrix, "",
                                               "matrix columns are not orthonormal");
        }
    }
    // Right-handed: a mirrored frame has determinant -1 and no quaternion.
    if (vec::dot(vec::cross(c0, c1), c2) <= 0.0)
    {
        return Result<Quaternion>::failure(ErrorCode::kInvalidRotationMatrix, "",
                                           "matrix is left-handed (negative determinant)");
    }

    // Shepperd's method: pick the branch with the largest denominator so the
    // division never amplifies rounding near a 180-degree rotation.
    const double trace = m.at(0, 0) + m.at(1, 1) + m.at(2, 2);
    Quaternion out;
    if (trace > 0.0)
    {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        out.w = 0.25 * s;
        out.x = (m.at(2, 1) - m.at(1, 2)) / s;
        out.y = (m.at(0, 2) - m.at(2, 0)) / s;
        out.z = (m.at(1, 0) - m.at(0, 1)) / s;
    }
    else if (m.at(0, 0) > m.at(1, 1) && m.at(0, 0) > m.at(2, 2))
    {
        const double s = std::sqrt(1.0 + m.at(0, 0) - m.at(1, 1) - m.at(2, 2)) * 2.0;
        out.w = (m.at(2, 1) - m.at(1, 2)) / s;
        out.x = 0.25 * s;
        out.y = (m.at(0, 1) + m.at(1, 0)) / s;
        out.z = (m.at(0, 2) + m.at(2, 0)) / s;
    }
    else if (m.at(1, 1) > m.at(2, 2))
    {
        const double s = std::sqrt(1.0 + m.at(1, 1) - m.at(0, 0) - m.at(2, 2)) * 2.0;
        out.w = (m.at(0, 2) - m.at(2, 0)) / s;
        out.x = (m.at(0, 1) + m.at(1, 0)) / s;
        out.y = 0.25 * s;
        out.z = (m.at(1, 2) + m.at(2, 1)) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + m.at(2, 2) - m.at(0, 0) - m.at(1, 1)) * 2.0;
        out.w = (m.at(1, 0) - m.at(0, 1)) / s;
        out.x = (m.at(0, 2) + m.at(2, 0)) / s;
        out.y = (m.at(1, 2) + m.at(2, 1)) / s;
        out.z = 0.25 * s;
    }
    return normalize(out);
}

} // namespace quat
} // namespace geometry
} // namespace cyberwave
