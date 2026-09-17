// Vector3 arithmetic. Small enough to be header-inline; the compiler folds
// these into the FK loop rather than emitting calls.
#ifndef CYBERWAVE_GEOMETRY_VECTOR_HPP
#define CYBERWAVE_GEOMETRY_VECTOR_HPP

#include "cyberwave/geometry/errors.hpp"
#include "cyberwave/geometry/types.hpp"

#include <cmath>

namespace cyberwave
{
namespace geometry
{
namespace vec
{

inline Vector3 add(const Vector3& a, const Vector3& b) { return Vector3{a.x + b.x, a.y + b.y, a.z + b.z}; }

inline Vector3 subtract(const Vector3& a, const Vector3& b) { return Vector3{a.x - b.x, a.y - b.y, a.z - b.z}; }

inline Vector3 negate(const Vector3& v) { return Vector3{-v.x, -v.y, -v.z}; }

inline Vector3 scale(const Vector3& v, double factor) { return Vector3{v.x * factor, v.y * factor, v.z * factor}; }

inline double dot(const Vector3& a, const Vector3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vector3 cross(const Vector3& a, const Vector3& b)
{
    return Vector3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline double squared_norm(const Vector3& v) { return dot(v, v); }

inline double norm(const Vector3& v) { return std::sqrt(squared_norm(v)); }

inline bool is_finite(const Vector3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

/// Unit vector, or kInvalidAxis when the input carries no direction.
///
/// Named for its only caller-visible purpose: every vector the core normalizes
/// is a joint axis, and a joint whose axis cannot be normalized has no motion
/// that can be composed.
inline Result<Vector3> normalize_axis(const Vector3& v)
{
    if (!is_finite(v))
    {
        return Result<Vector3>::failure(ErrorCode::kInvalidAxis, "", "axis has a non-finite component");
    }
    const double length = norm(v);
    if (length < kMinAxisNorm)
    {
        return Result<Vector3>::failure(ErrorCode::kInvalidAxis, "", "axis norm is below the minimum");
    }
    return Vector3{v.x / length, v.y / length, v.z / length};
}

} // namespace vec
} // namespace geometry
} // namespace cyberwave

#endif // CYBERWAVE_GEOMETRY_VECTOR_HPP
