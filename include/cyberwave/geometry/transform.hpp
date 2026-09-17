// Rigid transform composition and inversion.
#ifndef CYBERWAVE_GEOMETRY_TRANSFORM_HPP
#define CYBERWAVE_GEOMETRY_TRANSFORM_HPP

#include "cyberwave/geometry/errors.hpp"
#include "cyberwave/geometry/types.hpp"

namespace cyberwave
{
namespace geometry
{
namespace tf
{

/// Normalize the rotation, or fail. Use at every boundary where a Transform is
/// built from unvalidated data; everything below assumes the result of this.
Result<Transform> normalize(const Transform& t);

/// `parent` applied to `child`, i.e. child expressed in parent's parent frame.
/// Assumes both rotations are normalized (see normalize()).
Transform compose(const Transform& parent, const Transform& child);

/// The transform that undoes `t`. Assumes `t.rotation` is normalized.
Transform inverse_unit(const Transform& t);

/// The transform that undoes `t`, normalizing first.
Result<Transform> inverse(const Transform& t);

/// Map a point through the transform: `R*p + t`. Assumes normalized rotation.
Vector3 apply(const Transform& t, const Vector3& point);

/// Express a world-frame `child` pose in `parent`'s frame -- the inverse of
/// compose(). Assumes both rotations are normalized.
Transform relative(const Transform& parent, const Transform& child);

} // namespace tf
} // namespace geometry
} // namespace cyberwave

#endif // CYBERWAVE_GEOMETRY_TRANSFORM_HPP
