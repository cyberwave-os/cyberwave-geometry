#include "cyberwave/geometry/transform.hpp"

#include "cyberwave/geometry/quaternion.hpp"
#include "cyberwave/geometry/vector.hpp"

namespace cyberwave
{
namespace geometry
{
namespace tf
{

Result<Transform> normalize(const Transform& t)
{
    if (!vec::is_finite(t.translation))
    {
        return Result<Transform>::failure(ErrorCode::kNonFiniteValue, "", "translation has a non-finite component");
    }
    const Result<Quaternion> rotation = quat::normalize(t.rotation);
    if (!rotation)
    {
        return Result<Transform>(rotation.error());
    }
    return Transform{t.translation, rotation.value()};
}

Transform compose(const Transform& parent, const Transform& child)
{
    return Transform{
        vec::add(parent.translation, quat::rotate_unit(parent.rotation, child.translation)),
        quat::multiply(parent.rotation, child.rotation),
    };
}

Transform inverse_unit(const Transform& t)
{
    const Quaternion inverse_rotation = quat::conjugate(t.rotation);
    return Transform{
        vec::negate(quat::rotate_unit(inverse_rotation, t.translation)),
        inverse_rotation,
    };
}

Result<Transform> inverse(const Transform& t)
{
    const Result<Transform> unit = normalize(t);
    if (!unit)
    {
        return unit;
    }
    return inverse_unit(unit.value());
}

Vector3 apply(const Transform& t, const Vector3& point)
{
    return vec::add(t.translation, quat::rotate_unit(t.rotation, point));
}

Transform relative(const Transform& parent, const Transform& child) { return compose(inverse_unit(parent), child); }

} // namespace tf
} // namespace geometry
} // namespace cyberwave
