// Umbrella header for the Cyberwave shared geometry core.
//
// This library is the single executable source of truth for scalar quaternion,
// transform and forward-kinematics arithmetic across the monorepo. See
// common/geometry/CONVENTIONS.md before adding anything here.
#ifndef CYBERWAVE_GEOMETRY_GEOMETRY_HPP
#define CYBERWAVE_GEOMETRY_GEOMETRY_HPP

#include "cyberwave/geometry/errors.hpp"
#include "cyberwave/geometry/fk.hpp"
#include "cyberwave/geometry/geodetic.hpp"
#include "cyberwave/geometry/quaternion.hpp"
#include "cyberwave/geometry/transform.hpp"
#include "cyberwave/geometry/types.hpp"
#include "cyberwave/geometry/vector.hpp"

namespace cyberwave
{
namespace geometry
{

/// Bumped whenever the numeric behaviour of the core changes, so a binding can
/// refuse to load against a core it was not conformance-tested against.
constexpr int kVersionMajor = 0;
constexpr int kVersionMinor = 2;
constexpr int kVersionPatch = 0;

const char* version_string();

} // namespace geometry
} // namespace cyberwave

#endif // CYBERWAVE_GEOMETRY_GEOMETRY_HPP
