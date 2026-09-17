// Where on Earth, and which way up: the geodetic (GNSS) end of the core.
//
// The rest of this library is frame-agnostic -- it composes whatever you hand
// it. This header is the deliberate exception: a latitude is not just a number,
// and the point of putting it here is that the frame it lives in is stated once,
// in a type, instead of being implied by which subsystem produced it.
//
// The convention is CONVENTIONS.md section 9. In short:
//
//   * Position is WGS-84 geodetic, in DEGREES and metres, named `_deg` / `_m`.
//   * Orientation is a Hamilton quaternion mapping the BODY frame (forward-left-
//     up) into the LOCAL ENU frame (east-north-up) at that same position.
//   * The two travel together as a GeoPose, because the second means nothing
//     without the first: ENU rotates with the ellipsoid normal, so the same
//     quaternion 100 km away is a different attitude.
//   * NED and compass bearings are foreign conventions, reachable only through
//     the named adapters at the bottom of this file -- the same quarantine
//     `wxyz` gets in quaternion.hpp.
//
// No geoid model, no ECEF round trip, no projection library, ever. The
// no-dependencies rule in README.md is not negotiable here either, and a local
// tangent plane is already one to two orders of magnitude tighter than consumer
// GNSS error at the ranges Cyberwave environments span.
#ifndef CYBERWAVE_GEOMETRY_GEODETIC_HPP
#define CYBERWAVE_GEOMETRY_GEODETIC_HPP

#include "cyberwave/geometry/errors.hpp"
#include "cyberwave/geometry/types.hpp"

namespace cyberwave
{
namespace geometry
{

/// WGS-84 ellipsoid. The only Earth model this library has, and the only one it
/// will get: everything here is a local tangent plane about a stated origin.
constexpr double kWgs84SemiMajorAxis = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84EccentricitySquared = kWgs84Flattening * (2.0 - kWgs84Flattening);

/// Beyond this latitude the tangent plane's longitude scale (`N * cos(phi)`)
/// degenerates, so converting metres back to a longitude divides by something
/// approaching zero. An origin past it is rejected rather than silently
/// amplified into a longitude thousands of degrees wide.
///
/// Matches MAX_ANCHOR_ABS_LATITUDE in the backend and the frontend, which apply
/// the same bound at their API boundaries.
constexpr double kMaxTangentPlaneLatitude = 89.9;

/// A position on the WGS-84 ellipsoid.
///
/// Degrees, not radians -- the one place this library breaks its own "angles are
/// radians" rule, and the fields say `_deg` so the exception is never silent.
/// Every producer and consumer of a geodetic coordinate in the stack speaks
/// degrees (NMEA, MAVLink, DJI MSDK, the `twin/{uuid}/gps` payload, a maps URL),
/// and a latitude in radians is indistinguishable by eye from one in degrees --
/// it is simply a place 57 times closer to the equator.
///
/// `altitude_m` carries NO datum of its own. Whether it is orthometric (MSL) or
/// ellipsoidal (HAE) is a property of the receiver and of the site survey, is
/// recorded beside the data, and is never converted here: the conversion needs a
/// geoid model this library refuses to depend on, for a correction that GNSS
/// vertical error swamps anyway. Mixing the two is a ~30 m error, so a publisher
/// must be matched to the datum its site was surveyed in.
struct Geodetic
{
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
};

/// Metres per degree at a given latitude, on the WGS-84 ellipsoid.
///
/// `longitude` already includes the `cos(phi)` convergence, so it shrinks
/// towards the poles. Both are positive except at a pole, where `longitude`
/// reaches zero.
struct MetresPerDegree
{
    double latitude = 0.0;
    double longitude = 0.0;
};

/// A 6-DOF pose on Earth: three degrees of freedom of position, three of
/// attitude. This is the answer to "how do we write down a GPS pose".
///
/// `orientation` maps the BODY frame into the LOCAL ENU frame at `position`:
///
///     v_enu = rotate(orientation, v_body)
///
/// with ENU being +X east, +Y north, +Z up, and the body frame being FLU --
/// +X forward, +Y left, +Z up (REP-103, the same body convention the rest of
/// this core assumes when `quat::from_yaw` measures yaw counter-clockwise from
/// +X). An identity orientation is therefore a body facing due EAST and level,
/// which is the one counter-intuitive consequence of building on ENU rather
/// than on a compass; `quat_from_compass_heading` exists so nobody has to hold
/// that in their head.
///
/// Position and orientation are ONE value on purpose. ENU is defined by the
/// ellipsoid normal at a point, so an orientation is only interpretable
/// alongside the position it was measured at, and assembling the two from
/// separately-timestamped messages produces a pose that was never real. The
/// wire contract already states this rule for camera exposure poses; the type
/// is where it stops being a comment.
struct GeoPose
{
    Geodetic position;
    Quaternion orientation;
};

/// A site's georeference: where the environment's origin sits on Earth, and
/// which way the environment frame is turned relative to true north.
///
/// `heading_deg` is the true-north bearing of environment +Y, in degrees
/// CLOCKWISE. `heading_deg == 0` therefore means the environment frame IS the
/// local ENU frame: +X east, +Y north, +Z up.
///
/// +Y rather than +X because "which way is north on my floor plan" is a
/// statement about the axis that runs up the screen in the default top-down
/// view, and because it makes zero mean the standard frame for a georeferenced
/// survey. This matches `environment.settings["geo"]` in the backend.
struct GeoAnchor
{
    Geodetic origin;
    double heading_deg = 0.0;
};

namespace geo
{

/// True when every component is finite. Says nothing about the ranges.
bool is_finite(const Geodetic& g);

/// Finite, latitude in [-90, 90], longitude in [-180, 180].
///
/// Deliberately does NOT default a missing field to zero anywhere upstream:
/// (0, 0) is a real location in the Gulf of Guinea, so an absent coordinate that
/// arrives as zero is a confidently wrong place on Earth rather than an obviously
/// missing one.
Result<Geodetic> validate(const Geodetic& g);

/// Valid origin, finite heading, and an origin off the poles -- see
/// kMaxTangentPlaneLatitude.
Result<GeoAnchor> validate(const GeoAnchor& anchor);

/// Valid position and a normalizable orientation.
Result<GeoPose> normalize(const GeoPose& pose);

/// Wrap a longitude into [-180, 180).
///
/// Applied to anchor-to-target DIFFERENCES, not only to outputs. Longitude is
/// the one coordinate that wraps, and subtracting two raw values across the
/// antimeridian reports ~360 degrees where the true separation is metres --
/// which is why a site in Fiji or Chukotka would otherwise compute every fix as
/// most of the way around the planet.
double normalize_longitude_deg(double longitude_deg);

/// Wrap a bearing into [0, 360).
double normalize_bearing_deg(double bearing_deg);

/// WGS-84 meridional (M) and prime-vertical (N) scale at a latitude:
///
///     M = a (1 - e2) / (1 - e2 sin^2 phi)^1.5
///     N = a / (1 - e2 sin^2 phi)^0.5
///
/// Not a single metres-per-degree constant. Treating the semi-major axis as a
/// mean radius carries a +0.67% .. -0.29% scale error -- metres per kilometre,
/// and it is all in the position, not in the noise.
Result<MetresPerDegree> metres_per_degree(double latitude_deg);

/// ENU offset in metres from `origin` to `target`, on the tangent plane at
/// `origin`. `z` is the plain altitude difference, with no datum conversion.
///
/// Tangent-plane truncation is about `d^2 / 2R`: 2 cm at 500 m, 31 cm at 2 km.
/// Callers that span more than a few kilometres want their own anchor, not a
/// better projection.
Result<Vector3> enu_between(const Geodetic& origin, const Geodetic& target);

/// `origin` displaced by an ENU offset in metres. The inverse of enu_between.
Result<Geodetic> offset(const Geodetic& origin, const Vector3& enu);

/// Horizontal ground distance in metres, ignoring altitude.
Result<double> ground_distance(const Geodetic& a, const Geodetic& b);

/// The rotation that takes an ENU vector into the environment frame: a yaw of
/// `anchor.heading_deg` about +Z. Its inverse goes the other way.
Result<Quaternion> enu_to_local_rotation(const GeoAnchor& anchor);

/// Geodetic position -> environment-frame metres.
Result<Vector3> to_local(const GeoAnchor& anchor, const Geodetic& position);

/// Environment-frame metres -> geodetic position.
Result<Geodetic> from_local(const GeoAnchor& anchor, const Vector3& position);

/// The 6-DOF conversions: a GeoPose is a Transform once the site is known.
///
/// `to_local` gives a pose ready to hand to `fk::compute_frame_pose` as its
/// base; `from_local` reads a scene pose back out as a place on Earth.
Result<Transform> to_local_pose(const GeoAnchor& anchor, const GeoPose& pose);
Result<GeoPose> from_local_pose(const GeoAnchor& anchor, const Transform& pose);

// --- foreign conventions, quarantined -------------------------------------
//
// Everything below converts between the ENU/FLU convention above and a
// convention Cyberwave does not get to choose. Same rule as `wxyz` in
// quaternion.hpp: the foreign order exists only behind a name that says which
// one it is, never as a bare value whose meaning depends on the reader.

/// A yaw-only ENU orientation for a compass bearing.
///
/// `heading_deg` is degrees CLOCKWISE from TRUE north. Publishers apply magnetic
/// declination themselves -- most compasses and Android's rotation vector are
/// magnetic-referenced, and declination changes over a site's lifetime, so
/// correcting it at the anchor would silently rot.
///
/// Yaw-only on purpose. A GNSS fix plus a compass carries no roll or pitch, and
/// inventing a level attitude would be indistinguishable from a measured one.
///
///     yaw_enu = pi/2 - heading
///
/// because a bearing is measured clockwise from north (+Y) while an ENU yaw is
/// measured counter-clockwise from east (+X).
Result<Quaternion> quat_from_compass_heading(double heading_deg);

/// The compass bearing of an ENU orientation, in degrees clockwise from true
/// north, wrapped to [0, 360). Roll and pitch are discarded.
Result<double> to_compass_heading_deg(const Quaternion& enu_orientation);

/// NED attitude -> ENU orientation. Angles in RADIANS, like every other attitude
/// in this core; it is bearings, not attitudes, that are degrees here.
///
/// NED is the aviation/DJI frame: +X north, +Y east, +Z down, so yaw is a
/// compass heading and positive pitch is nose-UP. The pitch sign is the trap:
/// ENU/FLU pitch turns about +Y = LEFT, so positive there is nose-DOWN (ROS
/// REP-103). The two conventions disagree, and nothing in the numbers says so.
/// The conversion is
///
///     roll_enu  =  roll_ned
///     pitch_enu = -pitch_ned
///     yaw_enu   =  pi/2 - yaw_ned
///
/// Feeding NED angles into `quat::from_rpy` unconverted is a reflection about
/// the north-east diagonal: the heading error is `2*yaw - 90 degrees`, so it is
/// 90 degrees wrong at every cardinal heading, half a turn at 135 and 315, and
/// correct only at 45 and 225 -- which is exactly why it survives a casual test.
Result<Quaternion> quat_from_ned_rpy(double roll, double pitch, double yaw);

/// ENU orientation -> NED roll/pitch/yaw in radians. The inverse of
/// quat_from_ned_rpy, with the same gimbal-lock caveat as `quat::to_rpy`.
Result<Rpy> to_ned_rpy(const Quaternion& enu_orientation);

} // namespace geo
} // namespace geometry
} // namespace cyberwave

#endif // CYBERWAVE_GEOMETRY_GEODETIC_HPP
