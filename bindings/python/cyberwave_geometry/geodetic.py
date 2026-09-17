"""GNSS positions and 6-DOF poses on Earth, delegated to the shared core.

The convention this module implements is ``common/geometry/CONVENTIONS.md``
section 9. The short version:

* A position is WGS-84 ``Geodetic(latitude_deg, longitude_deg, altitude_m)`` --
  degrees and metres, with no altitude datum of its own.
* A 6-DOF pose is ``GeoPose(position, orientation)``, where ``orientation``
  takes the body frame (forward-left-up) into the local **ENU** frame
  (east-north-up) at that position.
* A site is a ``GeoAnchor(origin, heading_deg)``: where the environment's origin
  sits on Earth, and the true-north bearing of environment +Y.
* NED attitudes and compass bearings are foreign conventions, reachable only
  through the named adapters at the bottom of this module.

    >>> from cyberwave_geometry import GeoAnchor, Geodetic, geodetic as geo
    >>> anchor = GeoAnchor(Geodetic(47.3769, 8.5417, 408.0), heading_deg=0.0)
    >>> round(geo.to_local(anchor, Geodetic(47.3769, 8.5417, 418.0)).z, 6)
    10.0

This package holds no formulas of its own, here as everywhere else: the
arithmetic lives once in the C++ core so the browser, an edge driver and the
backend cannot drift apart on where a robot is.
"""

from __future__ import annotations

from ctypes import byref, c_double

from ._native import (
    CGeoAnchor,
    CGeodetic,
    CGeoPose,
    CQuat,
    CTransform,
    CVec3,
    check,
    lib,
)
from .types import (
    GeoAnchor,
    Geodetic,
    GeoPose,
    MetresPerDegree,
    Quaternion,
    Rpy,
    Transform,
    Vector3,
)

__all__ = [
    "MAX_TANGENT_PLANE_LATITUDE",
    "WGS84_ECCENTRICITY_SQUARED",
    "WGS84_FLATTENING",
    "WGS84_SEMI_MAJOR_AXIS",
    "enu_between",
    "enu_to_local_rotation",
    "from_local",
    "from_local_pose",
    "ground_distance",
    "is_anchor_valid",
    "is_valid",
    "metres_per_degree",
    "normalize_bearing_deg",
    "normalize_longitude_deg",
    "offset",
    "quat_from_compass_heading",
    "quat_from_ned_rpy",
    "to_compass_heading_deg",
    "to_local",
    "to_local_pose",
    "to_ned_rpy",
    "validate",
    "validate_anchor",
]

#: WGS-84, mirrored from ``geodetic.hpp``. The only Earth model the core has.
WGS84_SEMI_MAJOR_AXIS = 6378137.0
WGS84_FLATTENING = 1.0 / 298.257223563
WGS84_ECCENTRICITY_SQUARED = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING)

#: Past this latitude the tangent plane's longitude scale degenerates, so an
#: anchor there is refused rather than silently amplified. Matches
#: ``MAX_ANCHOR_ABS_LATITUDE`` in the backend and the frontend.
MAX_TANGENT_PLANE_LATITUDE = 89.9


def _c_geodetic(g: Geodetic) -> CGeodetic:
    return CGeodetic(g.latitude_deg, g.longitude_deg, g.altitude_m)


def _py_geodetic(g: CGeodetic) -> Geodetic:
    return Geodetic(g.latitude_deg, g.longitude_deg, g.altitude_m)


def _c_anchor(anchor: GeoAnchor) -> CGeoAnchor:
    return CGeoAnchor(_c_geodetic(anchor.origin), anchor.heading_deg)


def _c_quat(q: Quaternion) -> CQuat:
    return CQuat(x=q.x, y=q.y, z=q.z, w=q.w)


def _py_quat(q: CQuat) -> Quaternion:
    return Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)


def _c_pose(pose: GeoPose) -> CGeoPose:
    return CGeoPose(_c_geodetic(pose.position), _c_quat(pose.orientation))


def _c_transform(t: Transform) -> CTransform:
    return CTransform(
        CVec3(t.translation.x, t.translation.y, t.translation.z),
        _c_quat(t.rotation),
    )


def _py_transform(t: CTransform) -> Transform:
    return Transform(
        Vector3(t.translation.x, t.translation.y, t.translation.z),
        _py_quat(t.rotation),
    )


# --- validation --------------------------------------------------------------


def validate(position: Geodetic) -> Geodetic:
    """Return ``position`` if it is usable, else raise :class:`GeometryError`.

    Finite, latitude in ``[-90, 90]``, longitude in ``[-180, 180]``. There is no
    clamping branch and no zero default: ``(0, 0)`` is a real place in the Gulf
    of Guinea, so a missing coordinate must fail loudly rather than relocate a
    robot to the equator.
    """
    check(lib.cw_geom_geo_validate(_c_geodetic(position)))
    return position


def validate_anchor(anchor: GeoAnchor) -> GeoAnchor:
    """Return ``anchor`` if it can convert fixes, else raise.

    Adds two requirements to :func:`validate`: a finite heading, and an origin
    at most :data:`MAX_TANGENT_PLANE_LATITUDE` from the equator.
    """
    check(lib.cw_geom_geo_validate_anchor(_c_anchor(anchor)))
    return anchor


def is_valid(position: Geodetic) -> bool:
    """Non-raising :func:`validate`."""
    return lib.cw_geom_geo_validate(_c_geodetic(position)) == 0


def is_anchor_valid(anchor: GeoAnchor) -> bool:
    """Non-raising :func:`validate_anchor`."""
    return lib.cw_geom_geo_validate_anchor(_c_anchor(anchor)) == 0


# --- scale and wrapping ------------------------------------------------------


def normalize_longitude_deg(longitude_deg: float) -> float:
    """Wrap a longitude into ``[-180, 180)``.

    Apply it to anchor-to-target *differences*, not only to outputs: subtracting
    two raw longitudes across the antimeridian reports ~360 degrees where the
    true separation is metres, which is how GPS ends up simply never working for
    a site in Fiji or Chukotka.
    """
    return float(lib.cw_geom_geo_normalize_longitude_deg(float(longitude_deg)))


def normalize_bearing_deg(bearing_deg: float) -> float:
    """Wrap a bearing into ``[0, 360)``."""
    return float(lib.cw_geom_geo_normalize_bearing_deg(float(bearing_deg)))


def metres_per_degree(latitude_deg: float) -> MetresPerDegree:
    """WGS-84 scale at a latitude. ``longitude`` already includes ``cos(phi)``.

    Not a single spherical constant: treating the semi-major axis as a mean
    radius carries a +0.67% .. -0.29% scale error, which is metres per kilometre
    of position, not noise.
    """
    lat = c_double()
    lon = c_double()
    check(
        lib.cw_geom_geo_metres_per_degree(float(latitude_deg), byref(lat), byref(lon))
    )
    return MetresPerDegree(lat.value, lon.value)


# --- the local tangent plane -------------------------------------------------


def enu_between(origin: Geodetic, target: Geodetic) -> Vector3:
    """ENU metres from ``origin`` to ``target``: ``x`` east, ``y`` north, ``z`` up.

    ``z`` is the plain altitude difference, with no datum conversion. Truncation
    is about ``d^2 / 2R`` -- 2 cm at 500 m, 31 cm at 2 km -- so a caller spanning
    more than a few kilometres wants its own anchor, not a better projection.
    """
    out = CVec3()
    check(
        lib.cw_geom_geo_enu_between(
            _c_geodetic(origin), _c_geodetic(target), byref(out)
        )
    )
    return Vector3(out.x, out.y, out.z)


def offset(origin: Geodetic, enu: Vector3) -> Geodetic:
    """``origin`` displaced by an ENU offset in metres. Inverse of :func:`enu_between`."""
    out = CGeodetic()
    check(
        lib.cw_geom_geo_offset(
            _c_geodetic(origin), CVec3(enu.x, enu.y, enu.z), byref(out)
        )
    )
    return _py_geodetic(out)


def ground_distance(a: Geodetic, b: Geodetic) -> float:
    """Horizontal separation in metres, ignoring altitude.

    What a plausibility guard compares against: a fix hundreds of kilometres from
    its anchor means the anchor or the device is wrong, not that a robot
    travelled. The core deliberately does not own that threshold -- rejecting is
    a product decision, and the number differs per caller.
    """
    out = c_double()
    check(lib.cw_geom_geo_ground_distance(_c_geodetic(a), _c_geodetic(b), byref(out)))
    return out.value


# --- against a site's anchor -------------------------------------------------


def enu_to_local_rotation(anchor: GeoAnchor) -> Quaternion:
    """The yaw that takes an ENU vector into the environment frame."""
    out = CQuat()
    check(lib.cw_geom_geo_enu_to_local_rotation(_c_anchor(anchor), byref(out)))
    return _py_quat(out)


def to_local(anchor: GeoAnchor, position: Geodetic) -> Vector3:
    """Geodetic position to environment-frame metres."""
    out = CVec3()
    check(
        lib.cw_geom_geo_to_local(_c_anchor(anchor), _c_geodetic(position), byref(out))
    )
    return Vector3(out.x, out.y, out.z)


def from_local(anchor: GeoAnchor, position: Vector3) -> Geodetic:
    """Environment-frame metres to a geodetic position."""
    out = CGeodetic()
    check(
        lib.cw_geom_geo_from_local(
            _c_anchor(anchor), CVec3(position.x, position.y, position.z), byref(out)
        )
    )
    return _py_geodetic(out)


def to_local_pose(anchor: GeoAnchor, pose: GeoPose) -> Transform:
    """A 6-DOF pose on Earth as an environment-frame :class:`Transform`.

    The result is ready to hand to ``fk.RobotTree.frame_pose`` as its base.
    """
    out = CTransform()
    check(lib.cw_geom_geo_to_local_pose(_c_anchor(anchor), _c_pose(pose), byref(out)))
    return _py_transform(out)


def from_local_pose(anchor: GeoAnchor, pose: Transform) -> GeoPose:
    """An environment-frame pose read back out as a place on Earth."""
    out = CGeoPose()
    check(
        lib.cw_geom_geo_from_local_pose(
            _c_anchor(anchor), _c_transform(pose), byref(out)
        )
    )
    return GeoPose(_py_geodetic(out.position), _py_quat(out.orientation))


# --- foreign conventions, quarantined ----------------------------------------


def quat_from_compass_heading(heading_deg: float) -> Quaternion:
    """A yaw-only ENU orientation for a bearing clockwise from TRUE north.

    Publishers apply magnetic declination themselves: most compasses and
    Android's rotation vector are magnetic-referenced, and declination changes
    over a site's lifetime, so correcting it anchor-side would silently rot.

    Yaw-only on purpose -- a fix plus a compass carries no roll or pitch, and an
    invented level attitude is indistinguishable from a measured one.
    """
    out = CQuat()
    check(lib.cw_geom_geo_quat_from_compass_heading(float(heading_deg), byref(out)))
    return _py_quat(out)


def to_compass_heading_deg(enu_orientation: Quaternion) -> float:
    """The bearing of an ENU orientation, degrees clockwise from true north.

    Wrapped to ``[0, 360)``; roll and pitch are discarded.
    """
    out = c_double()
    check(lib.cw_geom_geo_to_compass_heading_deg(_c_quat(enu_orientation), byref(out)))
    return out.value


def quat_from_ned_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """A NED attitude as an ENU orientation. Angles in **radians**.

    NED is the aviation/DJI frame -- +X north, +Y east, +Z down -- so its yaw is
    a compass heading and its positive pitch is nose-**up**, where ENU/FLU pitch
    turns about +Y = left and so is nose-**down**. Feeding NED angles into
    ``quaternion.from_rpy`` unconverted is a reflection about the north-east
    diagonal: 90 degrees wrong at every cardinal heading, and right at 45 and 225,
    which is exactly why it survives a casual test.
    """
    out = CQuat()
    check(
        lib.cw_geom_geo_quat_from_ned_rpy(
            float(roll), float(pitch), float(yaw), byref(out)
        )
    )
    return _py_quat(out)


def to_ned_rpy(enu_orientation: Quaternion) -> Rpy:
    """An ENU orientation as NED roll/pitch/yaw in radians.

    Inverse of :func:`quat_from_ned_rpy`, with the same gimbal-lock caveat as
    ``quaternion.to_rpy``: at pitch ``+-pi/2`` the angles will not match what you
    put in, but re-composing them reconstructs the same rotation.
    """
    roll = c_double()
    pitch = c_double()
    yaw = c_double()
    check(
        lib.cw_geom_geo_to_ned_rpy(
            _c_quat(enu_orientation), byref(roll), byref(pitch), byref(yaw)
        )
    )
    return Rpy(roll.value, pitch.value, yaw.value)
