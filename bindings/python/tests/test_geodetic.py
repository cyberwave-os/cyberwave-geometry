"""The geodetic convention through the Python binding.

The C++ suite already pins the arithmetic; what is worth testing here is the
marshalling -- three doubles across an ABI in an order nothing checks for you --
and the strict/lenient split, which is the part a caller actually trips over.
"""

from __future__ import annotations

import math

import pytest

from cyberwave_geometry import (
    ErrorCode,
    GeoAnchor,
    Geodetic,
    GeometryError,
    GeoPose,
    Quaternion,
    Transform,
    Vector3,
)
from cyberwave_geometry import geodetic as geo
from cyberwave_geometry import quaternion as quat

#: A site with a non-zero heading, so nothing here passes only because the
#: environment frame happened to coincide with ENU.
ANCHOR = GeoAnchor(Geodetic(47.3769, 8.5417, 408.0), heading_deg=30.0)
ENU_ANCHOR = GeoAnchor(Geodetic(47.3769, 8.5417, 408.0), heading_deg=0.0)


def test_the_struct_crosses_the_abi_in_the_right_order():
    # Three bare doubles with no names on the wire: a field swap here would put a
    # robot in the wrong hemisphere and would not raise anything.
    # One degree at the equator is 110 574.3 m of latitude and 111 319.5 m of
    # longitude -- different numbers, which is what makes this test able to tell
    # the two fields apart at all.
    moved = geo.offset(Geodetic(0.0, 0.0, 0.0), Vector3(0.0, 110_574.3, 0.0))
    assert moved.latitude_deg == pytest.approx(1.0, abs=1e-5)
    assert moved.longitude_deg == pytest.approx(0.0, abs=1e-12)

    moved_east = geo.offset(Geodetic(0.0, 0.0, 0.0), Vector3(111_319.5, 0.0, 0.0))
    assert moved_east.longitude_deg == pytest.approx(1.0, abs=1e-5)
    assert moved_east.latitude_deg == pytest.approx(0.0, abs=1e-12)

    higher = geo.offset(Geodetic(0.0, 0.0, 10.0), Vector3(0.0, 0.0, 5.0))
    assert higher.altitude_m == pytest.approx(15.0)


def test_zero_heading_means_the_environment_frame_is_enu():
    east = geo.offset(ENU_ANCHOR.origin, Vector3(100.0, 0.0, 0.0))
    local = geo.to_local(ENU_ANCHOR, east)
    assert local.x == pytest.approx(100.0, abs=1e-6)
    assert local.y == pytest.approx(0.0, abs=1e-6)


def test_heading_is_the_bearing_of_environment_plus_y():
    rotated = GeoAnchor(ANCHOR.origin, heading_deg=90.0)
    east = geo.offset(rotated.origin, Vector3(100.0, 0.0, 0.0))
    local = geo.to_local(rotated, east)
    assert local.x == pytest.approx(0.0, abs=1e-6)
    assert local.y == pytest.approx(100.0, abs=1e-6)


def test_metres_per_degree_matches_the_published_wgs84_values():
    equator = geo.metres_per_degree(0.0)
    assert equator.latitude == pytest.approx(110_574.3, abs=0.1)
    assert equator.longitude == pytest.approx(111_319.5, abs=0.1)

    # Longitude converges towards the poles; latitude grows.
    high = geo.metres_per_degree(78.0)
    assert high.longitude == pytest.approx(23_220.0, abs=10.0)
    assert high.latitude > equator.latitude


def test_positions_round_trip():
    position = Vector3(-412.0, 118.5, -7.25)
    recovered = geo.to_local(ANCHOR, geo.from_local(ANCHOR, position))
    assert recovered.x == pytest.approx(position.x, abs=1e-6)
    assert recovered.y == pytest.approx(position.y, abs=1e-6)
    assert recovered.z == pytest.approx(position.z, abs=1e-9)


def test_a_six_dof_pose_round_trips():
    pose = GeoPose(Geodetic(47.3801, 8.5500, 421.0), quat.from_rpy(0.12, -0.34, 1.05))
    local = geo.to_local_pose(ANCHOR, pose)
    assert isinstance(local, Transform)
    recovered = geo.from_local_pose(ANCHOR, local)
    assert recovered.position.latitude_deg == pytest.approx(
        pose.position.latitude_deg, abs=1e-11
    )
    assert recovered.position.longitude_deg == pytest.approx(
        pose.position.longitude_deg, abs=1e-11
    )
    assert recovered.position.altitude_m == pytest.approx(pose.position.altitude_m)
    assert quat.dot(recovered.orientation, pose.orientation) == pytest.approx(
        1.0, abs=1e-12
    )


def test_an_identity_orientation_faces_east_not_north():
    # The one counter-intuitive consequence of an ENU-referenced pose. Pinned so
    # that nobody "fixes" it into a compass frame without changing the docs.
    assert geo.to_compass_heading_deg(Quaternion.identity()) == pytest.approx(90.0)


@pytest.mark.parametrize("bearing", [0.0, 17.5, 90.0, 179.9, 270.0, 359.5])
def test_compass_bearings_round_trip_and_come_back_positive(bearing):
    recovered = geo.to_compass_heading_deg(geo.quat_from_compass_heading(bearing))
    assert 0.0 <= recovered < 360.0
    assert recovered == pytest.approx(bearing, abs=1e-9)


def test_a_compass_heading_is_yaw_only():
    # A fix plus a compass carries no attitude; a fabricated level one would be
    # indistinguishable from a measured one.
    q = geo.quat_from_compass_heading(212.0)
    assert q.x == 0.0
    assert q.y == 0.0


def test_ned_is_a_quarantined_convention_not_the_native_one():
    # Straight from_rpy on NED angles is right at 45 degrees and wrong at every
    # cardinal heading, so both are checked.
    assert geo.to_compass_heading_deg(
        geo.quat_from_ned_rpy(0.0, 0.0, 0.0)
    ) == pytest.approx(0.0, abs=1e-9)
    assert geo.to_compass_heading_deg(
        geo.quat_from_ned_rpy(0.0, 0.0, math.pi / 2)
    ) == pytest.approx(90.0, abs=1e-9)

    naive = quat.from_rpy(0.0, 0.0, math.pi / 2)
    assert geo.to_compass_heading_deg(naive) != pytest.approx(90.0, abs=1e-6)

    # Positive NED pitch is nose-up; positive ENU/FLU pitch is nose-down.
    assert (
        quat.rotate(geo.quat_from_ned_rpy(0.0, 0.5, 0.0), Vector3(1.0, 0.0, 0.0)).z
        > 0.0
    )


def test_ned_attitude_round_trips():
    recovered = geo.to_ned_rpy(geo.quat_from_ned_rpy(0.21, -0.44, 2.05))
    assert recovered.roll == pytest.approx(0.21)
    assert recovered.pitch == pytest.approx(-0.44)
    assert recovered.yaw == pytest.approx(2.05)


def test_the_antimeridian_is_metres_away_not_a_half_turn():
    distance = geo.ground_distance(
        Geodetic(-16.5, 179.999, 0.0), Geodetic(-16.5, -179.999, 0.0)
    )
    assert distance < 500.0


# --- strictness --------------------------------------------------------------


@pytest.mark.parametrize(
    "position",
    [
        Geodetic(91.0, 0.0, 0.0),
        Geodetic(0.0, 181.0, 0.0),
        Geodetic(float("nan"), 0.0, 0.0),
        Geodetic(0.0, 0.0, float("inf")),
    ],
)
def test_an_unusable_coordinate_raises_rather_than_clamping(position):
    with pytest.raises(GeometryError) as excinfo:
        geo.validate(position)
    assert excinfo.value.code is ErrorCode.INVALID_GEODETIC
    assert not geo.is_valid(position)


def test_a_polar_anchor_is_refused():
    polar = GeoAnchor(Geodetic(89.95, 0.0, 0.0), heading_deg=0.0)
    assert not geo.is_anchor_valid(polar)
    with pytest.raises(GeometryError) as excinfo:
        geo.to_local(polar, polar.origin)
    assert excinfo.value.code is ErrorCode.INVALID_GEO_ANCHOR

    assert geo.is_anchor_valid(GeoAnchor(Geodetic(89.5, 0.0, 0.0), heading_deg=0.0))


def test_a_degenerate_orientation_is_refused_rather_than_defaulted():
    pose = GeoPose(ANCHOR.origin, Quaternion(x=0.0, y=0.0, z=0.0, w=0.0))
    with pytest.raises(GeometryError) as excinfo:
        geo.to_local_pose(ANCHOR, pose)
    assert excinfo.value.code is ErrorCode.INVALID_QUATERNION


def test_from_dict_refuses_a_missing_coordinate_rather_than_defaulting_to_zero():
    # (0, 0) is the Gulf of Guinea. Unlike Vector3.from_dict, which may default,
    # a dropped geodetic key has to fail: the fallback is a real place.
    assert geo.is_valid(
        Geodetic.from_dict({"latitude": 47.0, "longitude": 8.0, "altitude": 400.0})
    )
    for dropped in ("latitude", "longitude", "altitude"):
        payload = {"latitude": 47.0, "longitude": 8.0, "altitude": 400.0}
        del payload[dropped]
        with pytest.raises(GeometryError) as excinfo:
            Geodetic.from_dict(payload)
        assert dropped in str(excinfo.value)


def test_geo_pose_from_dict_reads_the_wire_spelling():
    pose = GeoPose.from_dict(
        {
            "latitude": 47.3801,
            "longitude": 8.55,
            "altitude": 421.0,
            "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        }
    )
    assert pose.position.latitude_deg == 47.3801
    assert pose.orientation == Quaternion.identity()
    assert pose.to_dict()["rotation"] == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
