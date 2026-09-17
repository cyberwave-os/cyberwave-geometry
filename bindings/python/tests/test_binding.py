"""Behaviour of the Python binding itself, as opposed to numeric conformance.

The golden suite proves the arithmetic matches C++. This file covers what only
exists on the Python side: handle lifetimes, the strict/lenient split, and the
named-component discipline the whole library exists to enforce.
"""

from __future__ import annotations

import gc
import math

import pytest

from cyberwave_geometry import (
    ErrorCode,
    GeometryError,
    JointType,
    Quaternion,
    Transform,
    Vector3,
    compat,
    core_version,
)
from cyberwave_geometry import quaternion as quat
from cyberwave_geometry import transform as tf
from cyberwave_geometry.fk import (
    JointDescription,
    MimicSpec,
    RobotDescription,
    RobotTree,
    SensorDescription,
    joint_type_from_string,
)


def two_link_arm() -> RobotDescription:
    return RobotDescription(
        links=["base_link", "upper_arm", "forearm"],
        joints=[
            JointDescription(
                name="shoulder",
                parent_link="base_link",
                child_link="upper_arm",
                type=JointType.REVOLUTE,
                origin=Transform(Vector3(0, 0, 1), Quaternion.identity()),
                axis=Vector3(0, 0, 1),
            ),
            JointDescription(
                name="elbow",
                parent_link="upper_arm",
                child_link="forearm",
                type=JointType.REVOLUTE,
                origin=Transform(Vector3(1, 0, 0), Quaternion.identity()),
                axis=Vector3(0, 1, 0),
            ),
        ],
        sensors=[
            SensorDescription(
                name="wrist_camera",
                parent_link="forearm",
                extrinsic=Transform(Vector3(0.1, 0, 0), Quaternion.identity()),
            )
        ],
    )


# --- component order --------------------------------------------------------


def test_xyzw_and_wxyz_are_different_readings_of_the_same_numbers():
    numbers = [0.1, 0.2, 0.3, 0.4]
    assert Quaternion.from_wxyz(numbers).w == 0.1
    assert Quaternion.from_xyzw(numbers).w == 0.4


def test_component_order_round_trips():
    original = quat.from_rpy(0.4, -0.5, 0.6)
    assert Quaternion.from_wxyz(original.to_wxyz()) == original
    assert Quaternion.from_xyzw(original.to_xyzw()) == original


def test_dict_round_trip_keeps_named_components():
    original = quat.from_rpy(0.1, 0.2, 0.3)
    assert Quaternion.from_dict(original.to_dict()) == original
    assert set(original.to_dict()) == {"w", "x", "y", "z"}


# --- strictness -------------------------------------------------------------


@pytest.mark.parametrize(
    ("call", "code"),
    [
        (
            lambda: quat.normalize(Quaternion(x=0, y=0, z=0, w=0)),
            ErrorCode.INVALID_QUATERNION,
        ),
        (
            lambda: quat.inverse(Quaternion(x=0, y=0, z=0, w=0)),
            ErrorCode.INVALID_QUATERNION,
        ),
        (
            lambda: quat.to_rpy(Quaternion(x=0, y=0, z=0, w=0)),
            ErrorCode.INVALID_QUATERNION,
        ),
        (
            lambda: quat.to_yaw(Quaternion(x=0, y=0, z=0, w=0)),
            ErrorCode.INVALID_QUATERNION,
        ),
        (
            lambda: quat.to_matrix(Quaternion(x=0, y=0, z=0, w=0)),
            ErrorCode.INVALID_QUATERNION,
        ),
        (lambda: quat.from_axis_angle(Vector3(0, 0, 0), 1.0), ErrorCode.INVALID_AXIS),
        (
            lambda: quat.rotate(Quaternion(x=0, y=0, z=0, w=0), Vector3(1, 0, 0)),
            ErrorCode.INVALID_QUATERNION,
        ),
        (
            lambda: tf.normalize(
                Transform(Vector3(1, 2, 3), Quaternion(x=0, y=0, z=0, w=0))
            ),
            ErrorCode.INVALID_QUATERNION,
        ),
    ],
)
def test_degenerate_input_raises_rather_than_defaulting(call, code):
    with pytest.raises(GeometryError) as caught:
        call()
    assert caught.value.code is code


def test_a_non_finite_quaternion_is_rejected():
    with pytest.raises(GeometryError):
        quat.normalize(Quaternion(x=0, y=0, z=0, w=float("nan")))
    with pytest.raises(GeometryError):
        quat.normalize(Quaternion(x=float("inf"), y=0, z=0, w=1.0))


def test_from_matrix_rejects_a_wrong_length_sequence():
    with pytest.raises(ValueError, match="9 matrix components"):
        quat.from_matrix([1, 0, 0, 0, 1, 0])


def test_from_matrix_rejects_a_scaled_matrix():
    scaled = [
        component * 2.0 for component in quat.to_matrix(quat.from_rpy(0.1, 0.2, 0.3))
    ]
    with pytest.raises(GeometryError) as caught:
        quat.from_matrix(scaled)
    assert caught.value.code is ErrorCode.INVALID_ROTATION_MATRIX


# --- lenient adapters -------------------------------------------------------


def test_compat_falls_back_to_identity_where_the_core_raises():
    assert compat.normalize_quaternion_or_identity(
        {"w": 0, "x": 0, "y": 0, "z": 0}
    ) == (Quaternion.identity())
    assert compat.normalize_quaternion_or_identity(None) == Quaternion.identity()
    assert compat.normalize_quaternion_or_identity("nonsense") == Quaternion.identity()


def test_compat_normalizes_euler_shaped_garbage_rather_than_scaling_by_it():
    # The historical failure: Euler angles stored under x/y/z read as a
    # quaternion, whose non-unit norm silently scales every rotated vector.
    stored = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 1.57}
    normalized = compat.normalize_quaternion_or_identity(stored)
    assert math.isclose(normalized.norm, 1.0, abs_tol=1e-12)


def test_compat_reads_a_four_element_list_as_wxyz():
    assert compat.coerce_quaternion([1.0, 0.0, 0.0, 0.0]) == Quaternion.identity()
    assert compat.coerce_quaternion([0.0, 1.0, 0.0, 0.0]) == Quaternion(
        x=1.0, y=0.0, z=0.0, w=0.0
    )


def test_compat_wire_order_must_be_named():
    numbers = [0.1, 0.2, 0.3, 0.4]
    assert compat.quaternion_from_wire(numbers, order="wxyz").w == 0.1
    assert compat.quaternion_from_wire(numbers, order="xyzw").w == 0.4
    with pytest.raises(ValueError, match="wxyz"):
        compat.quaternion_from_wire(numbers, order="whatever")


def test_compat_wire_reads_any_four_long_sequence():
    """A numpy array is the shape MuJoCo actually hands you.

    Narrowing this to list/tuple made ``data.xquat[i]`` read as the identity
    with no error at all, which is precisely the silent-wrong-answer this
    helper exists to prevent.
    """
    numpy = pytest.importorskip("numpy")

    array = numpy.array([0.1, 0.2, 0.3, 0.4])
    assert compat.quaternion_from_wire(array, order="wxyz").w == pytest.approx(0.1)
    assert compat.quaternion_from_wire(array, order="xyzw").w == pytest.approx(0.4)

    # A sequence of the wrong length, and a string, still fall back rather than
    # producing a half-read quaternion.
    assert compat.quaternion_from_wire(numpy.array([1.0, 0.0])) == Quaternion.identity()
    assert compat.quaternion_from_wire("wxyz") == Quaternion.identity()


def test_compat_coerce_vector_tolerates_missing_keys():
    assert compat.coerce_vector({"x": 1.0}) == Vector3(1.0, 0.0, 0.0)
    assert compat.coerce_vector(None) == Vector3()
    assert compat.coerce_vector([1, 2, 3]) == Vector3(1.0, 2.0, 3.0)


def test_compat_rejects_non_finite_numbers_as_defaults():
    assert compat.coerce_vector({"x": float("nan"), "y": 2.0}) == Vector3(0.0, 2.0, 0.0)


# --- joint types ------------------------------------------------------------


def test_joint_type_parsing_matches_the_core():
    assert joint_type_from_string(" Revolute ") is JointType.REVOLUTE
    assert joint_type_from_string("PRISMATIC") is JointType.PRISMATIC
    assert joint_type_from_string("floating") is JointType.UNSUPPORTED
    assert joint_type_from_string("") is JointType.UNSUPPORTED
    assert JointType.REVOLUTE.is_actuated
    assert not JointType.FIXED.is_actuated


# --- trees and poses --------------------------------------------------------


def test_tree_exposes_frames_and_base():
    tree = RobotTree(two_link_arm())
    assert tree.errors == ()
    assert tree.base_frame == "base_link"
    assert tree.frames == ("base_link", "forearm", "upper_arm", "wrist_camera")
    assert tree.is_usable
    assert tree.is_articulated


def test_frame_pose_composes_the_chain():
    tree = RobotTree(two_link_arm())
    pose = tree.frame_pose(
        "forearm", joint_positions={"shoulder": math.pi / 2, "elbow": 0.0}
    )
    assert pose.available
    assert math.isclose(pose.transform.translation.y, 1.0, abs_tol=1e-12)
    assert math.isclose(pose.transform.translation.z, 1.0, abs_tol=1e-12)


def test_missing_joint_state_is_reported_not_guessed():
    tree = RobotTree(two_link_arm())
    pose = tree.frame_pose("forearm", joint_positions={"shoulder": 0.3})
    assert not pose.available
    assert pose.transform is None
    assert pose.missing_joints == ("elbow",)


def test_an_unknown_frame_raises():
    tree = RobotTree(two_link_arm())
    with pytest.raises(GeometryError) as caught:
        tree.frame_pose("nope")
    assert caught.value.code is ErrorCode.UNKNOWN_FRAME
    assert caught.value.subject == "nope"


def test_frame_poses_returns_failures_instead_of_aborting_the_batch():
    tree = RobotTree(two_link_arm())
    results = tree.frame_poses(
        ["base_link", "forearm", "nope"],
        joint_positions={"shoulder": 0.0, "elbow": 0.0},
    )
    assert results["base_link"].available
    assert results["forearm"].available
    assert isinstance(results["nope"], GeometryError)


def test_a_mimic_joint_needs_its_source():
    description = RobotDescription(
        links=["palm", "left_finger", "right_finger"],
        joints=[
            JointDescription(
                "left_finger_joint",
                "palm",
                "left_finger",
                JointType.PRISMATIC,
                axis=Vector3(1, 0, 0),
            ),
            JointDescription(
                "right_finger_joint",
                "palm",
                "right_finger",
                JointType.PRISMATIC,
                axis=Vector3(1, 0, 0),
                mimic=MimicSpec("left_finger_joint", -1.0, 0.01),
            ),
        ],
    )
    tree = RobotTree(description)
    pose = tree.frame_pose("right_finger", joint_positions={"left_finger_joint": 0.02})
    assert pose.available
    assert math.isclose(pose.transform.translation.x, -0.01, abs_tol=1e-12)

    unsatisfied = tree.frame_pose(
        "right_finger", joint_positions={"right_finger_joint": 0.02}
    )
    assert not unsatisfied.available
    assert unsatisfied.missing_joints == ("left_finger_joint",)


def test_a_degenerate_joint_origin_is_collected_not_raised():
    description = two_link_arm()
    broken = list(description.joints)
    broken[1] = JointDescription(
        name="elbow",
        parent_link="upper_arm",
        child_link="forearm",
        type=JointType.REVOLUTE,
        origin=Transform(Vector3(1, 0, 0), Quaternion(x=0, y=0, z=0, w=0)),
        axis=Vector3(0, 1, 0),
    )
    tree = RobotTree(RobotDescription(description.links, broken, description.sensors))
    codes = [error.code for error in tree.errors]
    assert ErrorCode.INVALID_JOINT_POSE in codes
    # The rest of the robot still resolves.
    assert tree.frame_pose("upper_arm", joint_positions={"shoulder": 0.0}).available


def test_the_base_transform_is_applied():
    tree = RobotTree(two_link_arm())
    base = Transform(Vector3(10, 0, 0), quat.from_yaw(math.pi))
    positions = {"shoulder": 0.0, "elbow": 0.0}
    at_origin = tree.frame_pose("forearm", joint_positions=positions)
    at_base = tree.frame_pose("forearm", base=base, joint_positions=positions)
    expected = tf.compose(base, at_origin.transform)
    assert math.isclose(
        at_base.transform.translation.x, expected.translation.x, abs_tol=1e-12
    )
    assert math.isclose(
        at_base.transform.translation.y, expected.translation.y, abs_tol=1e-12
    )


# --- lifetime ---------------------------------------------------------------


def test_a_tree_survives_the_description_it_was_built_from():
    # The core copies what it needs, so dropping the description must not
    # invalidate the tree. Getting this wrong would read freed memory.
    tree = RobotTree(two_link_arm())
    gc.collect()
    assert tree.frame_pose("base_link").available


def test_many_trees_and_poses_do_not_leak_handles():
    # Not a memory assertion -- a crash or a corrupted result under repetition
    # is what an unbalanced destroy actually looks like from Python.
    for _ in range(200):
        tree = RobotTree(two_link_arm())
        pose = tree.frame_pose(
            "forearm", joint_positions={"shoulder": 0.1, "elbow": 0.2}
        )
        assert pose.available
    gc.collect()


def test_core_version_is_reported():
    assert core_version().count(".") == 2


def test_compat_passes_an_already_built_quaternion_through():
    # A caller that resolved xyzw-vs-wxyz itself should not have to round-trip
    # through a dict to reach the lenient normalization.
    original = Quaternion.from_xyzw([0.0, 0.0, 0.3826834, 0.9238795])
    assert compat.coerce_quaternion(original) is original
    normalized = compat.normalize_quaternion_or_identity(original)
    assert math.isclose(normalized.norm, 1.0, abs_tol=1e-12)


@pytest.mark.parametrize("pitch", [math.pi / 2 - 1e-7, -math.pi / 2 + 1e-7])
def test_to_rpy_keeps_the_roll_of_an_attitude_merely_near_the_pole(pitch):
    """1e-7 rad off vertical is not gimbal lock, and must not be read as it.

    Not in the golden suite: float32 cannot represent this roll at all, so it is
    pinned per language instead. See the note in ``tests/golden_ops.cpp``.
    """
    original = quat.from_rpy(0.7, pitch, -1.2)
    recovered = quat.to_rpy(original)
    assert recovered.roll == pytest.approx(0.7, abs=1e-8)
    assert recovered.pitch == pytest.approx(pitch, abs=1e-8)
    assert recovered.yaw == pytest.approx(-1.2, abs=1e-8)
    # to_yaw promises to equal to_rpy(q).yaw everywhere, this band included.
    assert quat.to_yaw(original) == pytest.approx(recovered.yaw, abs=1e-12)
