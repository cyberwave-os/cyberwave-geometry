"""Forward kinematics over a parsed robot description.

Schema parsing stays with the caller: the backend's Universal Robot Schema
reader, the SDK's URDF loader and an edge driver's own description each build a
:class:`RobotDescription` and hand it here, so the format quirks stay where
they belong while the traversal and the arithmetic happen once, in the shared
core, for every language.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from ctypes import c_char_p, c_double
from dataclasses import dataclass, field

from ._native import CQuat, CTransform, CVec3, lib
from .types import ErrorCode, GeometryError, JointType, Quaternion, Transform, Vector3

__all__ = [
    "FramePose",
    "JointDescription",
    "MimicSpec",
    "RobotDescription",
    "RobotTree",
    "SensorDescription",
    "TreeError",
    "joint_type_from_string",
]


def joint_type_from_string(text: str) -> JointType:
    """Case- and whitespace-insensitive.

    An unrecognized or empty string is :attr:`JointType.UNSUPPORTED`, never
    ``FIXED``: supplying URDF's default for an *absent* type is the schema
    reader's job, and silently freezing an unknown joint would publish a
    confidently wrong pose.
    """
    return JointType(lib.cw_geom_joint_type_from_string((text or "").encode()))


@dataclass(frozen=True)
class MimicSpec:
    """A joint whose position is a linear function of another joint's."""

    source_joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class JointDescription:
    name: str
    parent_link: str
    child_link: str
    type: JointType = JointType.FIXED
    #: Raw: the tree normalizes it and reports the ones it cannot.
    origin: Transform = field(default_factory=Transform)
    #: URDF's default joint axis.
    axis: Vector3 = field(default_factory=lambda: Vector3(0.0, 0.0, 1.0))
    mimic: MimicSpec | None = None


@dataclass(frozen=True)
class SensorDescription:
    """A frame rigidly attached to a link: a camera, an IMU, a tool tip."""

    name: str
    parent_link: str
    extrinsic: Transform = field(default_factory=Transform)


@dataclass(frozen=True)
class RobotDescription:
    links: Sequence[str] = ()
    joints: Sequence[JointDescription] = ()
    sensors: Sequence[SensorDescription] = ()


@dataclass(frozen=True)
class TreeError:
    code: ErrorCode
    subject: str
    detail: str

    def as_payload(self) -> dict[str, str]:
        return {
            "code": self.code.name.lower(),
            "subject": self.subject,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class FramePose:
    """The result of asking for one frame's world pose.

    Three outcomes, kept distinct on purpose: a pose, a list of joints whose
    state has not been reported yet, or -- as a raised
    :class:`GeometryError` -- a structural problem with the description.
    """

    available: bool
    transform: Transform | None = None
    missing_joints: tuple[str, ...] = ()


def _c_transform(t: Transform) -> CTransform:
    return CTransform(
        CVec3(t.translation.x, t.translation.y, t.translation.z),
        CQuat(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w),
    )


def _py_transform(t: CTransform) -> Transform:
    return Transform(
        Vector3(t.translation.x, t.translation.y, t.translation.z),
        Quaternion(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w),
    )


class RobotTree:
    """A validated kinematic view of one robot description.

    Owns a handle into the shared core. Structural problems are collected in
    :attr:`errors` rather than raised: a robot whose wrist camera is
    unreachable must still report its base pose, so the caller decides per
    frame what is unavailable.
    """

    __slots__ = (
        "_articulated",
        "_base_frame",
        "_errors",
        "_frames",
        "_handle",
        "_root_link",
    )

    def __init__(self, description: RobotDescription) -> None:
        builder = lib.cw_geom_builder_create()
        if not builder:
            raise MemoryError("cw_geom_builder_create failed")
        try:
            for link in description.links:
                lib.cw_geom_builder_add_link(builder, str(link).encode())
            for joint in description.joints:
                mimic = joint.mimic
                lib.cw_geom_builder_add_joint(
                    builder,
                    joint.name.encode(),
                    joint.parent_link.encode(),
                    joint.child_link.encode(),
                    int(joint.type),
                    _c_transform(joint.origin),
                    CVec3(joint.axis.x, joint.axis.y, joint.axis.z),
                    mimic.source_joint.encode() if mimic else None,
                    mimic.multiplier if mimic else 1.0,
                    mimic.offset if mimic else 0.0,
                )
            for sensor in description.sensors:
                lib.cw_geom_builder_add_sensor(
                    builder,
                    sensor.name.encode(),
                    sensor.parent_link.encode(),
                    _c_transform(sensor.extrinsic),
                )
            handle = lib.cw_geom_tree_build(builder)
        finally:
            lib.cw_geom_builder_destroy(builder)

        if not handle:
            raise MemoryError("cw_geom_tree_build failed")
        self._handle = handle

        # Read the immutable parts across once. Every string the ABI returns
        # points into the handle's own storage, so they are copied here rather
        # than held as pointers.
        self._root_link = lib.cw_geom_tree_root_link(handle).decode()
        self._base_frame = lib.cw_geom_tree_base_frame(handle).decode()
        self._articulated = bool(lib.cw_geom_tree_is_articulated(handle))
        self._frames = tuple(
            lib.cw_geom_tree_frame_name(handle, index).decode()
            for index in range(lib.cw_geom_tree_frame_count(handle))
        )
        errors = []
        for index in range(lib.cw_geom_tree_error_count(handle)):
            subject = c_char_p()
            detail = c_char_p()
            code = lib.cw_geom_tree_error(handle, index, subject, detail)
            errors.append(
                TreeError(
                    ErrorCode(code),
                    (subject.value or b"").decode(),
                    (detail.value or b"").decode(),
                )
            )
        self._errors = tuple(errors)

    def __del__(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            lib.cw_geom_tree_destroy(handle)
            self._handle = None

    @property
    def root_link(self) -> str:
        """Topological root: a link that is never a joint's child."""
        return self._root_link

    @property
    def base_frame(self) -> str:
        """The frame the robot's own pose topics describe.

        Often not the topological root, which is frequently a synthetic
        ``world`` anchor the robot is bolted to.
        """
        return self._base_frame

    @property
    def frames(self) -> tuple[str, ...]:
        """Every selectable frame: declared links plus sensor frames, sorted."""
        return self._frames

    @property
    def errors(self) -> tuple[TreeError, ...]:
        return self._errors

    @property
    def is_usable(self) -> bool:
        return bool(self._base_frame)

    @property
    def is_articulated(self) -> bool:
        return self._articulated

    def frame_pose(
        self,
        frame_name: str,
        *,
        base: Transform | None = None,
        joint_positions: Mapping[str, float] | None = None,
    ) -> FramePose:
        """World pose of ``frame_name``.

        ``base`` is the already-world-resolved pose of :attr:`base_frame`: the
        caller applies the environment's navigation transform before calling,
        so this stays a pure function of the description and the joint state.

        Raises :class:`GeometryError` for a structural problem -- an unknown
        frame, a cycle, an unsupported joint type. Returns a
        :class:`FramePose` with ``available=False`` when the description is
        fine but the state to evaluate it has not been reported.
        """
        positions = dict(joint_positions or {})
        count = len(positions)
        names_array = (c_char_p * max(count, 1))()
        values_array = (c_double * max(count, 1))()
        for index, (name, value) in enumerate(positions.items()):
            names_array[index] = str(name).encode()
            values_array[index] = float(value)

        pose = lib.cw_geom_compute_frame_pose(
            self._handle,
            str(frame_name).encode(),
            _c_transform(base if base is not None else Transform.identity()),
            names_array if count else None,
            values_array if count else None,
            count,
        )
        if not pose:
            raise MemoryError("cw_geom_compute_frame_pose failed")
        try:
            status = lib.cw_geom_pose_status(pose)
            if status != 0:
                raise GeometryError(
                    ErrorCode(status),
                    (lib.cw_geom_pose_error_subject(pose) or b"").decode(),
                    (lib.cw_geom_pose_error_detail(pose) or b"").decode(),
                )
            if not lib.cw_geom_pose_available(pose):
                missing = tuple(
                    lib.cw_geom_pose_missing_joint(pose, index).decode()
                    for index in range(lib.cw_geom_pose_missing_count(pose))
                )
                return FramePose(available=False, missing_joints=missing)
            return FramePose(
                available=True,
                transform=_py_transform(lib.cw_geom_pose_transform(pose)),
            )
        finally:
            lib.cw_geom_pose_destroy(pose)

    def frame_poses(
        self,
        frame_names: Sequence[str],
        *,
        base: Transform | None = None,
        joint_positions: Mapping[str, float] | None = None,
    ) -> dict[str, FramePose | GeometryError]:
        """Several frames at once, one entry per requested name.

        A structural failure is *returned* here rather than raised, because a
        telemetry tick recording twenty frames should still record the nineteen
        that resolved.

        This loops on the Python side: it crosses the ABI once per frame, not
        once per call. The C++ core has a batched ``compute_frame_poses``, but
        exposing it would mean a result-array handle with its own lifetime
        rules, and ``tests/benchmark.py`` puts a full 14-frame tick at well
        under a millisecond — the FFI crossing is not what costs. Revisit only
        if a profile says otherwise.
        """
        results: dict[str, FramePose | GeometryError] = {}
        for name in frame_names:
            try:
                results[name] = self.frame_pose(
                    name, base=base, joint_positions=joint_positions
                )
            except GeometryError as error:
                results[name] = error
        return results
