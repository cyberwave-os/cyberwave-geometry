"""Quaternion algebra, delegated to the shared core.

Every function here is strict in the same way the C++ core is: a degenerate or
non-finite input raises :class:`GeometryError` rather than quietly becoming an
identity rotation. The lenient behaviour some existing call sites rely on lives
in :mod:`cyberwave_geometry.compat`, where it is visible.
"""

from __future__ import annotations

from collections.abc import Sequence
from ctypes import byref, c_double

from ._native import CQuat, CVec3, check, lib
from .types import AxisAngle, Quaternion, Rpy, Vector3

__all__ = [
    "conjugate",
    "dot",
    "from_axis_angle",
    "from_matrix",
    "from_rpy",
    "from_yaw",
    "inverse",
    "multiply",
    "nlerp",
    "norm",
    "normalize",
    "rotate",
    "rotate_unit",
    "slerp",
    "to_axis_angle",
    "to_matrix",
    "to_rpy",
    "to_yaw",
]


def _c(q: Quaternion) -> CQuat:
    return CQuat(x=q.x, y=q.y, z=q.z, w=q.w)


def _py(q: CQuat) -> Quaternion:
    return Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)


def _cv(v: Vector3) -> CVec3:
    return CVec3(v.x, v.y, v.z)


def _pyv(v: CVec3) -> Vector3:
    return Vector3(v.x, v.y, v.z)


def norm(q: Quaternion) -> float:
    return lib.cw_geom_quat_norm(_c(q))


def dot(a: Quaternion, b: Quaternion) -> float:
    return lib.cw_geom_quat_dot(_c(a), _c(b))


def normalize(q: Quaternion) -> Quaternion:
    out = CQuat()
    check(lib.cw_geom_quat_normalize(_c(q), byref(out)))
    return _py(out)


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """Hamilton product: the rotation that applies ``b`` first, then ``a``."""
    return _py(lib.cw_geom_quat_multiply(_c(a), _c(b)))


def conjugate(q: Quaternion) -> Quaternion:
    return _py(lib.cw_geom_quat_conjugate(_c(q)))


def inverse(q: Quaternion) -> Quaternion:
    out = CQuat()
    check(lib.cw_geom_quat_inverse(_c(q), byref(out)))
    return _py(out)


def rotate_unit(q: Quaternion, v: Vector3) -> Vector3:
    """Rotate by an already-normalized quaternion. The precondition is not
    checked; use :func:`rotate` for unvalidated input."""
    return _pyv(lib.cw_geom_quat_rotate_unit(_c(q), _cv(v)))


def rotate(q: Quaternion, v: Vector3) -> Vector3:
    out = CVec3()
    check(lib.cw_geom_quat_rotate(_c(q), _cv(v), byref(out)))
    return _pyv(out)


def slerp(a: Quaternion, b: Quaternion, t: float) -> Quaternion:
    """Spherical linear interpolation along the shortest arc.

    ``b`` is flipped first when the two point into opposite hemispheres, so a
    pair stored as antipodes interpolates the short way rather than spinning
    almost all the way round. ``t`` is not clamped: outside ``[0, 1]`` this
    extrapolates.
    """
    out = CQuat()
    check(lib.cw_geom_quat_slerp(_c(a), _c(b), float(t), byref(out)))
    return _py(out)


def nlerp(a: Quaternion, b: Quaternion, t: float) -> Quaternion:
    """Normalized linear interpolation, also along the shortest arc.

    Cheaper than :func:`slerp`, at the cost of non-constant angular velocity.
    """
    out = CQuat()
    check(lib.cw_geom_quat_nlerp(_c(a), _c(b), float(t), byref(out)))
    return _py(out)


def from_axis_angle(axis: Vector3, angle: float) -> Quaternion:
    """The axis is normalized here, so a URDF axis may be passed verbatim."""
    out = CQuat()
    check(lib.cw_geom_quat_from_axis_angle(_cv(axis), float(angle), byref(out)))
    return _py(out)


def to_axis_angle(q: Quaternion) -> AxisAngle:
    axis = CVec3()
    angle = c_double()
    check(lib.cw_geom_quat_to_axis_angle(_c(q), byref(axis), byref(angle)))
    return AxisAngle(_pyv(axis), angle.value)


def from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Fixed-axis XYZ roll/pitch/yaw -- the URDF and ROS convention.

    This is *not* Three.js's default Euler order; see CONVENTIONS.md.
    """
    return _py(lib.cw_geom_quat_from_rpy(float(roll), float(pitch), float(yaw)))


def to_rpy(q: Quaternion) -> Rpy:
    roll = c_double()
    pitch = c_double()
    yaw = c_double()
    check(lib.cw_geom_quat_to_rpy(_c(q), byref(roll), byref(pitch), byref(yaw)))
    return Rpy(roll.value, pitch.value, yaw.value)


def from_yaw(yaw: float) -> Quaternion:
    """Rotation about +Z only."""
    return _py(lib.cw_geom_quat_from_yaw(float(yaw)))


def to_yaw(q: Quaternion) -> float:
    """Rotation about +Z extracted from a full rotation.

    Exactly ``to_rpy(q).yaw``, including at gimbal lock.
    """
    out = c_double()
    check(lib.cw_geom_quat_to_yaw(_c(q), byref(out)))
    return out.value


def to_matrix(q: Quaternion) -> tuple[float, ...]:
    """Row-major 3x3 rotation matrix as nine floats."""
    buffer = (c_double * 9)()
    check(lib.cw_geom_quat_to_matrix(_c(q), buffer))
    return tuple(buffer)


def from_matrix(matrix: Sequence[float], tolerance: float = 1e-6) -> Quaternion:
    """Inverse of :func:`to_matrix`.

    Rejects anything that is not a right-handed orthonormal matrix within
    ``tolerance``: a scaled or mirrored matrix reaching here is an upstream bug,
    not a rotation to be approximated.
    """
    values = list(matrix)
    if len(values) != 9:
        raise ValueError(f"expected 9 matrix components, got {len(values)}")
    buffer = (c_double * 9)(*[float(value) for value in values])
    out = CQuat()
    check(lib.cw_geom_quat_from_matrix(buffer, float(tolerance), byref(out)))
    return _py(out)
