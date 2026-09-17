"""Rigid transform composition and inversion, delegated to the shared core."""

from __future__ import annotations

from ctypes import byref

from ._native import CQuat, CTransform, CVec3, check, lib
from .types import Quaternion, Transform, Vector3

__all__ = ["apply", "compose", "inverse", "inverse_unit", "normalize", "relative"]


def _c(t: Transform) -> CTransform:
    return CTransform(
        CVec3(t.translation.x, t.translation.y, t.translation.z),
        CQuat(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w),
    )


def _py(t: CTransform) -> Transform:
    return Transform(
        Vector3(t.translation.x, t.translation.y, t.translation.z),
        Quaternion(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w),
    )


def normalize(t: Transform) -> Transform:
    """Normalize the rotation, or raise. Use at every unvalidated boundary."""
    out = CTransform()
    check(lib.cw_geom_transform_normalize(_c(t), byref(out)))
    return _py(out)


def compose(parent: Transform, child: Transform) -> Transform:
    """``parent`` applied to ``child``. Assumes both rotations are normalized."""
    return _py(lib.cw_geom_transform_compose(_c(parent), _c(child)))


def inverse_unit(t: Transform) -> Transform:
    """Assumes ``t.rotation`` is normalized."""
    return _py(lib.cw_geom_transform_inverse_unit(_c(t)))


def inverse(t: Transform) -> Transform:
    out = CTransform()
    check(lib.cw_geom_transform_inverse(_c(t), byref(out)))
    return _py(out)


def apply(t: Transform, point: Vector3) -> Vector3:
    """Map a point through the transform: ``R*p + t``."""
    result = lib.cw_geom_transform_apply(_c(t), CVec3(point.x, point.y, point.z))
    return Vector3(result.x, result.y, result.z)


def relative(parent: Transform, child: Transform) -> Transform:
    """Express a world-frame ``child`` in ``parent``'s frame -- inverse of compose."""
    return _py(lib.cw_geom_transform_relative(_c(parent), _c(child)))
