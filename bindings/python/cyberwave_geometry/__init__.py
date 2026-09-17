"""Python binding for the Cyberwave shared geometry core.

The C++ library under ``common/geometry`` is the single executable source of
truth for scalar quaternion, transform and forward-kinematics arithmetic. This
package is a thin, strict wrapper over its stable C ABI -- it holds no formulas
of its own, by design. Adding one here would recreate exactly the duplication
this library exists to remove.

    >>> from cyberwave_geometry import Quaternion, quaternion as quat
    >>> quat.to_yaw(quat.from_yaw(0.5))
    0.5

Conventions (Hamilton products, fixed-axis XYZ roll/pitch/yaw, URDF joint
semantics, and the ENU-referenced geodetic pose in ``geodetic``) are documented
in ``common/geometry/CONVENTIONS.md``.
"""

from __future__ import annotations

from . import compat, fk, geodetic, quaternion, transform
from ._native import library_path
from .types import (
    AxisAngle,
    ErrorCode,
    GeoAnchor,
    Geodetic,
    GeometryError,
    GeoPose,
    JointType,
    MetresPerDegree,
    Quaternion,
    Rpy,
    Transform,
    Vector3,
)

__all__ = [
    "AxisAngle",
    "ErrorCode",
    "GeoAnchor",
    "GeoPose",
    "Geodetic",
    "GeometryError",
    "JointType",
    "MetresPerDegree",
    "Quaternion",
    "Rpy",
    "Transform",
    "Vector3",
    "compat",
    "core_version",
    "fk",
    "geodetic",
    "library_path",
    "quaternion",
    "transform",
]


def core_version() -> str:
    """Version of the loaded C++ core, not of this Python package."""
    from ._native import lib

    return lib.cw_geom_version().decode()


__version__ = "0.2.0"
