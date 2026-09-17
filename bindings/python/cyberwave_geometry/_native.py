"""Loading and calling the shared geometry core through its stable C ABI.

ctypes rather than a compiled extension module, deliberately: the core is the
single source of truth either way, and this keeps `pip install` working on any
platform without a wheel matrix or a compiler, which is what makes migrating
the backend, the SDK and the edge nodes onto it tractable. The C ABI is the
seam a compiled accelerator would slot into later without changing anything
above this module.
"""

from __future__ import annotations

import ctypes
import os
import sys
from ctypes import POINTER, c_char_p, c_double, c_int32, c_size_t, c_void_p
from pathlib import Path

from .types import ErrorCode, GeometryError

__all__ = [
    "CGeoAnchor",
    "CGeoPose",
    "CGeodetic",
    "CQuat",
    "CTransform",
    "CVec3",
    "check",
    "lib",
    "library_path",
]


class CVec3(ctypes.Structure):
    _fields_ = [("x", c_double), ("y", c_double), ("z", c_double)]


class CQuat(ctypes.Structure):
    # Field order mirrors ``cw_geom_quat`` in c_api.h: x, y, z, w.
    _fields_ = [("x", c_double), ("y", c_double), ("z", c_double), ("w", c_double)]


class CTransform(ctypes.Structure):
    _fields_ = [("translation", CVec3), ("rotation", CQuat)]


class CGeodetic(ctypes.Structure):
    # Degrees, degrees, metres -- mirrors ``cw_geom_geodetic`` in c_api.h.
    _fields_ = [
        ("latitude_deg", c_double),
        ("longitude_deg", c_double),
        ("altitude_m", c_double),
    ]


class CGeoPose(ctypes.Structure):
    _fields_ = [("position", CGeodetic), ("orientation", CQuat)]


class CGeoAnchor(ctypes.Structure):
    _fields_ = [("origin", CGeodetic), ("heading_deg", c_double)]


def _candidate_paths() -> list[Path]:
    """Where to look for the shared library, most specific first."""
    candidates: list[Path] = []

    override = os.environ.get("CYBERWAVE_GEOMETRY_LIBRARY")
    if override:
        candidates.append(Path(override))

    if sys.platform == "darwin":
        names = ["libcyberwave_geometry_c.dylib"]
    elif sys.platform == "win32":
        names = ["cyberwave_geometry_c.dll"]
    else:
        names = ["libcyberwave_geometry_c.so"]

    # Bundled beside the package: how a built wheel ships it.
    package_dir = Path(__file__).resolve().parent
    for name in names:
        candidates.append(package_dir / name)
        candidates.append(package_dir / "_lib" / name)

    # A developer build tree, so the binding is usable straight after `cmake
    # --build` without an install step.
    build_root = os.environ.get("CYBERWAVE_GEOMETRY_BUILD_DIR")
    if build_root:
        for name in names:
            candidates.append(Path(build_root) / name)

    # Bare name, letting the platform loader search its usual paths.
    candidates.extend(Path(name) for name in names)
    return candidates


def _load() -> tuple[ctypes.CDLL, Path]:
    attempted: list[str] = []
    for candidate in _candidate_paths():
        try:
            return ctypes.CDLL(str(candidate)), candidate
        except OSError as exc:  # pragma: no cover - platform dependent
            attempted.append(f"{candidate}: {exc}")
    raise ImportError(
        "cyberwave_geometry could not load the shared geometry core.\n"
        "Build it with:\n"
        "    cmake -S common/geometry -B build && cmake --build build\n"
        "then either install it, copy the library next to this package, or set\n"
        "CYBERWAVE_GEOMETRY_LIBRARY to its path.\n"
        "Tried:\n  " + "\n  ".join(attempted)
    )


lib, library_path = _load()


def _declare() -> None:
    """Pin every signature.

    Without argtypes ctypes guesses, and a double silently truncated to an int
    is the kind of bug that shows up as a robot pointing the wrong way rather
    than as an exception.
    """
    signatures: list[tuple[str, list, object]] = [
        ("cw_geom_version", [], c_char_p),
        ("cw_geom_version_parts", [POINTER(c_int32)] * 3, None),
        ("cw_geom_error_name", [c_int32], c_char_p),
        # quaternion
        ("cw_geom_quat_identity", [], CQuat),
        ("cw_geom_quat_norm", [CQuat], c_double),
        ("cw_geom_quat_dot", [CQuat, CQuat], c_double),
        ("cw_geom_quat_normalize", [CQuat, POINTER(CQuat)], c_int32),
        ("cw_geom_quat_multiply", [CQuat, CQuat], CQuat),
        ("cw_geom_quat_conjugate", [CQuat], CQuat),
        ("cw_geom_quat_inverse", [CQuat, POINTER(CQuat)], c_int32),
        ("cw_geom_quat_rotate_unit", [CQuat, CVec3], CVec3),
        ("cw_geom_quat_rotate", [CQuat, CVec3, POINTER(CVec3)], c_int32),
        ("cw_geom_quat_from_axis_angle", [CVec3, c_double, POINTER(CQuat)], c_int32),
        (
            "cw_geom_quat_to_axis_angle",
            [CQuat, POINTER(CVec3), POINTER(c_double)],
            c_int32,
        ),
        ("cw_geom_quat_slerp", [CQuat, CQuat, c_double, POINTER(CQuat)], c_int32),
        ("cw_geom_quat_nlerp", [CQuat, CQuat, c_double, POINTER(CQuat)], c_int32),
        ("cw_geom_quat_from_rpy", [c_double, c_double, c_double], CQuat),
        (
            "cw_geom_quat_to_rpy",
            [CQuat, POINTER(c_double), POINTER(c_double), POINTER(c_double)],
            c_int32,
        ),
        ("cw_geom_quat_from_yaw", [c_double], CQuat),
        ("cw_geom_quat_to_yaw", [CQuat, POINTER(c_double)], c_int32),
        ("cw_geom_quat_to_matrix", [CQuat, POINTER(c_double)], c_int32),
        (
            "cw_geom_quat_from_matrix",
            [POINTER(c_double), c_double, POINTER(CQuat)],
            c_int32,
        ),
        # geodetic
        ("cw_geom_geo_validate", [CGeodetic], c_int32),
        ("cw_geom_geo_validate_anchor", [CGeoAnchor], c_int32),
        ("cw_geom_geo_normalize_longitude_deg", [c_double], c_double),
        ("cw_geom_geo_normalize_bearing_deg", [c_double], c_double),
        (
            "cw_geom_geo_metres_per_degree",
            [c_double, POINTER(c_double), POINTER(c_double)],
            c_int32,
        ),
        ("cw_geom_geo_enu_between", [CGeodetic, CGeodetic, POINTER(CVec3)], c_int32),
        ("cw_geom_geo_offset", [CGeodetic, CVec3, POINTER(CGeodetic)], c_int32),
        (
            "cw_geom_geo_ground_distance",
            [CGeodetic, CGeodetic, POINTER(c_double)],
            c_int32,
        ),
        ("cw_geom_geo_enu_to_local_rotation", [CGeoAnchor, POINTER(CQuat)], c_int32),
        ("cw_geom_geo_to_local", [CGeoAnchor, CGeodetic, POINTER(CVec3)], c_int32),
        ("cw_geom_geo_from_local", [CGeoAnchor, CVec3, POINTER(CGeodetic)], c_int32),
        (
            "cw_geom_geo_to_local_pose",
            [CGeoAnchor, CGeoPose, POINTER(CTransform)],
            c_int32,
        ),
        (
            "cw_geom_geo_from_local_pose",
            [CGeoAnchor, CTransform, POINTER(CGeoPose)],
            c_int32,
        ),
        ("cw_geom_geo_quat_from_compass_heading", [c_double, POINTER(CQuat)], c_int32),
        ("cw_geom_geo_to_compass_heading_deg", [CQuat, POINTER(c_double)], c_int32),
        (
            "cw_geom_geo_quat_from_ned_rpy",
            [c_double, c_double, c_double, POINTER(CQuat)],
            c_int32,
        ),
        (
            "cw_geom_geo_to_ned_rpy",
            [CQuat, POINTER(c_double), POINTER(c_double), POINTER(c_double)],
            c_int32,
        ),
        # transform
        ("cw_geom_transform_identity", [], CTransform),
        ("cw_geom_transform_normalize", [CTransform, POINTER(CTransform)], c_int32),
        ("cw_geom_transform_compose", [CTransform, CTransform], CTransform),
        ("cw_geom_transform_inverse_unit", [CTransform], CTransform),
        ("cw_geom_transform_inverse", [CTransform, POINTER(CTransform)], c_int32),
        ("cw_geom_transform_apply", [CTransform, CVec3], CVec3),
        ("cw_geom_transform_relative", [CTransform, CTransform], CTransform),
        # fk
        ("cw_geom_joint_type_from_string", [c_char_p], c_int32),
        ("cw_geom_joint_type_name", [c_int32], c_char_p),
        ("cw_geom_builder_create", [], c_void_p),
        ("cw_geom_builder_destroy", [c_void_p], None),
        ("cw_geom_builder_add_link", [c_void_p, c_char_p], None),
        (
            "cw_geom_builder_add_joint",
            [
                c_void_p,
                c_char_p,
                c_char_p,
                c_char_p,
                c_int32,
                CTransform,
                CVec3,
                c_char_p,
                c_double,
                c_double,
            ],
            None,
        ),
        (
            "cw_geom_builder_add_sensor",
            [c_void_p, c_char_p, c_char_p, CTransform],
            None,
        ),
        ("cw_geom_tree_build", [c_void_p], c_void_p),
        ("cw_geom_tree_destroy", [c_void_p], None),
        ("cw_geom_tree_root_link", [c_void_p], c_char_p),
        ("cw_geom_tree_base_frame", [c_void_p], c_char_p),
        ("cw_geom_tree_is_usable", [c_void_p], c_int32),
        ("cw_geom_tree_is_articulated", [c_void_p], c_int32),
        ("cw_geom_tree_frame_count", [c_void_p], c_size_t),
        ("cw_geom_tree_frame_name", [c_void_p, c_size_t], c_char_p),
        ("cw_geom_tree_error_count", [c_void_p], c_size_t),
        (
            "cw_geom_tree_error",
            [c_void_p, c_size_t, POINTER(c_char_p), POINTER(c_char_p)],
            c_int32,
        ),
        (
            "cw_geom_compute_frame_pose",
            [
                c_void_p,
                c_char_p,
                CTransform,
                POINTER(c_char_p),
                POINTER(c_double),
                c_size_t,
            ],
            c_void_p,
        ),
        ("cw_geom_pose_destroy", [c_void_p], None),
        ("cw_geom_pose_status", [c_void_p], c_int32),
        ("cw_geom_pose_error_subject", [c_void_p], c_char_p),
        ("cw_geom_pose_error_detail", [c_void_p], c_char_p),
        ("cw_geom_pose_available", [c_void_p], c_int32),
        ("cw_geom_pose_transform", [c_void_p], CTransform),
        ("cw_geom_pose_missing_count", [c_void_p], c_size_t),
        ("cw_geom_pose_missing_joint", [c_void_p, c_size_t], c_char_p),
    ]
    for name, argtypes, restype in signatures:
        function = getattr(lib, name)
        function.argtypes = argtypes
        function.restype = restype


_declare()


#: Core version this binding was conformance-tested against. The core's own
#: kVersionMajor/kVersionMinor must match: a mismatch means a stale artifact --
#: a wheel or a Docker layer built either side of a rename -- and failing here
#: beats failing at the call site with an AttributeError.
REQUIRED_CORE_VERSION = (0, 2)


def _check_core_version() -> None:
    major = c_int32()
    minor = c_int32()
    patch = c_int32()
    lib.cw_geom_version_parts(major, minor, patch)
    loaded = (major.value, minor.value)
    if loaded != REQUIRED_CORE_VERSION:
        raise ImportError(
            f"cyberwave_geometry needs geometry core "
            f"{REQUIRED_CORE_VERSION[0]}.{REQUIRED_CORE_VERSION[1]}.x, but "
            f"{library_path} is {major.value}.{minor.value}.{patch.value}.\n"
            "Rebuild it with:\n"
            "    cmake -S common/geometry -B build && cmake --build build\n"
            "or point CYBERWAVE_GEOMETRY_LIBRARY at a matching build."
        )


_check_core_version()


def check(status: int, subject: str = "", detail: str = "") -> None:
    """Turn a non-zero ABI status into a :class:`GeometryError`.

    The value entry points carry no detail string across the ABI -- the code is
    the whole message -- so `detail` stays empty unless a caller supplies
    something the code does not already say.
    """
    if status == 0:
        return
    try:
        code: ErrorCode | int = ErrorCode(status)
    except ValueError:
        # A code the core has and this enum does not. Name it from the core
        # rather than letting ValueError escape from ErrorCode().
        code = status
        detail = detail or (lib.cw_geom_error_name(status) or b"").decode()
    raise GeometryError(code, subject, detail)
