"""Cross-language conformance: the Python binding against the golden vectors.

The same file the C++ `golden_gen --check` reads, and the same one the
WASM/TypeScript and Kotlin runners are expected to read. If this passes and
that passes, the two languages agree on every case to within the tolerance the
file itself declares.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from cyberwave_geometry import (
    GeoAnchor,
    Geodetic,
    GeometryError,
    GeoPose,
    Quaternion,
    Transform,
    Vector3,
)
from cyberwave_geometry import fk as fk_module
from cyberwave_geometry import geodetic as geo
from cyberwave_geometry import quaternion as quat
from cyberwave_geometry import transform as tf
from cyberwave_geometry.fk import (
    JointDescription,
    MimicSpec,
    RobotDescription,
    RobotTree,
    SensorDescription,
)

GOLDEN_PATH = Path(__file__).resolve().parents[3] / "golden" / "geometry_golden.json"


@pytest.fixture(scope="module")
def golden() -> dict:
    assert GOLDEN_PATH.exists(), (
        f"{GOLDEN_PATH} is missing; generate it with "
        "`cmake --build <build> --target golden_gen && "
        "<build>/tests/golden_gen --write common/geometry/golden/geometry_golden.json`"
    )
    return json.loads(GOLDEN_PATH.read_text())


def _vector(data: dict) -> Vector3:
    return Vector3(data["x"], data["y"], data["z"])


def _quaternion(data: dict) -> Quaternion:
    return Quaternion(x=data["x"], y=data["y"], z=data["z"], w=data["w"])


def _transform(data: dict) -> Transform:
    return Transform(_vector(data["translation"]), _quaternion(data["rotation"]))


def _geodetic(data: dict) -> Geodetic:
    return Geodetic(data["latitude"], data["longitude"], data["altitude"])


def _anchor(data: dict) -> GeoAnchor:
    return GeoAnchor(_geodetic(data), data["heading_deg"])


def _robot(description: dict) -> RobotDescription:
    joints = []
    for raw in description["joints"]:
        mimic = raw.get("mimic")
        joints.append(
            JointDescription(
                name=raw["name"],
                parent_link=raw["parent_link"],
                child_link=raw["child_link"],
                type=fk_module.joint_type_from_string(raw["type"]),
                origin=_transform(raw["origin"]),
                axis=_vector(raw["axis"]),
                mimic=(
                    MimicSpec(
                        source_joint=mimic["source_joint"],
                        multiplier=mimic.get("multiplier", 1.0),
                        offset=mimic.get("offset", 0.0),
                    )
                    if mimic
                    else None
                ),
            )
        )
    sensors = [
        SensorDescription(
            name=raw["name"],
            parent_link=raw["parent_link"],
            extrinsic=_transform(raw.get("extrinsic") or raw["pose"]),
        )
        for raw in description["sensors"]
    ]
    return RobotDescription(links=description["links"], joints=joints, sensors=sensors)


def _evaluate(robots: dict, op: str, payload: dict) -> dict:
    """Mirror of `golden::evaluate` in tests/golden_ops.cpp."""
    try:
        if op == "quat_normalize":
            return {
                "status": "ok",
                "quaternion": quat.normalize(_quaternion(payload["q"])).to_dict(),
            }
        if op == "quat_multiply":
            result = quat.multiply(_quaternion(payload["a"]), _quaternion(payload["b"]))
            return {"status": "ok", "quaternion": result.to_dict()}
        if op == "quat_conjugate":
            return {
                "status": "ok",
                "quaternion": quat.conjugate(_quaternion(payload["q"])).to_dict(),
            }
        if op == "quat_inverse":
            return {
                "status": "ok",
                "quaternion": quat.inverse(_quaternion(payload["q"])).to_dict(),
            }
        if op == "quat_rotate":
            result = quat.rotate(_quaternion(payload["q"]), _vector(payload["v"]))
            return {"status": "ok", "vector": result.to_dict()}
        if op in ("quat_slerp", "quat_nlerp"):
            interpolate = quat.slerp if op == "quat_slerp" else quat.nlerp
            result = interpolate(
                _quaternion(payload["a"]), _quaternion(payload["b"]), payload["t"]
            )
            return {"status": "ok", "quaternion": result.to_dict()}
        if op == "quat_from_axis_angle":
            result = quat.from_axis_angle(_vector(payload["axis"]), payload["angle"])
            return {"status": "ok", "quaternion": result.to_dict()}
        if op == "quat_to_axis_angle":
            result = quat.to_axis_angle(_quaternion(payload["q"]))
            return {
                "status": "ok",
                "axis": result.axis.to_dict(),
                "angle": result.angle,
            }
        if op == "quat_from_rpy":
            result = quat.from_rpy(payload["roll"], payload["pitch"], payload["yaw"])
            return {"status": "ok", "quaternion": result.to_dict()}
        if op == "quat_to_rpy":
            result = quat.to_rpy(_quaternion(payload["q"]))
            return {
                "status": "ok",
                "roll": result.roll,
                "pitch": result.pitch,
                "yaw": result.yaw,
            }
        if op == "quat_from_yaw":
            return {
                "status": "ok",
                "quaternion": quat.from_yaw(payload["yaw"]).to_dict(),
            }
        if op == "quat_to_yaw":
            return {"status": "ok", "yaw": quat.to_yaw(_quaternion(payload["q"]))}
        if op == "quat_to_matrix":
            return {
                "status": "ok",
                "matrix": list(quat.to_matrix(_quaternion(payload["q"]))),
            }
        if op == "quat_from_matrix":
            return {
                "status": "ok",
                "quaternion": quat.from_matrix(payload["matrix"]).to_dict(),
            }
        if op == "transform_normalize":
            return {
                "status": "ok",
                "transform": tf.normalize(_transform(payload["t"])).to_dict(),
            }
        if op == "transform_compose":
            result = tf.compose(
                _transform(payload["parent"]), _transform(payload["child"])
            )
            return {"status": "ok", "transform": result.to_dict()}
        if op == "transform_inverse":
            return {
                "status": "ok",
                "transform": tf.inverse(_transform(payload["t"])).to_dict(),
            }
        if op == "transform_apply":
            result = tf.apply(_transform(payload["t"]), _vector(payload["point"]))
            return {"status": "ok", "vector": result.to_dict()}
        if op == "transform_relative":
            result = tf.relative(
                _transform(payload["parent"]), _transform(payload["child"])
            )
            return {"status": "ok", "transform": result.to_dict()}
        if op == "joint_type_from_string":
            return {
                "status": "ok",
                "joint_type": fk_module.joint_type_from_string(
                    payload["text"]
                ).name.lower(),
            }
        if op == "geo_metres_per_degree":
            scale = geo.metres_per_degree(payload["latitude_deg"])
            return {
                "status": "ok",
                "metres_per_degree_latitude": scale.latitude,
                "metres_per_degree_longitude": scale.longitude,
            }
        if op == "geo_normalize_longitude_deg":
            return {
                "status": "ok",
                "longitude_deg": geo.normalize_longitude_deg(payload["longitude_deg"]),
            }
        if op == "geo_enu_between":
            result = geo.enu_between(
                _geodetic(payload["origin"]), _geodetic(payload["target"])
            )
            return {"status": "ok", "vector": result.to_dict()}
        if op == "geo_offset":
            result = geo.offset(_geodetic(payload["origin"]), _vector(payload["enu"]))
            return {"status": "ok", "geodetic": result.to_dict()}
        if op == "geo_ground_distance":
            distance = geo.ground_distance(
                _geodetic(payload["a"]), _geodetic(payload["b"])
            )
            return {"status": "ok", "distance": distance}
        if op == "geo_to_local":
            result = geo.to_local(
                _anchor(payload["anchor"]), _geodetic(payload["position"])
            )
            return {"status": "ok", "vector": result.to_dict()}
        if op == "geo_from_local":
            result = geo.from_local(
                _anchor(payload["anchor"]), _vector(payload["position"])
            )
            return {"status": "ok", "geodetic": result.to_dict()}
        if op == "geo_to_local_pose":
            pose = GeoPose(
                _geodetic(payload["pose"]), _quaternion(payload["pose"]["rotation"])
            )
            result = geo.to_local_pose(_anchor(payload["anchor"]), pose)
            return {"status": "ok", "transform": result.to_dict()}
        if op == "geo_from_local_pose":
            result = geo.from_local_pose(
                _anchor(payload["anchor"]), _transform(payload["pose"])
            )
            return {
                "status": "ok",
                "geodetic": result.position.to_dict(),
                "quaternion": result.orientation.to_dict(),
            }
        if op == "geo_quat_from_compass_heading":
            result = geo.quat_from_compass_heading(payload["heading_deg"])
            return {"status": "ok", "quaternion": result.to_dict()}
        if op == "geo_to_compass_heading_deg":
            return {
                "status": "ok",
                "heading_deg": geo.to_compass_heading_deg(_quaternion(payload["q"])),
            }
        if op == "geo_quat_from_ned_rpy":
            result = geo.quat_from_ned_rpy(
                payload["roll"], payload["pitch"], payload["yaw"]
            )
            return {"status": "ok", "quaternion": result.to_dict()}
        if op == "geo_to_ned_rpy":
            result = geo.to_ned_rpy(_quaternion(payload["q"]))
            return {
                "status": "ok",
                "roll": result.roll,
                "pitch": result.pitch,
                "yaw": result.yaw,
            }
        if op == "fk_tree":
            tree = RobotTree(_robot(robots[payload["robot"]]))
            return {
                "root_link": tree.root_link,
                "base_frame": tree.base_frame,
                "usable": tree.is_usable,
                "articulated": tree.is_articulated,
                "frames": list(tree.frames),
                "errors": [
                    {"code": error.code.name.lower(), "subject": error.subject}
                    for error in tree.errors
                ],
            }
        if op == "fk_frame_pose":
            tree = RobotTree(_robot(robots[payload["robot"]]))
            base = _transform(payload["base"]) if "base" in payload else None
            pose = tree.frame_pose(
                payload["frame"], base=base, joint_positions=payload["joint_positions"]
            )
            if not pose.available:
                return {
                    "status": "ok",
                    "available": False,
                    "missing_joints": list(pose.missing_joints),
                }
            return {
                "status": "ok",
                "available": True,
                "transform": pose.transform.to_dict(),
            }
    except GeometryError as error:
        result = {"status": error.code.name.lower()}
        if op == "fk_frame_pose":
            result["error_subject"] = error.subject
        return result
    raise AssertionError(f"unhandled op {op!r}")


def _budget(expected: float, tolerance: float) -> float:
    """Relative above 1, absolute below. Mirrors ``golden_budget`` in
    ``tests/golden_ops.hpp`` and ``budget`` in ``tests/run_golden.mjs``."""
    return tolerance * max(1.0, abs(expected))


def _compare(expected, actual, path: str, tolerance: float) -> list[str]:
    if isinstance(expected, bool) or isinstance(actual, bool):
        return (
            []
            if expected == actual
            else [f"{path}: expected {expected!r}, got {actual!r}"]
        )
    if isinstance(expected, int | float) and isinstance(actual, int | float):
        if math.isclose(expected, actual, rel_tol=0.0, abs_tol=_budget(expected, tolerance)):
            return []
        return [f"{path}: expected {expected!r}, got {actual!r}"]
    if isinstance(expected, dict):
        if not isinstance(actual, dict):
            return [f"{path}: expected an object, got {type(actual).__name__}"]
        problems = []
        for key in set(expected) | set(actual):
            if key not in expected:
                problems.append(f"{path}.{key}: unexpected")
            elif key not in actual:
                problems.append(f"{path}.{key}: missing")
            else:
                problems += _compare(
                    expected[key], actual[key], f"{path}.{key}", tolerance
                )
        return problems
    if isinstance(expected, list):
        if not isinstance(actual, list) or len(expected) != len(actual):
            return [f"{path}: expected {expected!r}, got {actual!r}"]
        problems = []
        for index, (left, right) in enumerate(zip(expected, actual, strict=False)):
            problems += _compare(left, right, f"{path}[{index}]", tolerance)
        return problems
    return (
        [] if expected == actual else [f"{path}: expected {expected!r}, got {actual!r}"]
    )


def _case_ids(document: dict) -> list[str]:
    return [case["id"] for case in document["cases"]]


def test_golden_file_is_not_empty(golden: dict) -> None:
    assert len(golden["cases"]) > 100, "the golden suite should cover the whole surface"
    assert len(set(_case_ids(golden))) == len(golden["cases"]), (
        "case ids must be unique"
    )


def test_core_version_matches_the_golden_file(golden: dict) -> None:
    from cyberwave_geometry import core_version

    assert core_version() == golden["version"], (
        "the loaded core is a different version from the one the golden vectors "
        "were generated with; rebuild the library or regenerate the file"
    )


def _load_cases() -> list:
    if not GOLDEN_PATH.exists():
        return []
    document = json.loads(GOLDEN_PATH.read_text())
    return [(document, case) for case in document["cases"]]


@pytest.mark.parametrize(
    ("document", "case"), _load_cases(), ids=[case["id"] for _, case in _load_cases()]
)
def test_golden_case(document: dict, case: dict) -> None:
    actual = _evaluate(document["robots"], case["op"], case["input"])
    problems = _compare(case["expect"], actual, "expect", document["tolerance"])
    assert not problems, "\n".join([f"case {case['id']}", *problems])
