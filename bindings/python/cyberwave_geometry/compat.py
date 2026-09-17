"""Lenient adapters for boundaries that must keep their existing behaviour.

The core is strict: a degenerate quaternion is an error. Plenty of existing
Cyberwave call sites instead fall back to identity, because they parse
persisted JSON that may predate any validation and would rather render a robot
upright than 500. That behaviour is preserved -- but here, named, and only
here, so a reviewer can see exactly which paths swallow an error and a future
migration has a list to work through.

Every function in this module is a *boundary* helper. Anything doing geometry
rather than parsing should call the strict API in the sibling modules.
"""

from __future__ import annotations

import math
from typing import Any

from . import quaternion as quat
from .types import GeometryError, Quaternion, Transform, Vector3

__all__ = [
    "IDENTITY_ROTATION",
    "ZERO_POSITION",
    "coerce_quaternion",
    "coerce_vector",
    "normalize_quaternion_or_identity",
    "quaternion_from_wire",
    "quaternion_to_wire",
    "transform_from_dicts",
]

#: Key order is the core's x, y, z, w -- these are read by name, but they are
#: also serialized, and JSON preserves insertion order.
IDENTITY_ROTATION = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
ZERO_POSITION = {"x": 0.0, "y": 0.0, "z": 0.0}


def _number(value: Any, default: float = 0.0) -> float:
    if isinstance(value, bool) or value is None:
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def coerce_vector(value: Any, default: Vector3 | None = None) -> Vector3:
    """Read a position from whatever shape it was persisted in."""
    fallback = default if default is not None else Vector3()
    if isinstance(value, dict):
        return Vector3(
            _number(value.get("x"), fallback.x),
            _number(value.get("y"), fallback.y),
            _number(value.get("z"), fallback.z),
        )
    if isinstance(value, list | tuple) and len(value) == 3:
        # Per-component fallback, same as the dict branch above. Passing no
        # default here made an unparseable component silently read 0.0 while
        # its siblings honoured the caller's default, so a partly-corrupt
        # sequence came back as a mix of the two -- worse than either.
        return Vector3(
            _number(value[0], fallback.x),
            _number(value[1], fallback.y),
            _number(value[2], fallback.z),
        )
    return fallback


def coerce_quaternion(value: Any) -> Quaternion:
    """Read a rotation without normalizing it.

    A four-element sequence is read as ``w, x, y, z``, which is what the
    Cyberwave backend has always persisted. Anything arriving from protobuf,
    ROS or Three.js is ``xyzw`` and must go through
    :meth:`Quaternion.from_xyzw` instead -- this function cannot tell them
    apart, which is why the strict API has no such entry point.

    An already-built :class:`Quaternion` passes through untouched, so a caller
    that has resolved the component order itself can still reach the lenient
    normalization below without round-tripping through a dict.
    """
    if isinstance(value, Quaternion):
        return value
    if isinstance(value, dict):
        return Quaternion(
            x=_number(value.get("x")),
            y=_number(value.get("y")),
            z=_number(value.get("z")),
            w=_number(value.get("w"), 1.0),
        )
    if isinstance(value, list | tuple) and len(value) == 4:
        # Positional, and deliberately wxyz: see the docstring above.
        return Quaternion(
            x=_number(value[1]),
            y=_number(value[2]),
            z=_number(value[3]),
            w=_number(value[0], 1.0),
        )
    return Quaternion.identity()


def normalize_quaternion_or_identity(value: Any) -> Quaternion:
    """Coerce to a unit quaternion, falling back to identity.

    ``coerce_quaternion`` accepts whatever keys are present, so a caller that
    stored Euler angles under ``x/y/z`` yields something like
    ``{w: 1, x: 0, y: 0, z: 1.57}``. Rotating a vector by that also scales it,
    so anything doing geometry rather than passing the value through has to
    normalize -- and a value that cannot be normalized becomes identity here
    rather than raising, matching what the backend has always done.
    """
    try:
        return quat.normalize(coerce_quaternion(value))
    except GeometryError:
        return Quaternion.identity()


def quaternion_from_wire(value: Any, *, order: str = "wxyz") -> Quaternion:
    """Read a quaternion off the wire in an explicitly named component order.

    ``order`` is required to be spelled out at the call site precisely because
    getting it wrong is silent: ``xyzw`` for protobuf, ROS and Three.js;
    ``wxyz`` for MuJoCo and the legacy backend payloads.
    """
    if order not in ("wxyz", "xyzw"):
        raise ValueError(f"order must be 'wxyz' or 'xyzw', got {order!r}")
    if isinstance(value, dict):
        # A dict is self-describing, so the order argument does not apply.
        return coerce_quaternion(value)
    # Any 4-long sequence, not just list/tuple: MuJoCo hands out `data.xquat[i]`
    # as a numpy array, and narrowing this to list|tuple made that fall through
    # to the identity below -- silently, which is the one failure mode this
    # function exists to prevent. str/bytes are excluded because they are
    # sequences of the wrong thing.
    if (
        not isinstance(value, str | bytes)
        and hasattr(value, "__len__")
        and hasattr(value, "__getitem__")
        and len(value) == 4
    ):
        numbers = [_number(value[index]) for index in range(4)]
        return (
            Quaternion.from_wxyz(numbers)
            if order == "wxyz"
            else Quaternion.from_xyzw(numbers)
        )
    return Quaternion.identity()


def quaternion_to_wire(q: Quaternion, *, order: str = "wxyz") -> list[float]:
    """Inverse of :func:`quaternion_from_wire`."""
    if order == "wxyz":
        return list(q.to_wxyz())
    if order == "xyzw":
        return list(q.to_xyzw())
    raise ValueError(f"order must be 'wxyz' or 'xyzw', got {order!r}")


def transform_from_dicts(position: Any, rotation: Any) -> Transform:
    """Build a Transform from the loose dicts persisted across the backend."""
    return Transform(
        coerce_vector(position), normalize_quaternion_or_identity(rotation)
    )
