"""Value types and errors mirroring the C++ core.

Named components throughout: a quaternion is ``Quaternion(x=, y=, z=, w=)``,
never a bare four-element sequence whose order you have to infer from which
subsystem produced it. Use :meth:`Quaternion.from_wxyz` /
:meth:`Quaternion.to_wxyz` at every boundary that speaks the other order.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from enum import IntEnum

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
]


class ErrorCode(IntEnum):
    """Mirrors ``cyberwave::geometry::ErrorCode``. Values are part of the ABI."""

    OK = 0
    INVALID_QUATERNION = 1
    INVALID_AXIS = 2
    NON_FINITE_VALUE = 3
    INVALID_ROTATION_MATRIX = 4
    INVALID_GEODETIC = 5
    INVALID_GEO_ANCHOR = 6
    NO_LINKS = 10
    NO_ROOT = 11
    MULTIPLE_ROOTS = 12
    AMBIGUOUS_PARENT = 13
    CYCLE = 14
    INVALID_JOINT_POSE = 15
    INVALID_SENSOR_POSE = 16
    UNSUPPORTED_JOINT_TYPE = 17
    MISSING_JOINT = 18
    UNKNOWN_FRAME = 19


class GeometryError(ValueError):
    """Raised where the core returns an error.

    Subclasses ``ValueError`` so that a caller migrating off a function that
    used to raise on bad input does not have to widen its ``except``.
    """

    def __init__(
        self, code: ErrorCode | int, subject: str = "", detail: str = ""
    ) -> None:
        self.code = code
        self.subject = subject
        self.detail = detail
        # A plain int is a code this enum does not mirror -- see `check`.
        parts = [code.name.lower() if isinstance(code, ErrorCode) else f"error_{code}"]
        if subject:
            parts.append(f"subject={subject!r}")
        if detail:
            parts.append(detail)
        super().__init__(": ".join(parts))


class JointType(IntEnum):
    """Mirrors ``cyberwave::geometry::fk::JointType``."""

    FIXED = 0
    REVOLUTE = 1
    CONTINUOUS = 2
    PRISMATIC = 3
    #: Anything whose motion is not a single scalar. Frames below one of these
    #: resolve as an error rather than being frozen at the joint origin.
    UNSUPPORTED = 4

    @property
    def is_actuated(self) -> bool:
        return self in (JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC)

    @property
    def is_supported(self) -> bool:
        return self is not JointType.UNSUPPORTED


@dataclass(frozen=True)
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def zero(cls) -> Vector3:
        return cls()

    @classmethod
    def from_sequence(cls, values: Sequence[float]) -> Vector3:
        x, y, z = values
        return cls(float(x), float(y), float(z))

    def to_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}

    @classmethod
    def from_dict(cls, data: dict) -> Vector3:
        return cls(
            float(data.get("x", 0.0)),
            float(data.get("y", 0.0)),
            float(data.get("z", 0.0)),
        )


@dataclass(frozen=True)
class Quaternion:
    """A Hamilton quaternion with named components.

    The dataclass field order is ``x, y, z, w`` -- the Cyberwave convention,
    matching ``cw_geom_quat`` and the C++ core. Positional construction is
    still discouraged for exactly the reason this class exists; prefer
    :meth:`from_wxyz` or :meth:`from_xyzw`, which say which order they mean.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @classmethod
    def identity(cls) -> Quaternion:
        return cls()

    @classmethod
    def from_wxyz(cls, values: Iterable[float]) -> Quaternion:
        w, x, y, z = values
        return cls(x=float(x), y=float(y), z=float(z), w=float(w))

    @classmethod
    def from_xyzw(cls, values: Iterable[float]) -> Quaternion:
        x, y, z, w = values
        return cls(x=float(x), y=float(y), z=float(z), w=float(w))

    def to_wxyz(self) -> tuple[float, float, float, float]:
        """Order used by MuJoCo and the legacy Cyberwave backend payloads."""
        return (self.w, self.x, self.y, self.z)

    def to_xyzw(self) -> tuple[float, float, float, float]:
        """Order used by protobuf, ROS and Three.js."""
        return (self.x, self.y, self.z, self.w)

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "w": self.w}

    @classmethod
    def from_dict(cls, data: dict) -> Quaternion:
        return cls(
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            z=float(data.get("z", 0.0)),
            w=float(data.get("w", 1.0)),
        )

    @property
    def norm(self) -> float:
        """Delegates to the core, like everything else in this package.

        Imported inside the property rather than at module scope because
        ``_native`` imports this module for :class:`GeometryError`; a top-level
        import would close that cycle.
        """
        from . import quaternion

        return quaternion.norm(self)


@dataclass(frozen=True)
class Transform:
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)

    @classmethod
    def identity(cls) -> Transform:
        return cls()

    def to_dict(self) -> dict[str, dict[str, float]]:
        return {
            "translation": self.translation.to_dict(),
            "rotation": self.rotation.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> Transform:
        return cls(
            Vector3.from_dict(data.get("translation") or {}),
            Quaternion.from_dict(data.get("rotation") or {}),
        )


@dataclass(frozen=True)
class Rpy:
    """Roll/pitch/yaw in radians, fixed-axis XYZ. See CONVENTIONS.md."""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class AxisAngle:
    axis: Vector3 = field(default_factory=lambda: Vector3(1.0, 0.0, 0.0))
    angle: float = 0.0


@dataclass(frozen=True)
class Geodetic:
    """A WGS-84 position: degrees, degrees, metres.

    Degrees rather than radians is the one place this package departs from the
    core's "angles are radians" rule, and the field names carry ``_deg`` so the
    exception is never silent. Everything that produces a geodetic coordinate --
    NMEA, MAVLink, DJI MSDK, the ``twin/{uuid}/gps`` payload -- speaks degrees.

    ``altitude_m`` carries no datum. Whether it is MSL or ellipsoidal is a
    property of the receiver and of the site survey, recorded beside the data
    and never converted here: the conversion needs a geoid model the core will
    not depend on, for a correction GNSS vertical error swamps anyway.
    """

    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    altitude_m: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return {
            "latitude": self.latitude_deg,
            "longitude": self.longitude_deg,
            "altitude": self.altitude_m,
        }

    @classmethod
    def from_dict(cls, data: dict) -> Geodetic:
        """Build from the ``latitude``/``longitude``/``altitude`` wire spelling.

        Missing keys are *not* defaulted to zero here the way ``Vector3`` and
        ``Quaternion`` default theirs: (0, 0) is the Gulf of Guinea, so a
        dropped coordinate would become a confident position on the far side of
        the planet rather than an obvious failure.
        """
        missing = [
            k for k in ("latitude", "longitude", "altitude") if data.get(k) is None
        ]
        if missing:
            raise GeometryError(
                ErrorCode.INVALID_GEODETIC,
                "",
                f"geodetic position is missing {', '.join(missing)}",
            )
        return cls(
            float(data["latitude"]),
            float(data["longitude"]),
            float(data["altitude"]),
        )


@dataclass(frozen=True)
class GeoPose:
    """Six degrees of freedom on Earth: where, and which way up.

    ``orientation`` maps the body frame (forward-left-up) into the local ENU
    frame (east-north-up) at ``position``, and means nothing without it: ENU is
    defined by the ellipsoid normal at a point, so the same quaternion 100 km
    away is a different attitude. The two are one value for that reason -- never
    assemble them from separately-timestamped messages.

    See ``common/geometry/CONVENTIONS.md`` section 9.
    """

    position: Geodetic = field(default_factory=Geodetic)
    orientation: Quaternion = field(default_factory=Quaternion)

    def to_dict(self) -> dict[str, object]:
        return {**self.position.to_dict(), "rotation": self.orientation.to_dict()}

    @classmethod
    def from_dict(cls, data: dict) -> GeoPose:
        return cls(
            Geodetic.from_dict(data), Quaternion.from_dict(data.get("rotation") or {})
        )


@dataclass(frozen=True)
class GeoAnchor:
    """A site's georeference.

    ``heading_deg`` is the true-north bearing of environment +Y, in degrees
    clockwise, so ``0`` means the environment frame *is* the local ENU frame.
    Mirrors ``environment.settings["geo"]`` in the backend.
    """

    origin: Geodetic = field(default_factory=Geodetic)
    heading_deg: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return {**self.origin.to_dict(), "heading_deg": self.heading_deg}

    @classmethod
    def from_dict(cls, data: dict) -> GeoAnchor:
        heading = data.get("heading_deg")
        if heading is None:
            raise GeometryError(
                ErrorCode.INVALID_GEO_ANCHOR, "", "geo anchor is missing heading_deg"
            )
        return cls(Geodetic.from_dict(data), float(heading))


@dataclass(frozen=True)
class MetresPerDegree:
    """Local scale on the WGS-84 ellipsoid. ``longitude`` includes ``cos(phi)``."""

    latitude: float = 0.0
    longitude: float = 0.0
