# `cyberwave-geometry` (Python)

**Internal document.**

Python binding for the [Cyberwave shared geometry core](../../README.md). It
holds **no formulas of its own** — every function here marshals to the C++
library and back. Adding arithmetic to this package would recreate exactly the
duplication the core exists to remove.

## Install

```bash
pip install -e common/geometry/bindings/python
```

The package needs `libcyberwave_geometry_c.so` (`.dylib` / `.dll`) at import
time. It looks, in order:

1. `$CYBERWAVE_GEOMETRY_LIBRARY` — an explicit path;
2. beside the package, or in `cyberwave_geometry/_lib/` — how a built wheel ships;
3. `$CYBERWAVE_GEOMETRY_BUILD_DIR` — a developer's CMake build tree;
4. the platform loader's own search path.

For local development:

```bash
cmake -S common/geometry -B build/geometry && cmake --build build/geometry -j
export CYBERWAVE_GEOMETRY_LIBRARY=$PWD/build/geometry/libcyberwave_geometry_c.so
```

If it cannot find the library the `ImportError` prints every path it tried.

## Use

```python
from cyberwave_geometry import Quaternion, Transform, Vector3
from cyberwave_geometry import quaternion as quat, transform as tf

q = quat.from_rpy(0.1, 0.2, 0.3)          # fixed-axis XYZ, the URDF convention
tf.compose(parent, child)
quat.rotate(q, Vector3(1, 0, 0))
```

Forward kinematics takes a parsed description — this package does **not** read
the Universal Robot Schema, URDF or MJCF; your schema reader does, and hands
the result over:

```python
from cyberwave_geometry import JointType
from cyberwave_geometry.fk import JointDescription, RobotDescription, RobotTree

tree = RobotTree(RobotDescription(
    links=["base_link", "tool"],
    joints=[JointDescription("j", "base_link", "tool", JointType.REVOLUTE,
                             axis=Vector3(0, 0, 1))],
))

pose = tree.frame_pose("tool", joint_positions={"j": 1.57})
if pose.available:
    print(pose.transform)
else:
    print("waiting on", pose.missing_joints)
```

Three outcomes, kept distinct on purpose: a pose, `available=False` with
`missing_joints` when the robot has not reported that state yet, and a raised
`GeometryError` for a structural problem. Collapsing any of them into a default
pose is the failure mode this library prevents.

## GPS poses

A position on Earth is `Geodetic(latitude_deg, longitude_deg, altitude_m)` --
WGS-84, degrees and metres. A six-degree-of-freedom pose is a `GeoPose`, whose
`orientation` maps the body frame (forward-left-up) into the local **ENU** frame
at its own position:

```python
from cyberwave_geometry import GeoAnchor, Geodetic, GeoPose
from cyberwave_geometry import geodetic as geo

anchor = GeoAnchor(Geodetic(47.3769, 8.5417, 408.0), heading_deg=30.0)

geo.to_local(anchor, Geodetic(47.3801, 8.5500, 421.0))   # -> Vector3, metres
geo.to_local_pose(anchor, pose)                          # -> Transform, ready for FK
geo.from_local_pose(anchor, transform)                   # -> GeoPose
```

`heading_deg` is the true-north bearing of environment +Y, so `0` means the
environment frame *is* ENU. Altitude carries no datum -- MSL vs ellipsoidal is
recorded beside the data and never converted here.

Compass bearings and NED attitudes are foreign conventions and go through named
adapters, for the same reason `wxyz` does:

```python
geo.quat_from_compass_heading(137.5)   # degrees CLOCKWISE from true north
geo.quat_from_ned_rpy(roll, pitch, yaw)  # radians; NED pitch is nose-UP
```

Two traps the adapters exist to prevent: feeding NED angles straight into
`quat.from_rpy` is right at 45 degrees and 90 degrees wrong at every cardinal
heading, and NED positive pitch is nose-up where ENU/FLU positive pitch is
nose-down. The full convention is
[`CONVENTIONS.md`](../../CONVENTIONS.md) section 9.

## Strict by default

Degenerate input raises rather than becoming an identity rotation:

```python
quat.normalize(Quaternion(0, 0, 0, 0))    # GeometryError: invalid_quaternion
geo.validate(Geodetic(91.0, 0.0, 0.0))    # GeometryError: invalid_geodetic
```

Geodetic values are stricter still: `Geodetic.from_dict` refuses a *missing*
coordinate rather than defaulting it to zero, because `(0, 0)` is a real place
in the Gulf of Guinea -- a silently relocated robot, not an obvious failure.

Where a call site must stay lenient — parsing persisted JSON that predates any
validation — use `cyberwave_geometry.compat`, which keeps the identity fallback
but names it:

```python
from cyberwave_geometry import compat

compat.normalize_quaternion_or_identity(whatever_was_stored)
compat.quaternion_from_wire(values, order="xyzw")   # order is not optional
```

## Component order

A bare four-element array is ambiguous between `xyzw` (protobuf, ROS, Three.js)
and `wxyz` (MuJoCo, legacy backend payloads), and reading one as the other is
silent — you get a real, unit, plausible, wrong rotation. So there is no
positional constructor:

```python
Quaternion.from_xyzw(protobuf_values)
Quaternion.from_wxyz(mujoco_values)
q.to_xyzw()
```

The dataclass fields are declared `x, y, z, w` — the Cyberwave order, matching
`cw_geom_quat` and the C++ core — so a positional `Quaternion(a, b, c, d)` is
`xyzw`. Construct with keywords (`Quaternion(x=…, y=…, z=…, w=…)`) or the named
converters above rather than relying on that.

## Why ctypes and not a compiled extension

`pip install` keeps working on every platform with no wheel matrix and no
compiler, which is what makes migrating the backend, the SDK and the edge nodes
onto the core tractable at all. The C ABI is the seam a compiled accelerator
would slot into later without changing anything above `_native.py`. See
[`../../docs/MIGRATION.md`](../../docs/MIGRATION.md) for the packaging decision
that still has to be made before the SDK can depend on this.

## Tests

```bash
cd common/geometry/bindings/python && python -m pytest
```

`tests/test_golden.py` runs the shared
[golden vectors](../../golden/geometry_golden.json) — the same file the C++
runner reads, which is what makes "C++ and Python agree" a tested claim.
`tests/test_binding.py` covers what only exists on this side: handle lifetimes,
the strict/lenient split, and the named-component discipline.
`tests/test_geodetic.py` covers the geodetic marshalling -- three bare doubles
across an ABI, where a field swap would place a robot in the wrong hemisphere
without raising anything.
