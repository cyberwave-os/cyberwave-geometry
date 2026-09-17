# Geometry conventions

Every convention the Cyberwave geometry core commits to, and every place the
rest of the stack disagrees with one. If you are about to write a quaternion
formula, the answer is in here — and then you should not write the formula, you
should call the core.

The executable version of this document is
[`golden/geometry_golden.json`](golden/geometry_golden.json): 420 cases that
every language binding is tested against.

---

## 1. Quaternions are Hamilton, and components are named

`multiply(a, b)` is the rotation that applies **b first, then a**. This matches
ROS, Eigen, Three.js and MuJoCo. It does **not** match the JPL convention some
IMU vendors ship, where the product order is reversed.

Components are always reached by name — `x`, `y`, `z`, `w` — never by position.
This is not stylistic. The monorepo speaks both orders:

| Order  | Used by                                                          |
| ------ | ---------------------------------------------------------------- |
| `xyzw` | protobuf (`common/protobuf`), ROS, Three.js                       |
| `wxyz` | MuJoCo, the legacy Cyberwave backend payloads, `quaternion_wxyz`  |

A bare four-element array is ambiguous between them, and reading one as the
other is silent: you get a real, unit, plausible rotation that is simply wrong.
So the core has no positional constructor. Convert explicitly at the boundary:

```python
from cyberwave_geometry import Quaternion

Quaternion.from_xyzw(protobuf_values)   # protobuf / ROS / Three.js
Quaternion.from_wxyz(mujoco_values)     # MuJoCo / legacy backend
q.to_xyzw()                             # going back out
```

```cpp
const auto q = cyberwave::geometry::quat::from_xyzw(x, y, z, w);
const auto out = cyberwave::geometry::quat::to_wxyz(q);
```

The core's own in-memory layout — `Quaternion` in `types.hpp`, `cw_geom_quat`
across the C ABI, the `Quaternion` dataclass and `CQuat` in the Python binding —
is `x, y, z, w`, so the one order the core stores is the Cyberwave one. Nothing
should read it positionally anyway: C++ brace-initializes with designated
initializers (`Quaternion{.x = …, .y = …, .z = …, .w = …}`) and Python
constructs with keywords, which is what makes a layout change like this one a
compile-or-crash rather than a silently rotated pose.

**Wire formats are not being rewritten.** protobuf, ROS and Three.js stay
`xyzw`; MuJoCo and the legacy backend payloads stay `wxyz`. The change is that
the conversion is now spelled out where it happens instead of being implied by
which file you are reading.

Two of those orders are not ours to choose. MuJoCo hands out `data.xquat` as
`wxyz`, so every simulator-facing surface in `cyberwave-sim`, `cyberwave-rl` and
`cyberwave-demos` is `wxyz` by definition — converting at that seam is the only
move available. "`xyzw` everywhere" is therefore not a reachable end state; the
reachable one is **`xyzw` inside Cyberwave, `wxyz` quarantined behind named
adapters at the simulator and legacy-payload seams.**

Where a wire order is genuinely ours but cannot flip yet, name it once and route
every read and write through that name, so the flip is a one-line change rather
than an audit. The navigate command does this: its scalar-first order lives in
`NAVIGATE_WIRE_QUAT_ORDER`
(`cyberwave-backend/src/app/services/control/navigation/geometry.py`), and both
the resolver and the workflow code emitter go through
`quat_from_navigate_wire` / `quat_to_navigate_wire`. Flipping it remains a wire
break — deployed edge drivers parse `[w, x, y, z]` off `twin/{uuid}/navigate/cmd`
— so it needs a payload-version bump and a dual-read window. Centralising the
order is the prerequisite for that, not a substitute for it.

---

## 2. Roll/pitch/yaw is fixed-axis XYZ — *not* Three.js's default

`from_rpy(roll, pitch, yaw)` composes

```
q = qz(yaw) · qy(pitch) · qx(roll)
```

which is **fixed-axis (extrinsic) XYZ**, equivalently **intrinsic Z-Y-X**. This
is the URDF and ROS convention, and it is what
`cyberwave_robot_format.math_utils.Quaternion.from_rpy` and
`cyberwave.schema.Quaternion.from_rpy` already implement — the core matches
those bit for bit, so swapping a call site over changes nothing numerically.

⚠️ **Three.js defaults to intrinsic `"XYZ"`**, which is a *different* rotation
for the same three angles. `cyberwave-frontend/lib/utils/rotation.ts` calls
`new THREE.Euler().setFromQuaternion(quat, "XYZ")`. That is correct for what it
does — it drives a Three.js scene graph — but the numbers it produces are not
URDF roll/pitch/yaw and must not be persisted as such. When a frontend value
crosses into stored geometry, convert through the quaternion, never by copying
the three angles across.

At **gimbal lock** (pitch = ±π/2) roll and yaw are the same degree of freedom.
`to_rpy` attributes the whole rotation to yaw and reports roll as zero. The
angles will not match what you put in, but `from_rpy(to_rpy(q))` still
reconstructs `q` — which is the property that actually matters. "At lock" means
`|cos(pitch)| < kMinCosPitch` (§8), so an attitude merely *near* vertical keeps
its roll.

---

## 3. Transforms rotate, then translate

`apply(t, p) = R·p + t`. Composition is

```
compose(parent, child).translation = parent.translation + parent.rotation · child.translation
compose(parent, child).rotation    = parent.rotation · child.rotation
```

`relative(parent, child)` is the exact inverse of `compose`, for reading a
world-baked pose back out as parent-relative (what MJCF `<body pos= quat=>`
wants).

`inverse_unit` and `compose` assume normalized rotations. That is not laziness:
they are the FK inner loop, and everything reaching them has been normalized by
`RobotTree::build` or by a validating factory. The checked forms — `normalize`,
`inverse` — exist for unvalidated input and say so in their return type.

---

## 4. Joints follow URDF

* A joint's **origin is applied before its motion**. The origin places the joint
  frame in its parent; the motion then happens about or along the axis *in that
  frame*. The other order puts a wrist camera 90° out whenever the origin
  carries a rotation, which is why
  [`tests/test_fk.cpp`](tests/test_fk.cpp) pins both orders explicitly.
* `fixed` contributes its origin only.
* `revolute` / `continuous` rotate about the **normalized** axis by the reported
  position, in radians.
* `prismatic` translates along the **normalized** axis by the reported position,
  in metres. Normalizing matters: a URDF axis of `(3, 0, 0)` with a position of
  `0.5` means 0.5 m of travel, not 1.5 m.
* A **mimic** joint's position is `multiplier · source + offset`. Its own name
  is never reported by the robot; its source's is, and that is what the chain
  declares as required.
* Any other joint type (`floating`, `planar`, a vendor extension) parses to
  `unsupported`. Frames below it resolve as an **error**, never as a pose frozen
  at the joint origin.

---

## 5. Strictness, and where leniency is allowed

The core never falls back to identity, never guesses a default and never
throws. An operation it cannot carry out returns a structured error naming the
subject it failed on:

| Code                      | Meaning                                              |
| ------------------------- | ---------------------------------------------------- |
| `invalid_quaternion`      | non-finite, or norm below `1e-9`                      |
| `invalid_axis`            | non-finite, or norm below `1e-12`                     |
| `non_finite_value`        | a NaN or infinity reached the arithmetic              |
| `invalid_rotation_matrix` | not right-handed orthonormal within tolerance         |
| `no_links`                | the description declares nothing                      |
| `no_root`                 | every link has a parent joint, or the frame is cut off |
| `multiple_roots`          | the description is disconnected                       |
| `ambiguous_parent`        | two joints claim the same child link                  |
| `cycle`                   | a loop was walked resolving a chain                   |
| `invalid_joint_pose`      | a joint origin quaternion could not be normalized     |
| `invalid_sensor_pose`     | a sensor extrinsic could not be normalized            |
| `unsupported_joint_type`  | the joint's motion is not a single scalar             |
| `missing_joint`           | a joint the chain needs has no reported position      |
| `unknown_frame`           | the frame is neither a link nor a sensor              |

A near-zero quaternion is an *error* rather than an identity because treating it
as identity persists a confidently wrong orientation — far worse than an
honestly missing frame.

**Compatibility adapters may still be lenient**, and several must be. The
clearest example is already written into the wire contract:
`common/protobuf/v0/cyberwave/mqtt/twin/rotation.proto` says an all-zero
quaternion "is treated as identity [w=1] by consumers that need a valid
rotation". That is a parsing boundary doing exactly what the core refuses to
do, and correctly so — a proto3 field that was never set arrives as all zeros
and means "absent", not "degenerate".

Behaviour like that is preserved in `cyberwave_geometry.compat`, where it is
named and greppable, rather than buried in the arithmetic:

```python
from cyberwave_geometry import compat

compat.normalize_quaternion_or_identity(payload.rotation)   # all-zero -> identity
```

### Tree errors are collected, not raised

`RobotTree::build` gathers structural problems instead of failing. A robot whose
wrist camera is unreachable must still report its base pose and joint state, so
the caller decides *per frame* what is unavailable. Note that one problem
cascades: dropping a malformed joint orphans everything below it, so you will
usually see an `invalid_joint_pose` followed by a `multiple_roots`.

### Three outcomes, kept distinct

Asking for a frame pose yields one of:

1. **a pose** — the chain resolved and the state was there;
2. **`available=False` with `missing_joints`** — the description is fine, the
   robot has not reported that joint yet;
3. **an error** — something structural is wrong.

Collapsing (2) into (3), or either into a default pose, is the failure mode this
library exists to prevent.

---

## 6. Determinism

Anything the core returns as a list is sorted: `frames`, `required_joints`,
`missing_joints`. Root selection among several candidates takes the
lowest-sorting name. This is so that the same description gives the same answer
in C++, Python, the browser and on an edge node, regardless of hash seeds or
iteration order.

---

## 7. Units and frames

Lengths are **metres**, angles are **radians**, and the core is frame-agnostic:
it composes whatever you hand it. The `base` transform passed to a frame-pose
query is the already-world-resolved pose of the tree's base frame — the caller
applies the environment's navigation anchor *before* calling, which keeps FK a
pure function of the description and the joint state.

Cyberwave environments and the navigation frame are both **Z-up**; see
`cyberwave-backend/src/lib/transform_utils.py` for the anchor transform itself,
which is a separate concern from this library.

The one exception to both halves of that — the units and the
frame-agnosticism — is §9. A latitude is not a number the core can compose
blindly, and it is not in radians.

---

## 8. Numerical tolerances

| Constant             | Value   | Meaning                                             |
| -------------------- | ------- | --------------------------------------------------- |
| `kMinQuaternionNorm` | `1e-9`  | below this a quaternion carries no usable rotation   |
| `kMinAxisNorm`       | `1e-12` | below this an axis cannot be normalized              |
| `kMinCosPitch`       | `1e-12` | below this `\|cos(pitch)\|` an rpy split is singular |
| golden `tolerance`   | `1e-12` | cross-language comparison budget                     |

`kMinCosPitch` is tested on `cos(pitch)`, taken as `hypot(R00, R10)`, and never
on `sin(pitch)`. Analytically the two say the same thing; in double they do not.
1e-7 rad off vertical `sin(pitch)` is `1 - 5e-15`, three ulps from the pole's own
value, so a test on `sin` cannot tell a steep attitude from a singular one and
throws away a roll that was still there — which for anything that rebuilds
through `from_rpy` is a wrong rotation, not a rounding error. `hypot(R00, R10)`
is `1e-7` with its full relative accuracy. The batched `cyberwave_geometry.torch`
kernel widens the same threshold to `8 ulp` where the dtype is coarser, because
at float32 the noise floor of `1 - 2(y^2 + z^2)` is already ~`5e-8`.

The golden tolerance is *not* exact equality on purpose: libm's `sin`/`cos`
differ in the last ulp between platforms and languages, and a bit-exact file
would fail for reasons that have nothing to do with geometry. It is still tight
enough that a wrong sign, a swapped component order or a transposed matrix
fails immediately.


---

## 9. A GPS pose is `lat, lon, alt` plus a rotation into **local ENU**

`GeoPose` is the six-degree-of-freedom answer:

```
GeoPose = { Geodetic position, Quaternion orientation }
Geodetic = { latitude_deg, longitude_deg, altitude_m }
```

Three numbers of position, three degrees of freedom of attitude carried as a
quaternion. `orientation` maps the **body** frame into the **local ENU** frame
at that same position:

```
v_enu = rotate(orientation, v_body)
```

with **ENU** being `+X east, +Y north, +Z up`, and the body frame being **FLU** —
`+X forward, +Y left, +Z up` (REP-103, and already what the rest of this core
assumes when `from_yaw` measures yaw counter-clockwise from `+X`).

An identity orientation is therefore a body facing **east** and level. That is
the one counter-intuitive consequence of an ENU-referenced pose, and
`quat_from_compass_heading` exists so that nobody has to keep it in their head.

### Position and orientation are one value

ENU is defined by the ellipsoid normal at a point, so the same quaternion
100 km away is a different attitude. Assembling a `GeoPose` from two
separately-timestamped messages therefore produces a pose that was never real.
The wire contract already says this for camera exposure poses
(`CameraGeodeticPosePayload` in `cyberwave-backend/src/lib/mqtt_schemas.py`);
the type is where it stops being a comment.

### Degrees, and why this one rule is broken on purpose

§7 says angles are radians. Geodetic coordinates and bearings are **degrees**,
and every such field and parameter says so in its name (`latitude_deg`,
`heading_deg`). Everything that produces a geodetic coordinate speaks degrees —
NMEA, MAVLink, DJI MSDK, the `twin/{uuid}/gps` payload, a maps URL — and a
latitude in radians is indistinguishable by eye from one in degrees: it is
simply a place 57 times closer to the equator.

The split inside this module is: **a bearing is degrees, an attitude angle is
radians.** So `quat_from_compass_heading(heading_deg)` takes degrees and
`quat_from_ned_rpy(roll, pitch, yaw)` takes radians, like `from_rpy`.

### Altitude carries no datum

`altitude_m` is a number, not a number-plus-a-datum. Whether it is orthometric
(MSL) or ellipsoidal (HAE) is a property of the receiver and of the site
survey; it is recorded beside the data and **never converted here**. The
conversion needs a geoid model (EGM96/EGM2008), which is a large data
dependency for a correction that GNSS vertical error swamps anyway. Mixing the
two is a ~30 m Z error, so a publisher has to be matched to the datum its site
was surveyed in — which is what `anchor["altitude_datum"]` records and does not
instruct.

### A site is a `GeoAnchor`

```
GeoAnchor = { Geodetic origin, heading_deg }
```

`heading_deg` is the true-north bearing of environment **+Y**, in degrees
**clockwise**. `heading_deg == 0` therefore means the environment frame *is* the
local ENU frame.

`+Y` rather than `+X` because "which way is north on my floor plan" is a
statement about the axis that runs up the screen in the default top-down view,
and because zero then means the standard frame for a georeferenced survey. This
is `environment.settings["geo"]` in the backend, unchanged.

Given an anchor, the whole conversion is:

```
enu   = tangent_plane(anchor.origin -> position)     # metres
local = rotate(from_yaw(radians(heading_deg)), enu)  # environment frame
rotation_local = from_yaw(radians(heading_deg)) * orientation
```

### One projection, and no library

The tangent plane uses the WGS-84 radii of curvature `M` and `N` **at the anchor
latitude**, not a single metres-per-degree constant. Treating the semi-major
axis as a mean radius carries a +0.67% .. −0.29% scale error — metres per
kilometre, all of it in the position.

Truncation is about `d²/2R`: 2 cm at 500 m, 31 cm at 2 km, one to two orders of
magnitude below consumer GNSS error (1.5–5 m CEP). **No ECEF round trip, no
UTM, no geoid, no projection library** — the no-dependencies rule applies here
too, and a caller spanning more than a few kilometres wants its own anchor, not
a better projection.

The core owns the arithmetic and not the policy. It will happily tell you a fix
is 800 km from its anchor; deciding that such a fix is a bug rather than a
journey is the caller's call, and the threshold differs per caller
(`MAX_GEO_ANCHOR_DISTANCE_M` in the backend and the frontend).

### Longitude wraps, and the difference wraps too

`normalize_longitude_deg` is applied to anchor-to-target **differences**, not
only to outputs. Longitude is the one coordinate that wraps, and subtracting two
raw values across the antimeridian gives ~360° where the true separation is
metres. Without it, an environment anchored in Fiji, Chukotka or the far east of
New Zealand computes every fix as ~40 000 km away, fails the caller's distance
guard, and GPS simply never works there — with the operator told their anchor or
their device is wrong.

### Absent is not zero, and out of range is not clamped

`(0, 0)` is a real location in the Gulf of Guinea. A missing coordinate that
defaults to zero is a confidently wrong place on Earth rather than an obviously
missing one, so `Geodetic.from_dict` refuses a dropped key — deliberately unlike
`Vector3.from_dict`, whose zero default is harmless. A latitude outside
`[-90, 90]` is `invalid_geodetic`, never a clamp. An anchor within
`kMaxTangentPlaneLatitude` (89.9°) of a pole is `invalid_geo_anchor`, because the
longitude scale `N·cos(φ)` degenerates there and the inverse would amplify
metres into thousands of degrees.

### NED and compass bearings are quarantined, exactly like `wxyz`

Two foreign conventions reach the core only through named adapters, for the same
reason §1 gives: the value alone does not say which one it is.

| Foreign form | Adapter | Conversion |
| ------------ | ------- | ---------- |
| Compass bearing, degrees CW from **true** north | `quat_from_compass_heading` / `compass_heading_deg` | `yaw_enu = π/2 − heading` |
| NED attitude (aviation, DJI), radians | `quat_from_ned_rpy` / `ned_rpy` | `roll` unchanged, `pitch_enu = −pitch_ned`, `yaw_enu = π/2 − yaw_ned` |

Two traps worth stating outright, because both survive a casual test:

* **Feeding NED angles straight into `from_rpy` is a reflection about the
  north-east diagonal.** The heading error is `2·ψ − 90°`: 90° wrong at every
  cardinal heading, half a turn at 135° and 315°, and *correct at 45° and 225°*.
* **Positive NED pitch is nose-UP; positive ENU/FLU pitch is nose-DOWN.** ENU
  pitch turns about `+Y` = left (REP-103), so the two conventions disagree and
  nothing in the numbers says so. A dropped sign points a nadir camera at the
  sky.

Magnetic declination is **not** part of this. Publishers apply it themselves —
most compasses and Android's rotation vector are magnetic-referenced, and
declination changes over a site's lifetime, so an anchor-side declination field
would silently rot.

A compass heading produces a **yaw-only** rotation. A GNSS fix plus a compass
carries no roll or pitch, and a fabricated level attitude is indistinguishable
from a measured one.

### What this replaces

`cyberwave-backend/src/lib/geo_transform.py` and
`cyberwave-frontend/lib/utils/geo-transform.ts` are the same arithmetic written
twice, kept in agreement by a "Keep in sync with" comment at the top of each —
the exact duplication this library exists to remove. The core reproduces both to
within 1e-13 m at site ranges, so migrating them is a call-site change, not a
behaviour change. A third copy, `offsetWgs84` in the Android driver's
`CameraPoseTransform.kt`, uses the spherical constant and is the one that will
move measurably (by its documented 0.67%).
