# `common/geometry` — the shared geometry core

**Internal document.** There is no `LICENSE` in this directory, so nothing here
is published. See [`CONVENTIONS.md`](CONVENTIONS.md) for the conventions
themselves.

A dependency-free C++20 library that is the single executable source of truth
for **scalar quaternion, transform, forward-kinematics and geodetic arithmetic**
across the monorepo. Every language reaches it through the same core, so a frame pose
computed in the backend, the Python SDK, the browser and an edge driver is the
same pose.

## Why this exists

An audit found the same arithmetic reimplemented across the backend, the SDKs,
`cyberwave-robot-format`, `cyberwave-ml`, `cyberwave-sim`, the edge runtime and
nodes, the frontend, the C++ SDK and the Android app. Some of it was
byte-identical — `Quaternion.from_rpy` in
`cyberwave-robot-format/cyberwave_robot_format/math_utils.py` and
`cyberwave-sdks/cyberwave-python/cyberwave/schema.py` are the same twenty
lines. Some of it was subtly different, which is worse: two implementations
that agree on the test case and disagree at gimbal lock.

Solver and rendering stacks stay where they are. pinocchio, frax, MoveIt,
MuJoCo, Three.js and mjlab are external authorities, not duplication. What this
library replaces is the deterministic arithmetic *around* them.

## Layout

```
common/geometry/
├── include/cyberwave/geometry/   public headers (C++ and the C ABI)
├── src/                          implementation
├── tests/                        C++ unit tests + the golden generator
├── golden/                       cross-language golden vectors
└── bindings/
    ├── python/                   ctypes binding over the C ABI
    └── wasm/                     Emscripten build for the frontend
```

| Header          | Contents                                                    |
| --------------- | ----------------------------------------------------------- |
| `types.hpp`     | `Vector3`, `Quaternion`, `Transform`, `Matrix3`, tolerances  |
| `errors.hpp`    | `ErrorCode`, `Error`, `Result<T>`                            |
| `vector.hpp`    | vector arithmetic, axis normalization                        |
| `quaternion.hpp`| algebra plus RPY / axis-angle / matrix conversions           |
| `transform.hpp` | rigid transform composition and inversion                    |
| `geodetic.hpp`  | WGS-84 positions, `GeoPose`, and the ENU/NED/compass adapters |
| `fk.hpp`        | robot tree, chain resolution, frame poses                    |
| `c_api.h`       | the stable C ABI every non-C++ binding sits on               |
| `geometry.hpp`  | umbrella                                                     |

## Design decisions worth knowing

**The core does not parse anything.** Not the Universal Robot Schema, not URDF,
not MJCF, not USD. Each language keeps its own schema reader — that is where the
format quirks and the telemetry policy live — and hands the result over as a
`RobotDescription`. Everything after that point happens once, here.

**Validation happens in the core, parsing happens in the caller.** A
`JointDescription` carries a *raw* origin; `RobotTree::build` normalizes it and
reports the ones it cannot. So the "is this quaternion usable" decision is made
identically in every language, while "which JSON key holds it" is not this
library's problem.

**The core is unbatched.** `compute_frame_poses` takes several frames of one
robot in a single call, which is FFI amortization, not vectorization. The
tensor/vectorized path in `cyberwave-rl` is an explicit exception and continues
to use mjlab and native tensor helpers.

**No dependencies, ever.** Not Eigen, not fmt, not gtest. Everything that
consumes this — a Python extension, a WASM bundle, an Android native build —
must be able to compile it with nothing but a standard library, so adding a
dependency here is a cross-cutting decision rather than a local one. The
~100-line test registry in `tests/test_support.hpp` and the JSON codec in
`tests/mini_json.hpp` exist for that reason, and are test-only.

## Building

```bash
cmake -S common/geometry -B build/geometry -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/geometry -j
ctest --test-dir build/geometry --output-on-failure
```

Produces `libcyberwave_geometry.a` (the C++ core) and
`libcyberwave_geometry_c.so` (the C ABI). The shared library exports exactly the
`cw_geom_*` entry points `c_api.h` declares and nothing else — verified by
`nm -D`; the `--exclude-libs` link option in `CMakeLists.txt` is what keeps the
C++ symbols from leaking out of a library whose whole point is a minimal
surface.

### C++ consumers

```cmake
find_package(CyberwaveGeometry 0.1 REQUIRED)
target_link_libraries(my_target PRIVATE Cyberwave::Geometry)
```

Include `cyberwave/geometry/geometry.hpp` and skip the C ABI entirely.

### Python

```bash
pip install -e common/geometry/bindings/python
export CYBERWAVE_GEOMETRY_LIBRARY=/path/to/libcyberwave_geometry_c.so
```

The binding is **ctypes over the C ABI**, not a compiled extension. That is a
deliberate deviation from the original plan: it keeps `pip install` working on
every platform with no wheel matrix and no compiler, which is what makes
migrating the backend, the SDK and the edge nodes onto it tractable at all. The
C ABI is the seam a compiled accelerator would slot into later without changing
a line above `_native.py`. Library discovery order is
`CYBERWAVE_GEOMETRY_LIBRARY` → bundled beside the package →
`CYBERWAVE_GEOMETRY_BUILD_DIR` → the platform loader's own search path.

```bash
cd common/geometry/bindings/python && python -m pytest
```

### Publishing the Python wheel

The package is **not on PyPI**. Prereleases go to the internal Buildkite
registry (`cyberwave-internal-python`), built by the `prerelease-wheels` and
`publish-prerelease-wheels` jobs in
[`geometry-core.yml`](../../.github/workflows/geometry-core.yml): a push to
`dev` or `staging` that touches `common/geometry/**`, or a manual
`workflow_dispatch`.

Two things make this more than `python -m build`:

* **The wheel must be a manylinux wheel.** Built on a plain runner it is tagged
  `linux_x86_64`, and pip will not install that tag from an index. The build
  runs inside `quay.io/pypa/manylinux_2_28_*` and finishes with `auditwheel
  repair`, via
  [`build_geometry_manylinux_wheels.sh`](../../.github/scripts/build_geometry_manylinux_wheels.sh).
* **`cibuildwheel` does not fit.** It copies only the package directory into
  its build container, and this package cannot build from `bindings/python`
  alone — `setup.py` compiles the core from `common/geometry`, two levels up.
  The script mounts the whole repo instead, which is also why it builds from a
  copy: setuptools writes `egg-info` beside `setup.py`, and a developer's stale
  `build/` tree would otherwise hand CMake a cache generated for another path.

One wheel per CPython minor (3.10-3.13) per architecture (`x86_64`,
`aarch64` under QEMU). `setup.py` forces a platform tag and `bdist_wheel` then
also stamps the ABI, so the wheels are `cp3XX`-specific even though the binding
is pure ctypes.

**Versioning gotcha:** the computed dev version is `<base>.devN`, which is
*lower* than `<base>` under PEP 440 and so does **not** satisfy the
`cyberwave-geometry>=0.1.1,<0.2.0` pin its consumers declare. Cutting a version
those pins accept means either passing an explicit `version` to the
`workflow_dispatch`, or bumping the base version in `pyproject.toml` first.

There are no macOS wheels yet. A Mac resolves the dependency by building from
the checkout — `.github/actions/setup-geometry-core`, or
`.github/scripts/install_sdk.sh`, both of which install it before the SDK so
pip never reaches for an index.

### WASM

See [`bindings/wasm/README.md`](bindings/wasm/README.md). The Emscripten SDK is
not part of this repo's toolchain, so the build runs in the official image; CI
does it in the `wasm` job of `.github/workflows/geometry-core.yml`, which then
replays the golden vectors through the module. It has no `package.json` or
TypeScript types yet, and is not wired into the frontend build.

## Golden vectors

[`golden/geometry_golden.json`](golden/geometry_golden.json) holds 420 cases
covering quaternion algebra, every conversion path, slerp and nlerp (including
the antipodal and near-parallel branches), degenerate inputs, transform
composition, geodetic conversion against four anchors (including a southern-
hemisphere one and one on the antimeridian), and FK across five robot
descriptions including mimic joints, prismatic joints, rotated origins, sensor
frames and four structural failure modes.

Every language's conformance runner reads that one file:

| Language        | Runner                                                    | Status     |
| --------------- | ---------------------------------------------------------- | ---------- |
| C++             | `ctest -R golden_vectors` (`tests/golden_gen.cpp`)           | passing    |
| Python          | `bindings/python/tests/test_golden.py`                      | passing    |
| TypeScript/WASM | `node bindings/wasm/tests/run_golden.mjs`                    | passing    |
| Kotlin          | not written                                                 | **to do**  |

Comparison is numeric against the tolerance declared in the file, not
byte-exact — see `CONVENTIONS.md` §8 for why.

**Regenerating**, after an intentional behaviour change:

```bash
build/geometry/tests/golden_gen --write common/geometry/golden/geometry_golden.json
```

Review the diff. A change you did not intend is the whole point of the file.

## What is tested

| Suite | Cases | Covers |
| ----- | ----: | ------ |
| `test_quaternion` | 45 | algebra, every conversion path, slerp/nlerp, degenerate and non-finite input, all four Shepperd branches, both gimbal-lock poles and the band just outside them |
| `test_transform` | 15 | composition, associativity, inversion, the rotate-then-translate order, axis normalization |
| `test_fk` | 39 | tree construction, base-frame resolution, chain resolution, origin-before-motion, mimic joints, and every structural failure mode |
| `test_geodetic` | 23 | the ENU frame, the anchor heading, tangent-plane round trips, the antimeridian, compass and NED adapters, and the polar/out-of-range refusals |
| `test_c_api` | 21 | handle lifetimes, null tolerance, and agreement with the C++ core |
| `golden_vectors` | 420 | the shared conformance file |
| Python `test_golden.py` | 420 | the same file, through the binding |
| Python `test_binding.py` | 33 | handle lifetimes, the strict/lenient split, component-order discipline |
| Python `test_geodetic.py` | 24 | ABI field order, the wire spelling, and the strict/lenient split for coordinates |

The C++ suites also pass under `-fsanitize=address,undefined` with leak
detection on, which is what actually checks the builder/tree/pose handle
balance in the C ABI.

`bindings/python/tests/benchmark.py` measures the FFI cost — around 1 µs for a
scalar op and 99 µs for every frame of a 12-joint arm, so a telemetry tick is
well under a millisecond and the unbatched core is not the constraint. It is a
script rather than a test on purpose: a timing assertion in CI is a flake
generator.

## Migration status

The core, the C ABI, the platform wheel, the Python binding, the golden vectors
and the conformance harness are done and tested. **`cyberwave-backend`,
`cyberwave-robot-format`, the Python SDK and `cyberwave-ml` are migrated** and
report zero findings; the backend images build and install the core.

Still to do: the edge nodes and runtime, the simulator, the CLIs, the demos,
the C++ SDK, and the frontend (which needs the WASM build first).

The geodetic module (§9) is new and **nothing is migrated onto it yet**. It was
written against the three existing implementations rather than in place of them,
and reproduces `cyberwave-backend/src/lib/geo_transform.py` to within 1e-13 m at
site ranges, so the migrations are call-site changes:

| Copy | Status |
| ---- | ------ |
| `cyberwave-backend/src/lib/geo_transform.py` | numerically identical; migrate first, it is the reference |
| `cyberwave-frontend/lib/utils/geo-transform.ts` | numerically identical; needs the WASM build, and its "keep in sync" comment goes away with it |
| `offsetWgs84` in `cyberwave-edge-nodes/…/CameraPoseTransform.kt` | uses the spherical constant, so migrating it **changes numbers** by its documented 0.67%; needs the Kotlin runner, which is still to do |

The Kotlin driver also hand-rolls `nedAttitudeToEnuQuaternion`, which is now
`geo::quat_from_ned_rpy`, and carries the open question about DJI's real
gimbal-to-optical rotation — that question is unaffected by this and stays where
it is documented.
`scripts/check_geometry_duplication.py` reports what remains,
`scripts/geometry_duplication_baseline.json` carries a per-component removal
target, and [`docs/MIGRATION.md`](docs/MIGRATION.md) has the reasoning —
including why the wheel is built in a manylinux container rather than by
`cibuildwheel`. The SDK now declares the dependency, so a release of it needs
the wheel published (see *Publishing the Python wheel* above) and
`poetry.lock` regenerated against the registry that serves it.

### A caution about the checker

It matches formulas and declarations it knows about. A rotation expressed some
other way — assembled column by column, or routed through a library call —
passes it silently. A clean run means "no known pattern reappeared", not "there
is no duplicated geometry left". It gained rules for slerp and for re-declared
operations partway through this migration and immediately found 30 more
findings that had been there all along.
