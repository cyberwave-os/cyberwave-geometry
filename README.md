# Cyberwave Geometry

A dependency-free C++20 library that is the single executable source of truth
for **scalar quaternion, transform, forward-kinematics and geodetic
arithmetic**. Every language binding reaches the same core, so a frame pose
computed in a server, in Python, in a browser and on a robot is the same pose.

Apache-2.0 licensed. The Python binding is published as
[`cyberwave-geometry`](https://pypi.org/project/cyberwave-geometry/).

## Why this exists

This arithmetic is easy to write and easy to write *slightly* differently. Two
implementations agree on the test case and disagree at gimbal lock, or at the
antimeridian, or on which of `xyzw` and `wxyz` a four-element array means. The
disagreement then surfaces as a robot in the wrong place, with nothing in the
logs to say which side was wrong.

So there is one implementation, in C++, and every other language calls it
through a stable C ABI. The golden vectors below are what turns "the languages
agree" into a tested claim rather than an assumption.

Solver and rendering stacks stay where they are. pinocchio, MoveIt, MuJoCo and
Three.js are external authorities, not duplication. What this library replaces
is the deterministic arithmetic *around* them.

## Layout

```
├── include/cyberwave/geometry/   public headers (C++ and the C ABI)
├── src/                          implementation
├── tests/                        C++ unit tests + the golden generator
├── golden/                       cross-language golden vectors
└── bindings/
    ├── python/                   ctypes binding over the C ABI
    └── wasm/                     Emscripten build for the browser
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

Conventions — component order, frame definitions, tolerances, the geodetic
model — are in [`CONVENTIONS.md`](CONVENTIONS.md).

## Design decisions worth knowing

**The core does not parse anything.** Not URDF, not MJCF, not USD. Each caller
keeps its own schema reader — that is where the format quirks live — and hands
the result over as a `RobotDescription`. Everything after that point happens
once, here.

**Validation happens in the core, parsing happens in the caller.** A
`JointDescription` carries a *raw* origin; `RobotTree::build` normalizes it and
reports the ones it cannot. So the "is this quaternion usable" decision is made
identically in every language, while "which JSON key holds it" is not this
library's problem.

**The core is unbatched.** `compute_frame_poses` takes several frames of one
robot in a single call, which is FFI amortization, not vectorization. Workloads
that need a whole tensor of poses at a time should use a vectorized stack; the
Python binding ships batched torch kernels checked against these same golden
vectors.

**No dependencies, ever.** Not Eigen, not fmt, not gtest. Everything that
consumes this — a Python wheel, a WASM bundle, an Android native build — must be
able to compile it with nothing but a standard library, so adding a dependency
here is a cross-cutting decision rather than a local one. The ~100-line test
registry in `tests/test_support.hpp` and the JSON codec in `tests/mini_json.hpp`
exist for that reason, and are test-only.

## Building

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Requires CMake 3.14+ and a C++20 compiler (GCC 10+, Clang 10+, AppleClang 12+).

Produces `libcyberwave_geometry.a` (the C++ core) and
`libcyberwave_geometry_c.so` (the C ABI). The shared library exports exactly the
`cw_geom_*` entry points `c_api.h` declares and nothing else — verified by
`nm -D` in CI; the `--exclude-libs` link option in `CMakeLists.txt` is what
keeps the C++ symbols from leaking out of a library whose whole point is a
minimal surface.

### C++ consumers

```cmake
find_package(CyberwaveGeometry 0.2 REQUIRED)
target_link_libraries(my_target PRIVATE Cyberwave::Geometry)
```

Include `cyberwave/geometry/geometry.hpp` and skip the C ABI entirely.

### Python

```bash
pip install cyberwave-geometry
```

The wheel carries the core beside the package, so it needs no compiler and no
separate build. See [`bindings/python/README.md`](bindings/python/README.md) for
the API, the component-order rules and the geodetic adapters.

The binding is **ctypes over the C ABI**, not a compiled extension: there are no
formulas on the Python side at all, and the C ABI is the seam a compiled
accelerator would slot into later without changing anything above `_native.py`.
Library discovery order is `CYBERWAVE_GEOMETRY_LIBRARY` → bundled beside the
package → `CYBERWAVE_GEOMETRY_BUILD_DIR` → the platform loader's own search path.

Wheels are published for CPython 3.10–3.14 on manylinux `x86_64` and `aarch64`,
macOS `arm64` and `x86_64`, and Windows `AMD64`. There is no sdist on purpose:
the Python package cannot build from `bindings/python` alone — `setup.py`
compiles the core from the repository root — so an sdist would turn a clean "no
wheel for your platform" into a confusing compile error. On an unsupported
platform, build from a checkout instead:

```bash
pip install ./bindings/python
```

### WASM

See [`bindings/wasm/README.md`](bindings/wasm/README.md). The Emscripten SDK is
not part of this repository's toolchain, so the build runs in the official
`emscripten/emsdk` image. It has no `package.json` or TypeScript types yet.

## Golden vectors

[`golden/geometry_golden.json`](golden/geometry_golden.json) holds 420 cases
covering quaternion algebra, every conversion path, slerp and nlerp (including
the antipodal and near-parallel branches), degenerate inputs, transform
composition, geodetic conversion against four anchors (including a southern-
hemisphere one and one on the antimeridian), and FK across five robot
descriptions including mimic joints, prismatic joints, rotated origins, sensor
frames and four structural failure modes.

Every language's conformance runner reads that one file:

| Language        | Runner                                                      |
| --------------- | ----------------------------------------------------------- |
| C++             | `ctest -R golden_vectors` (`tests/golden_gen.cpp`)           |
| Python          | `bindings/python/tests/test_golden.py`                       |
| TypeScript/WASM | `node bindings/wasm/tests/run_golden.mjs`                    |

Comparison is numeric against the tolerance declared in the file, not
byte-exact — see `CONVENTIONS.md` §8 for why.

**Regenerating**, after an intentional behaviour change:

```bash
build/tests/golden_gen --write golden/geometry_golden.json
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
detection on, which is what actually checks the builder/tree/pose handle balance
in the C ABI.

`bindings/python/tests/benchmark.py` measures the FFI cost — around 1 µs for a
scalar op and 99 µs for every frame of a 12-joint arm, so a telemetry tick is
well under a millisecond and the unbatched core is not the constraint. It is a
script rather than a test on purpose: a timing assertion in CI is a flake
generator.

## Versioning

The version in `CMakeLists.txt` and `bindings/python/pyproject.toml` is what a
conformance-tested binding pins against. The minor version is bumped whenever
the C ABI or a binding's surface changes, so a consumer fails at its version pin
rather than at a call site with a missing symbol.

## License

Apache-2.0. See [`LICENSE`](LICENSE).
