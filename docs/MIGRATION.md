# Migrating onto the shared geometry core

**Internal document.** Tracks what has moved onto `common/geometry` and what
has not. The machine-readable version of "what is left" is
`scripts/check_geometry_duplication.py`; this file is the reasoning around it.

## Status

| Step | Work | State |
| ---- | ---- | ----- |
| 1 | C++ core, C ABI, golden vectors, C++ + Python conformance | **done** |
| 1a | Self-contained platform wheel (`pip install`, no compiler) | **done** |
| 1b | Backend image builds and installs the core; import smoke test | **done** |
| 1c | WASM binding, built and golden-verified in CI | **done** — not wired into the frontend |
| 1d | Kotlin/JNI binding | **not started** |
| 2 | Backend telemetry FK (`services/telemetry/fk.py`) | **blocked** — not on `dev` |
| 3a | Backend transform / navigation / procedural / sensor / robot-export | **done** — maths migrated; 2 positional-construction findings remain in `robot_export/scene_schema_builder.py` |
| 3b | `cyberwave-robot-format` | **done** — maths migrated; 10 positional-construction findings remain (`mjcf/parser.py` + test fixtures) |
| 3c | Python SDK (schema, twin, placement, fusion) | **done** — C++ SDK remains |
| 3d | `cyberwave-ml`, simulator, CLIs, demos, e2e | **done** — 0 findings |
| 3e | Edge nodes, edge runtime, cloud nodes | **blocked** — see below |
| 4 | Frontend | **not started** — WASM exists; wiring and payload call still open |
| 5 | C++ SDK and Kotlin call sites | **blocked** — see below |
| 3f | Geodetic (`geo_transform.py`, `geo-transform.ts`, Kotlin `offsetWgs84`) | **not started** — module landed, see below |
| 6 | Allowlist for deliberate exceptions | **done** (`geometry-core-exempt`, `-file`, allowlist) |
| 7 | Repository check, two CI modes | **done** |

### Geodetic (step 3f)

`geodetic.hpp` landed in core 0.1.2 with `Geodetic`, `GeoPose`, `GeoAnchor` and
151 golden cases. The convention is `CONVENTIONS.md` §9; nothing is migrated onto
it yet.

Three implementations of the same arithmetic exist today, two of which are kept
in agreement only by a `Keep in sync with` comment at the top of each file:

| Copy | Numerics vs the core | Migration |
| ---- | -------------------- | --------- |
| `cyberwave-backend/src/lib/geo_transform.py` | identical to 1e-13 m at site ranges | pure call-site change; do this one first, it is the reference the core was written against |
| `cyberwave-frontend/lib/utils/geo-transform.ts` | identical (it is a hand-port of the above) | needs step 1c wired into the frontend build |
| `offsetWgs84` in `cyberwave-edge-nodes/cyberwave-edge-dji-mini-android/.../CameraPoseTransform.kt` | **differs** — spherical constant, +0.67%/-0.29% scale error | needs step 1d; migrating it *changes published positions*, so it wants its own PR and a release note |

Two things the core deliberately did **not** take over, so a migration does not
have to invent them:

* **The distance guard.** `MAX_GEO_ANCHOR_DISTANCE_M` is a product decision
  ("this fix is a bug, not a journey"), differs per caller, and stays in the
  backend and the frontend. The core exposes `ground_distance` and no threshold.
* **Anchor envelope validation.** `collect_geo_anchor_violations` also checks
  `contract_version`, `anchor_status` and `altitude_datum` — payload concerns,
  not geometry. The core validates only what it needs to compute: finite, in
  range, and off the poles. A migrated backend keeps its validator and calls the
  core underneath it.

### What the checker says

Down from 304 findings across 113 files to **154**. `cyberwave-ml`,
`cyberwave-sim`, the CLIs, `cyberwave-rl`, `cyberwave-demos` and `e2e` are all
at zero; `cyberwave-backend` and `cyberwave-robot-format` have no hand-written
maths left as findings, only positional quaternion construction -- plus one
exempted copy, the temporary singular-rotation guard in the backend's
`schema_joint_geometry.py`, which goes when the deployed robot-format floor
reaches the wheel carrying the fixed `Quaternion.to_rpy` (CYB-3869). The
baseline in `scripts/geometry_duplication_baseline.json` carries a per-component
`migration_plan` with a removal target for everything that remains, and the
checker recomputes its counts from the entry list on every `--write-baseline`,
so the plan cannot drift from the list it describes. Rather than trusting the
number in this paragraph, run the checker.

What is left, and why each is blocked rather than merely undone:

| Component | Findings | Blocker |
| --------- | -------- | ------- |
| `cyberwave-edge-nodes` | 68 | Deployed to devices from published wheels; no aarch64 wheel exists yet |
| `cyberwave-edge-runtime` | 42 | Same |
| `cyberwave-sdks/cyberwave-cpp` | 7 | Would add a link dependency to the SDK's `install(EXPORT)` contract, which downstream consumers resolve via `find_package` — and the core is not published or open-sourced, so the mirror could not build |
| `cyberwave-sdks/cyberwave-python` | 16 | 10 are positional `Quaternion(...)` construction rather than maths (8 `test_data_fusion.py` fixtures to exempt, 2 call sites to convert); the other 6 are real maths in `calibration/frames.py` and its test, which arrived from `dev` — see the baseline's note for what has to be checked before `to_matrix`/`from_matrix` can take it over |
| `cyberwave-robot-format` | 10 | Same shape: `mjcf/parser.py` plus test fixtures. The maths is migrated |
| `cyberwave-frontend` | 6 | Needs the WASM module wired into the Next.js build; weigh the ~140KB payload against six findings first |
| `cyberwave-cloud-nodes` | 3 | Ships as a deployed node depending on published wheels |
| `cyberwave-backend` | 2 | Positional construction in `robot_export/scene_schema_builder.py` |

That is 154, matching `entries` in the baseline; the `migration_plan` beside
it is recomputed from the same list, so those two agree by construction rather
than by transcription. This table is still transcribed by hand, so treat it as
a snapshot and let the checker settle any disagreement.

Every one of these is the same underlying problem: **the core is published to
no index**. Declaring it anywhere that installs from an index fails outright —
that is what broke five CI checks once already. Publishing the wheels unblocks
four of the five rows at a stroke.

The count is not monotonic, and should not be read as one: it grew by 30 when
the checker gained rules for slerp weights and for re-declared operations.
Those were always there. A checker finding more is progress, not regression —
which is why `--write-baseline` refuses to add entries unless you pass
`--accept-new` and say why.

It also grows on a merge, for as long as this gate lives on a branch and not on
`dev`. Merging `dev` brought 25 findings written while nothing was checking:
`cyberwave-edge-runtime`'s new intelligence services and the SDK's
`calibration/frames.py` went into the baseline with `--accept-new`, and the ten
that were deliberate — three test helpers that must not call the code they pin,
and the temporary singular-rotation guard in the backend's
`schema_joint_geometry.py` — took a `geometry-core-exempt` comment instead. The
same 16 that left were `dev` deleting the legacy Go2 runtimes. Landing the gate
on `dev` is what stops this recurring.

**The checker is not proof that all quaternion maths is gone.** It matches
formulas and declarations it knows about. Anything expressed differently — a
rotation built through a library call, a matrix assembled column by column —
passes it silently. Treat a clean run as "no *known* pattern reappeared", not
as a completion certificate.

## Packaging: settled for the backend, open for the SDK

`common/geometry/bindings/python` now builds a **self-contained platform
wheel**: `setup.py` drives CMake and bundles `libcyberwave_geometry_c` beside
the package, so `pip install cyberwave_geometry-*.whl` works in a bare venv
with no compiler and no `CYBERWAVE_GEOMETRY_LIBRARY`. That is what unblocked
everything below.

* **Backend — done.** Both `compose/local/django/Dockerfile` and
  `compose/production/django/Dockerfile` build the core in the stage that
  already carries CMake, copy the `.so` into the run stage, install the
  binding, and set `CYBERWAVE_GEOMETRY_LIBRARY`. The build fails rather than a
  request if the core cannot be imported, and
  `src/lib/tests/test_geometry_core_available.py` covers it from the test
  suite. `local.yml`, `test.yml` and the four `docker buildx` invocations in
  `backend-test-and-release.yml` pass the build context.

* **`cyberwave-robot-format` — migrated, and now depends on
  `cyberwave-geometry`.** ⚠️ **This package is published to PyPI. The
  dependency must be published before the next robot-format release**, or
  `pip install cyberwave-robot-format` will fail to resolve. Inside the
  monorepo it already works, because the backend image installs both.

* **Python SDK — migrated, same caveat.**

* **Still to do: a `cibuildwheel` matrix** (manylinux, macOS arm64 + x86_64,
  Windows) so the wheel exists for every platform an SDK user installs on.
  Deliberately no pure-Python fallback: a fallback would be a second
  implementation of the maths, which is the thing being removed. An
  unsupported platform should fail at install time and say so.

## The rule until the wheel is published

`cyberwave-geometry` is on no index. That single fact caused five red CI checks
on the first PR, in five different ways, and the rule that came out of it is:

**Do not declare it as a dependency, and do not import it at module scope, in
anything a consumer can install without the core present.**

* **Declaring it breaks installation outright** — `pip install
  cyberwave-robot-format` fails with *"Could not find a version that satisfies
  the requirement cyberwave-geometry (from versions: none)"*, and `poetry
  install` for the SDK fails the same way at resolution. Deferred in both, with
  the reason written where the declaration would go.

* **Importing it at module scope breaks `import cyberwave` itself.** The chain
  `cyberwave/__init__` → `compact` → `client` → `data` → `fusion` reaches it on
  every import, so a camera-driver E2E that never touches geometry died on it.
  Both the SDK and `cyberwave-robot-format` now go through a `_geometry.py`
  accessor that imports on first *use*.

  This is not a fallback. There is no second implementation and there will not
  be one; the failure just moves to the point where rotation maths is actually
  requested, with a message naming what to install.

* **The rule has no exceptions, including for internal packages.** It first
  read "where the dependency is intrinsic, keep it and install the core first",
  on the grounds that `cyberwave-rl`, `cyberwave-sim`, `cyberwave-ml` and the
  MCP server control their own images. That was wrong within the hour: the sim
  image's `pip install -r requirements.txt` and the MCP server's integration
  script both install the package *without* installing the core first, and both
  died on *"from versions: none"*. Every declaration is now deferred, each with
  the error you get if you put it back. What replaces it: cyberwave-sim builds
  the core in its Dockerfile, the MCP server gained a `_geometry.py` accessor,
  and cyberwave-rl and the collision pipeline rely on their workflows running
  `setup-geometry-core`.

  The accessor alone stopped being enough once `test-mcp-server.sh` started
  running the MCP unit tests inside its own pip venv: a lazy import still has
  to resolve when a test exercises rotation maths, and nine did. That script
  now installs the core into the venv itself, through the same
  `install_geometry_core.sh` the workflows use, before it installs the SDK.

* **Installing it from a checkout has its own three failure modes**, all of
  which turned up in CI, and all of which now live in one place —
  `.github/scripts/install_geometry_core.sh`, which both
  `.github/actions/setup-geometry-core` and `.github/scripts/install_sdk.sh`
  call:

  1. *The checkout may be read-only.* The Debian package builds mount the
     workspace at `/repo-root:ro`, and the binding builds with setuptools,
     whose `get_requires_for_build_wheel` hook writes `*.egg-info` beside
     `setup.py` — *"could not create 'cyberwave_geometry.egg-info': Read-only
     file system"*, before it compiled anything. So the install runs from a
     `tar`-copy in a temp directory, never from the checkout. That also keeps
     build trees out of the checkout, which is what stopped a stale
     `CMakeCache.txt` travelling into later image builds on reused self-hosted
     workspaces. (The SDK itself installs fine from a read-only mount: it
     builds with poetry-core, which writes nothing into the source tree.)
  2. *An old **pip** builds it against a setuptools that cannot read the
     metadata.* PEP 621's `[project]` table needs setuptools >= 61 and
     `[build-system] requires` already asks for >= 68 — but with Ubuntu's
     distro pip (22.0.2 on 22.04), build isolation installs the modern
     setuptools and the build still runs against the distro's 59.6.0. The
     result is an unnamed, empty `UNKNOWN-0.0.0` wheel — 973 bytes, no package
     inside — that pip installs and reports as success, so the failure surfaces
     much later as a bare `ModuleNotFoundError`.

     Which half matters was measured in `ubuntu:22.04` against this package,
     because the obvious guess is wrong:

     | environment | result |
     | --- | --- |
     | distro pip + distro setuptools | `UNKNOWN-0.0.0`, 973 bytes |
     | distro pip + setuptools 84 | `UNKNOWN-0.0.0`, 973 bytes |
     | pip 26 + setuptools 84 | `cyberwave-geometry 0.1.1`, 170 KB |

     So the installer upgrades **pip** (a setuptools upgrade alone does
     nothing) and then *asserts the installed distribution is named
     `cyberwave-geometry`*, because pip's exit status does not say.
  3. *`python` may not exist.* Neither ubuntu:22.04 nor the self-hosted runners
     have it (no `python-is-python3`); the verification step died with
     `python: command not found`, exit 127, after a successful install. The
     installer resolves an interpreter and drives pip through it, so what gets
     installed and what gets imported cannot land in different environments.

  These were two copies of the same logic for a while — the action grew the
  setuptools guard and the script did not — which is how the same empty install
  kept coming back through the other door. `.github/scripts/tests/test_install_geometry_core.py`
  now fails if either caller grows its own copy.

Once the wheels are published, the declarations go back and the lazy accessors
can stay — they cost one cached call and keep a native library off the import
path of consumers that do not need it.

## Step 2 is not actionable on `dev`

The plan calls for migrating `cyberwave-backend/src/app/services/telemetry/fk.py`
so that it keeps schema parsing and telemetry policy but delegates transforms
and traversal to the core. **That file does not exist on `dev`.** It lives on
`tmachacek/cyb-3787-frame-pose-telemetry-and-structured-collection-of-arbitrary`
(commit `7cfdd9dea7`), unmerged.

That branch is nonetheless the design source for `fk.hpp`: the core's joint
semantics, base-frame resolution, chain resolution, mimic handling and error
taxonomy are a direct port of it, so the migration when that branch lands is
mechanical — delete the arithmetic, keep the parsing, call `RobotTree`.

One deliberate difference: `fk.py` reports an unknown frame as `no_links`.
The core has a distinct `unknown_frame` for it. If the persisted payload's
`code` field has to stay byte-compatible, map it back in the adapter rather
than degrading the core's taxonomy.

## The starting survey

This is the **original** picture, kept because the reasoning below is still
what shaped the migration order. For the current count see *What the checker
says* above, or just run the checker.

304 findings across 113 files, from the guard. By component:

| Component | Findings | Files |
| --------- | -------: | ----: |
| `cyberwave-edge-nodes` | 64 | 24 |
| `cyberwave-demos` | 56 | 26 |
| `cyberwave-edge-runtime` | 49 | 18 |
| `cyberwave-sim` | 45 | 12 |
| `cyberwave-backend` | 40 | 13 |
| `cyberwave-sdks` | 13 | 5 |
| `cyberwave-robot-format` | 12 | 3 |
| `cyberwave-ml` | 9 | 3 |
| `cyberwave-clis` | 6 | 2 |
| `cyberwave-frontend` | 6 | 5 |
| `cyberwave-cloud-nodes` | 3 | 1 |
| `e2e` | 1 | 1 |

32 of those are in test files, where an independent reimplementation is often
the *point* — a test that checks the core by calling the core checks nothing.
Those should be exempted with a reason, not migrated.

### Two findings worth acting on first

**Both are now resolved**: `from_rpy` was migrated in the first pass, and the
edge copies are what the remaining edge-nodes / edge-runtime counts consist of.

**`Quaternion.from_rpy` is duplicated byte-for-byte** in
`cyberwave-robot-format/cyberwave_robot_format/math_utils.py`,
`cyberwave-robot-format/cyberwave_robot_format/schema.py` and
`cyberwave-sdks/cyberwave-python/cyberwave/schema.py`. The core's `from_rpy`
matches all three exactly — verified against the golden vectors — so these are
behaviour-preserving swaps and the natural first migration once packaging is
settled. *(Done.)*

**Much of the edge count is file-level copies, not formula duplication.** The
Go2 driver nodes exist three times over — under
`cyberwave-edge-nodes/.../humble-jetson/`, `cyberwave-edge-nodes/.../jazzy/` and
`cyberwave-edge-runtime/runtime-services/...` — with the same
`amcl_bootstrap_node.py` and `image_collector_node.py` in each. Pointing all of
them at the core is worth doing, but the larger problem there is that the files
are copies at all, which is outside this library's scope.

## Running the check

```bash
python3 scripts/check_geometry_duplication.py          # fail on new findings
python3 scripts/check_geometry_duplication.py --all    # the full inventory
```

`scripts/geometry_duplication_baseline.json` records the findings that predate
the core, so CI fails only on *new* duplication while the migration proceeds.
The file's own `$detection_note` is the authority on the count, which moves
whenever the checker learns to see a formula it previously missed. Entries should only ever be removed. As each call site moves over,
drop its line — the baseline shrinking is the progress bar.

For duplication that is deliberate — an independent reference implementation in
a test, most often — annotate the line instead:

```python
# geometry-core-exempt: independent reference for the conformance test
```
