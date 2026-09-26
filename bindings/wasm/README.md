# WASM binding

The Emscripten SDK is not part of this repository's toolchain, so the build runs
in the official image. From the repository root:

```bash
docker run --rm -v "$PWD":/src -w /src emscripten/emsdk:3.1.64 \
    bash bindings/wasm/build.sh
node bindings/wasm/tests/run_golden.mjs
```

The runner replays all 269 cases of
[`../../golden/geometry_golden.json`](../../golden/geometry_golden.json) — the
same file the C++ and Python suites read — and currently passes every one at the
file's `1e-12` tolerance. CI runs both steps in the `wasm` job of
`.github/workflows/geometry-core.yml`, so a formula that drifts here and nowhere
else still fails a check.

## What is here

| File                     | State                                                    |
| ------------------------ | -------------------------------------------------------- |
| `src/embind.cpp`         | embind surface over the core                              |
| `build.sh`               | emcc invocation                                           |
| `tests/run_golden.mjs`   | conformance runner, 269 cases / 1075 assertions           |
| `dist/`                  | build output, gitignored                                  |
| `package.json`           | **missing**                                               |
| TypeScript types         | **missing** — embind does not generate them               |

## Still to do

1. Add `package.json` and hand-written `.d.ts` types.
2. Wire the build into the frontend's pipeline and weigh the `.wasm` payload
   (~140KB, ~55KB of JS glue) against what it saves. If that is not worth it for
   the frontend's six findings, say so and close this out rather than shipping
   it — the binding being correct is not by itself a reason to load it.

## Design notes

* Values cross as plain objects with **named** fields (`{w, x, y, z}`,
  `{x, y, z}`), matching the golden file and the Python binding.
* Errors are **thrown**, not returned as a status, so frontend call sites use
  `try`/`catch` and the strict/lenient split stays visible. What is thrown is a
  real `Error` (so `instanceof Error` and a stack both work) carrying `name:
  "CyberwaveGeometryError"` plus `code`, `codeValue`, `subject` and `detail`.
  Note `throw payload` in C++ does **not** do this — a C++-thrown `val` reaches
  JS as the raw exception pointer, a bare number, and every `e.code` reads
  `undefined`. `val::throw_()` is the supported call; the golden runner caught
  exactly this, on 32 cases whose values were already correct.
* `RobotTree` is an embind class and embind does **not** garbage collect: JS
  must call `.delete()` on it or leak WASM heap, which `build.sh` deliberately
  fixes at 16MB so a leak fails loudly. A `using`-style helper in the TypeScript
  wrapper is worth writing.
* The module is built for `web,worker` only. Node can therefore not `fetch` the
  `.wasm` off disk; the golden runner passes the bytes as `wasmBinary` instead,
  so it tests the exact artifact the frontend would ship rather than a
  Node-flavoured rebuild.
* Three.js keeps its scene-graph APIs for rendering. Only *deterministic saved-
  pose* FK should route through here — see
  [`../../CONVENTIONS.md`](../../CONVENTIONS.md) §2 for why the Euler orders
  differ and must not be crossed.
