#!/usr/bin/env bash
# Build the geometry core to WASM for the frontend.
#
# Requires an activated emsdk (`source /path/to/emsdk/emsdk_env.sh`). The SDK is
# not part of this repo's toolchain, so the reproducible way to run this is the
# official image, from common/geometry:
#
#     docker run --rm -v "$PWD":/src -w /src emscripten/emsdk:3.1.64 \
#         bash bindings/wasm/build.sh
#     node bindings/wasm/tests/run_golden.mjs
#
# The golden runner replays all 269 cases of golden/geometry_golden.json through
# the build; it is what proves this artifact agrees with the C++ and Python ones.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
core="${here}/../.."
out="${here}/dist"

if ! command -v emcc >/dev/null 2>&1; then
    echo "emcc not found. Activate the Emscripten SDK first:" >&2
    echo "    source /path/to/emsdk/emsdk_env.sh" >&2
    exit 1
fi

mkdir -p "${out}"

# MODULARIZE + EXPORT_ES6 so the frontend can `await createGeometryModule()`
# rather than depending on a global, which Next.js will not tolerate.
# ALLOW_MEMORY_GROWTH is off on purpose: the core allocates only small robot
# trees, and a fixed heap makes an accidental leak fail loudly instead of
# quietly growing a browser tab.
emcc \
    "${core}/src/errors.cpp" \
    "${core}/src/geodetic.cpp" \
    "${core}/src/quaternion.cpp" \
    "${core}/src/transform.cpp" \
    "${core}/src/fk.cpp" \
    "${core}/src/version.cpp" \
    "${here}/src/embind.cpp" \
    -I"${core}/include" \
    -std=c++20 \
    -O3 \
    -flto \
    --bind \
    -sMODULARIZE=1 \
    -sEXPORT_ES6=1 \
    -sEXPORT_NAME=createGeometryModule \
    -sENVIRONMENT=web,worker \
    -sALLOW_MEMORY_GROWTH=0 \
    -sINITIAL_MEMORY=16MB \
    -sDISABLE_EXCEPTION_CATCHING=0 \
    -sFILESYSTEM=0 \
    -o "${out}/cyberwave_geometry.mjs"

echo "wrote ${out}/cyberwave_geometry.mjs and .wasm"
