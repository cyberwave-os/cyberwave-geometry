#!/usr/bin/env bash
# Build manylinux wheels for cyberwave-geometry.
#
# Runs INSIDE a quay.io/pypa/manylinux_2_28_* container, with the repository
# mounted read-only at /io and an output directory mounted at /out:
#
#   docker run --rm \
#     -v "$PWD":/io:ro -v "$PWD/wheelhouse":/out \
#     quay.io/pypa/manylinux_2_28_x86_64 \
#     bash /io/common/geometry/.github/scripts/build_manylinux_wheels.sh
#
# The same script runs in the public cyberwave-os/cyberwave-geometry repo,
# where the checkout root is this tree rather than the monorepo.
#
# Why a container rather than cibuildwheel: cibuildwheel copies only the
# package directory into its build container, and this package cannot build
# from bindings/python alone -- setup.py compiles the core from the geometry
# tree, which is two levels up. Mounting the whole tree is what makes the
# source build possible at all.
#
# Why manylinux rather than a plain ubuntu runner: a wheel built on a stock
# runner is tagged linux_x86_64, and pip refuses to install that tag from an
# index. auditwheel repair below is what turns it into an installable
# manylinux wheel.
#
# Environment:
#   OUT      - output directory for the repaired wheels (default /out)
#   PYTHONS  - space-separated CPython tags to build for. One wheel per tag:
#              setup.py forces a platform tag, and bdist_wheel then also
#              stamps the ABI, so the wheels are cp3XX-specific in practice.
set -euo pipefail

# Build from a copy, not from the mounted checkout: setuptools writes egg-info
# next to setup.py (which fails outright on a read-only mount), and a
# developer's stale build/ tree would otherwise travel in and hand CMake a
# cache generated for another path.
if [ -f /io/CMakeLists.txt ]; then
  SRC=/io
elif [ -f /io/common/geometry/CMakeLists.txt ]; then
  SRC=/io/common/geometry
else
  echo "geometry sources not found under /io" >&2
  exit 1
fi
CORE=/tmp/geometry-src
rm -rf "$CORE"
mkdir -p "$CORE"
tar -C "$SRC" --exclude=build --exclude='*.egg-info' -cf - . | tar -C "$CORE" -xf -

OUT=${OUT:-/out}
PYTHONS=${PYTHONS:-"cp310-cp310 cp311-cp311 cp312-cp312 cp313-cp313"}

# cmake is not guaranteed in the manylinux image; take it from pip so the
# version comes from the image's Python rather than the base OS.
export PATH="/opt/python/cp312-cp312/bin:$PATH"
pip install --no-cache-dir --quiet cmake ninja

# Build the core once and hand it to every per-Python wheel build, rather than
# recompiling it four times.
cmake -S "$CORE" -B /tmp/geometry-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCYBERWAVE_GEOMETRY_BUILD_TESTS=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build /tmp/geometry-build -j"$(nproc)"

# Straight from the build tree rather than `cmake --install`: GNUInstallDirs
# puts the library in lib64 on these RHEL-based images and lib on Debian-based
# ones, and nothing here needs it on the loader path.
CORE_LIB=/tmp/geometry-build/libcyberwave_geometry_c.so
test -f "$CORE_LIB"

mkdir -p /tmp/raw "$OUT"
for py in $PYTHONS; do
  echo "=== building for ${py} ==="
  CYBERWAVE_GEOMETRY_PREBUILT_LIBRARY="$CORE_LIB" \
    "/opt/python/${py}/bin/pip" wheel --no-deps -w /tmp/raw "$CORE/bindings/python"
done

for w in /tmp/raw/*.whl; do
  echo "=== auditwheel repair $(basename "$w") ==="
  auditwheel repair "$w" -w "$OUT"
done

echo "=== wheels ==="
ls -1 "$OUT"

# Prove the published artefact actually works, here, rather than finding out
# when something tries to install it. The wheel carries the core inside the
# package, so this needs no CYBERWAVE_GEOMETRY_LIBRARY in the environment.
echo "=== install and import check ==="
/opt/python/cp312-cp312/bin/pip install --no-index --find-links "$OUT" cyberwave-geometry
/opt/python/cp312-cp312/bin/python -c \
  "import cyberwave_geometry as g; print('geometry core', g.core_version(), 'from', g._native.library_path)"
