"""Build the geometry core with CMake and bundle it into the wheel.

The Python package is pure ctypes, but the wheel is *not* pure Python: it
carries ``libcyberwave_geometry_c`` beside the package so that installing it
gets you a working core with no separate build step and no compiler on the
target machine. That is what makes it possible for the backend, the SDK,
``cyberwave-robot-format`` and the edge nodes to depend on it.

Consequences, both deliberate:

* the wheel is tagged for a platform, not ``py3-none-any``, so a release needs
  one build per platform (cibuildwheel);
* an unsupported platform fails loudly at install time rather than silently
  falling back to a second, drifting Python implementation of the same maths.

``pip install`` from an sdist needs CMake and a C++20 compiler (GCC 10+,
Clang 10+, AppleClang 12+). A prebuilt
wheel needs neither.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

from setuptools import Distribution, setup
from setuptools.command.build_py import build_py

HERE = Path(__file__).resolve().parent
#: bindings/python -> bindings -> common/geometry
CORE_ROOT = HERE.parent.parent
PACKAGE = "cyberwave_geometry"


def _library_names() -> tuple[str, ...]:
    if sys.platform == "darwin":
        return ("libcyberwave_geometry_c.dylib",)
    if sys.platform == "win32":
        return ("cyberwave_geometry_c.dll",)
    # Ship the SONAME'd files too: the loader follows the symlink chain, and a
    # wheel cannot carry symlinks reliably, so all of them are real files.
    return (
        "libcyberwave_geometry_c.so",
        "libcyberwave_geometry_c.so.0",
    )


class BuildWithCore(build_py):
    """Run CMake, then copy the built library into the package directory."""

    def run(self) -> None:
        prebuilt = os.environ.get("CYBERWAVE_GEOMETRY_PREBUILT_LIBRARY")
        target_dir = Path(self.build_lib) / PACKAGE
        target_dir.mkdir(parents=True, exist_ok=True)

        if prebuilt:
            # cibuildwheel and the backend image build the core once and hand
            # the path in, rather than compiling it per Python version.
            source = Path(prebuilt)
            if not source.exists():
                raise SystemExit(
                    f"CYBERWAVE_GEOMETRY_PREBUILT_LIBRARY={prebuilt} does not exist"
                )
            shutil.copy2(source, target_dir / source.name)
        elif not (CORE_ROOT / "CMakeLists.txt").exists():
            raise SystemExit(
                "Cannot find the geometry core sources.\n"
                f"Expected {CORE_ROOT / 'CMakeLists.txt'}.\n"
                "Building this package from a source tree requires the whole of\n"
                "common/geometry, not just bindings/python. Install a prebuilt\n"
                "wheel, or set CYBERWAVE_GEOMETRY_PREBUILT_LIBRARY to a built\n"
                "libcyberwave_geometry_c."
            )
        else:
            self._cmake_build(target_dir)

        super().run()

    def _cmake_build(self, target_dir: Path) -> None:
        if shutil.which("cmake") is None:
            raise SystemExit(
                "cmake is required to build cyberwave-geometry from source.\n"
                "Install a prebuilt wheel instead, or set "
                "CYBERWAVE_GEOMETRY_PREBUILT_LIBRARY."
            )
        build_dir = (
            Path(self.build_temp if hasattr(self, "build_temp") else "build")
            / "geometry"
        )
        build_dir.mkdir(parents=True, exist_ok=True)

        # A CMakeCache.txt records the absolute source directory it was
        # generated for, and CMake hard-errors rather than reconfiguring when
        # that no longer matches -- which happens whenever a build tree is
        # copied somewhere else, most easily by a Docker COPY picking up a
        # developer's build/ directory. Discard a mismatched cache instead of
        # failing with a message about paths that mean nothing in the new
        # context. (`common/geometry/.dockerignore` stops it travelling in the
        # first place; this is the backstop.)
        cache = build_dir / "CMakeCache.txt"
        if cache.exists():
            recorded = ""
            for line in cache.read_text(errors="replace").splitlines():
                if line.startswith("CMAKE_HOME_DIRECTORY:"):
                    recorded = line.split("=", 1)[-1].strip()
                    break
            if recorded and Path(recorded).resolve() != CORE_ROOT.resolve():
                self.announce(
                    f"discarding a CMake cache generated for {recorded}", level=2
                )
                shutil.rmtree(build_dir)
                build_dir.mkdir(parents=True, exist_ok=True)
        subprocess.check_call(
            [
                "cmake",
                "-S",
                str(CORE_ROOT),
                "-B",
                str(build_dir),
                "-DCMAKE_BUILD_TYPE=Release",
                "-DCYBERWAVE_GEOMETRY_BUILD_TESTS=OFF",
                "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
            ]
        )
        subprocess.check_call(
            ["cmake", "--build", str(build_dir), "--config", "Release", "-j"]
        )

        copied = []
        for name in _library_names():
            for candidate in (build_dir / name, build_dir / "Release" / name):
                if candidate.exists():
                    shutil.copy2(candidate, target_dir / name)
                    copied.append(name)
                    break
        if not copied:
            raise SystemExit(
                f"CMake reported success but produced none of {_library_names()} "
                f"under {build_dir}"
            )
        self.announce(f"bundled {', '.join(copied)} into {target_dir}", level=2)


class BinaryDistribution(Distribution):
    """Force a platform tag.

    setuptools decides ``py3-none-any`` from the absence of ext_modules, and
    this package has none -- the native code arrives via CMake above. Without
    this the wheel would claim to be portable and then fail on every machine
    whose libc differs from the builder's.
    """

    def has_ext_modules(self) -> bool:
        return True

    def get_tag(self):  # pragma: no cover - setuptools internals
        return (sysconfig.get_python_version(), "none", None)


setup(
    cmdclass={"build_py": BuildWithCore},
    distclass=BinaryDistribution,
)
