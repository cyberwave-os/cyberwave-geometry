# Cyberwave shared geometry core.
#
# Dependency-free C++20: no Eigen, no fmt, no gtest. Everything that consumes
# this -- the Python extension, the WASM bundle, the C++ SDK, an Android native
# build -- must be able to compile it with nothing but a standard library, so
# adding a dependency here is a cross-cutting decision, not a local one.
#
# C++20 is the repo's newer C++ baseline: the edge-runtime driver common lib,
# the ROS 2 C++ driver template and the native camera drivers all build at 20.
# The core uses no C++20 feature -- it is written to the same subset it always
# was -- so the standard here is about not being the one target that holds a
# consumer back, not about what the sources need.
cmake_minimum_required(VERSION 3.14)

# 0.2.0: renamed the three accessors that did not read as conversions --
#        quat::yaw -> to_yaw, geo::compass_heading_deg -> to_compass_heading_deg,
#        geo::ned_rpy -> to_ned_rpy, and the matching cw_geom_* entry points.
#        Fixed quat::to_yaw at gimbal lock, where it disagreed with to_rpy.
#        Added the Pose conversions and the missing WASM exports.
# 0.1.2: added the geodetic module (geo::, cw_geom_geo_*), which grows the
#        surface without changing any existing numeric behaviour.
# 0.1.1: added quat::slerp / quat::nlerp.
# 0.1.0: first release.
#
# The version is what a conformance-tested binding pins against. Bump the minor
# whenever numeric behaviour changes and the patch when the surface grows, so a
# stale artifact -- a wheel or a Docker layer built before the change -- fails
# on its version pin rather than at the call site with an AttributeError.
project(CyberwaveGeometry VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(CYBERWAVE_GEOMETRY_BUILD_TESTS "Build cyberwave geometry tests" ON)
option(CYBERWAVE_GEOMETRY_BUILD_C_API "Build the stable C ABI shared library" ON)

set(CYBERWAVE_GEOMETRY_SOURCES
    src/errors.cpp
    src/geodetic.cpp
    src/quaternion.cpp
    src/transform.cpp
    src/fk.cpp
    src/version.cpp
)

add_library(cyberwave_geometry ${CYBERWAVE_GEOMETRY_SOURCES})
add_library(Cyberwave::Geometry ALIAS cyberwave_geometry)

target_include_directories(cyberwave_geometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
# PUBLIC is a floor, not a cap: a consumer may build at a later standard, but
# not at an earlier one. Anything that links Cyberwave::Geometry therefore has
# to be at C++20 too -- which today is nothing in-tree (the core is reached
# through the C ABI and the WASM bundle), and is why this can move now rather
# than after a migration. cyberwave-sdks/cyberwave-cpp is still at 17 and has to
# move before it can link this target.
target_compile_features(cyberwave_geometry PUBLIC cxx_std_20)
set_target_properties(cyberwave_geometry PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    # An ALIAS is build-tree only; EXPORT_NAME is what makes an installed
    # consumer's find_package see Cyberwave::Geometry rather than
    # Cyberwave::cyberwave_geometry. Without it the two spellings diverge and
    # only the in-tree build works.
    EXPORT_NAME Geometry
)

if(MSVC)
    target_compile_options(cyberwave_geometry PRIVATE /W4 /WX)
else()
    target_compile_options(cyberwave_geometry PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(CYBERWAVE_GEOMETRY_BUILD_C_API)
    add_library(cyberwave_geometry_c SHARED src/c_api.cpp)
    add_library(Cyberwave::GeometryC ALIAS cyberwave_geometry_c)
    target_link_libraries(cyberwave_geometry_c PRIVATE cyberwave_geometry)
    target_include_directories(cyberwave_geometry_c
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    set_target_properties(cyberwave_geometry_c PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
        EXPORT_NAME GeometryC
        # Only the cw_geom_* entry points are exported; everything else stays
        # internal so the ABI surface is exactly what c_api.h declares.
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )
    if(NOT MSVC)
        target_compile_options(cyberwave_geometry_c PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    # CXX_VISIBILITY_PRESET only governs this target's own objects. The core is
    # linked in as a static archive, and an archive's symbols keep the
    # visibility they were compiled with, so without this the whole C++ API
    # leaks out of a library whose entire point is that its surface is exactly
    # what c_api.h declares. --exclude-libs is GNU ld / lld only; on other
    # linkers the extra symbols are cosmetic rather than harmful.
    if(UNIX AND NOT APPLE)
        target_link_options(cyberwave_geometry_c PRIVATE "LINKER:--exclude-libs,ALL")
    endif()
endif()

if(CYBERWAVE_GEOMETRY_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

install(TARGETS cyberwave_geometry
    EXPORT CyberwaveGeometryTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
if(CYBERWAVE_GEOMETRY_BUILD_C_API)
    install(TARGETS cyberwave_geometry_c
        EXPORT CyberwaveGeometryTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/CyberwaveGeometryConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    # SameMajorVersion: while the core is 0.x a minor bump signals a numeric
    # behaviour change, which a conformance-tested binding wants to notice, but
    # not one that should lock a 0.x consumer out at configure time.
    COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CyberwaveGeometryConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/CyberwaveGeometryConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CyberwaveGeometry
)
install(EXPORT CyberwaveGeometryTargets
    FILE CyberwaveGeometryTargets.cmake
    NAMESPACE Cyberwave::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CyberwaveGeometry
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/CyberwaveGeometryConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/CyberwaveGeometryConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CyberwaveGeometry
)
