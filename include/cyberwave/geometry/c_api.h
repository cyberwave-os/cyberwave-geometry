/*
 * Stable C ABI for the Cyberwave geometry core.
 *
 * This is the seam every non-C++ consumer sits on: the Python binding, the
 * Emscripten/WASM bundle and an Android JNI shim all call these entry points.
 * C++ consumers should include geometry.hpp instead and skip the marshalling.
 *
 * ABI rules, which is the whole point of this file existing:
 *
 *  - Structs here are plain C aggregates of doubles. Their layout is frozen.
 *  - Enumerated values are int32_t and append-only; a value is never reused.
 *  - Functions are never removed or given different parameters. A changed
 *    signature gets a new name.
 *  - Every `const char*` returned points into memory owned by the handle it
 *    came from, and is valid until that handle is destroyed. Callers copy.
 *  - Every function returning a `cw_geom_*` pointer allocates; the matching
 *    `_destroy` frees it. Passing NULL to a `_destroy` is a no-op.
 *  - No function throws, longjmps or aborts on bad input. Failure is a status.
 */
#ifndef CYBERWAVE_GEOMETRY_C_API_H
#define CYBERWAVE_GEOMETRY_C_API_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#define CW_GEOM_API __declspec(dllexport)
#else
#define CW_GEOM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /* --- value types --------------------------------------------------------- */

    typedef struct
    {
        double x;
        double y;
        double z;
    } cw_geom_vec3;

    /* Component order in memory is x, y, z, w -- the Cyberwave convention, shared
     * with protobuf, ROS and Three.js. Use the _from_/_to_ helpers rather than
     * depending on it; wxyz callers have cw_geom_quat_from_wxyz/_to_wxyz. */
    typedef struct
    {
        double x;
        double y;
        double z;
        double w;
    } cw_geom_quat;

    typedef struct
    {
        cw_geom_vec3 translation;
        cw_geom_quat rotation;
    } cw_geom_transform;

    /* WGS-84 geodetic position. Degrees and metres -- the deliberate exception to
     * "angles are radians", because every producer of one speaks degrees. The
     * altitude carries no datum of its own; see CONVENTIONS.md section 9. */
    typedef struct
    {
        double latitude_deg;
        double longitude_deg;
        double altitude_m;
    } cw_geom_geodetic;

    /* A 6-DOF pose on Earth. `orientation` maps the body frame (forward-left-up)
     * into the local ENU frame at `position`, and is only interpretable together
     * with it -- never assemble the two from separate messages. */
    typedef struct
    {
        cw_geom_geodetic position;
        cw_geom_quat orientation;
    } cw_geom_geo_pose;

    /* A site's georeference. `heading_deg` is the true-north bearing of
     * environment +Y, degrees clockwise; 0 means the environment frame is ENU. */
    typedef struct
    {
        cw_geom_geodetic origin;
        double heading_deg;
    } cw_geom_geo_anchor;

    /* 0 on success, otherwise a cw_geom_error_* code. */
    typedef int32_t cw_geom_status;

    enum
    {
        CW_GEOM_OK = 0,
        CW_GEOM_ERR_INVALID_QUATERNION = 1,
        CW_GEOM_ERR_INVALID_AXIS = 2,
        CW_GEOM_ERR_NON_FINITE_VALUE = 3,
        CW_GEOM_ERR_INVALID_ROTATION_MATRIX = 4,
        CW_GEOM_ERR_INVALID_GEODETIC = 5,
        CW_GEOM_ERR_INVALID_GEO_ANCHOR = 6,
        CW_GEOM_ERR_NO_LINKS = 10,
        CW_GEOM_ERR_NO_ROOT = 11,
        CW_GEOM_ERR_MULTIPLE_ROOTS = 12,
        CW_GEOM_ERR_AMBIGUOUS_PARENT = 13,
        CW_GEOM_ERR_CYCLE = 14,
        CW_GEOM_ERR_INVALID_JOINT_POSE = 15,
        CW_GEOM_ERR_INVALID_SENSOR_POSE = 16,
        CW_GEOM_ERR_UNSUPPORTED_JOINT_TYPE = 17,
        CW_GEOM_ERR_MISSING_JOINT = 18,
        CW_GEOM_ERR_UNKNOWN_FRAME = 19
    };

    enum
    {
        CW_GEOM_JOINT_FIXED = 0,
        CW_GEOM_JOINT_REVOLUTE = 1,
        CW_GEOM_JOINT_CONTINUOUS = 2,
        CW_GEOM_JOINT_PRISMATIC = 3,
        CW_GEOM_JOINT_UNSUPPORTED = 4
    };

    /* --- version ------------------------------------------------------------- */

    CW_GEOM_API const char* cw_geom_version(void);
    CW_GEOM_API void cw_geom_version_parts(int32_t* major, int32_t* minor, int32_t* patch);
    CW_GEOM_API const char* cw_geom_error_name(cw_geom_status status);

    /* --- quaternion ---------------------------------------------------------- */

    CW_GEOM_API cw_geom_quat cw_geom_quat_identity(void);
    CW_GEOM_API cw_geom_quat cw_geom_quat_from_wxyz(double w, double x, double y, double z);
    CW_GEOM_API cw_geom_quat cw_geom_quat_from_xyzw(double x, double y, double z, double w);
    /* Writes 4 doubles. */
    CW_GEOM_API void cw_geom_quat_to_wxyz(cw_geom_quat q, double* out4);
    CW_GEOM_API void cw_geom_quat_to_xyzw(cw_geom_quat q, double* out4);

    CW_GEOM_API double cw_geom_quat_norm(cw_geom_quat q);
    CW_GEOM_API double cw_geom_quat_dot(cw_geom_quat a, cw_geom_quat b);
    CW_GEOM_API cw_geom_status cw_geom_quat_normalize(cw_geom_quat q, cw_geom_quat* out);
    CW_GEOM_API cw_geom_quat cw_geom_quat_multiply(cw_geom_quat a, cw_geom_quat b);
    CW_GEOM_API cw_geom_quat cw_geom_quat_conjugate(cw_geom_quat q);
    CW_GEOM_API cw_geom_status cw_geom_quat_inverse(cw_geom_quat q, cw_geom_quat* out);

    /* Precondition: q is normalized. Unchecked -- this is the FK inner loop. */
    CW_GEOM_API cw_geom_vec3 cw_geom_quat_rotate_unit(cw_geom_quat q, cw_geom_vec3 v);
    CW_GEOM_API cw_geom_status cw_geom_quat_rotate(cw_geom_quat q, cw_geom_vec3 v, cw_geom_vec3* out);

    CW_GEOM_API cw_geom_status cw_geom_quat_from_axis_angle(cw_geom_vec3 axis, double angle, cw_geom_quat* out);
    CW_GEOM_API cw_geom_status cw_geom_quat_to_axis_angle(cw_geom_quat q, cw_geom_vec3* out_axis, double* out_angle);
    /* Interpolation, both along the shortest arc. `t` is not clamped. */
    CW_GEOM_API cw_geom_status cw_geom_quat_slerp(cw_geom_quat a, cw_geom_quat b, double t, cw_geom_quat* out);
    CW_GEOM_API cw_geom_status cw_geom_quat_nlerp(cw_geom_quat a, cw_geom_quat b, double t, cw_geom_quat* out);

    /* Fixed-axis XYZ (URDF/ROS). See CONVENTIONS.md. */
    CW_GEOM_API cw_geom_quat cw_geom_quat_from_rpy(double roll, double pitch, double yaw);
    CW_GEOM_API cw_geom_status cw_geom_quat_to_rpy(cw_geom_quat q, double* out_roll, double* out_pitch,
                                                   double* out_yaw);
    CW_GEOM_API cw_geom_quat cw_geom_quat_from_yaw(double yaw);
    CW_GEOM_API cw_geom_status cw_geom_quat_to_yaw(cw_geom_quat q, double* out);
    /* Row-major, 9 doubles. */
    CW_GEOM_API cw_geom_status cw_geom_quat_to_matrix(cw_geom_quat q, double* out9);
    CW_GEOM_API cw_geom_status cw_geom_quat_from_matrix(const double* m9, double tolerance, cw_geom_quat* out);

    /* --- transform ----------------------------------------------------------- */

    CW_GEOM_API cw_geom_transform cw_geom_transform_identity(void);
    CW_GEOM_API cw_geom_status cw_geom_transform_normalize(cw_geom_transform t, cw_geom_transform* out);
    CW_GEOM_API cw_geom_transform cw_geom_transform_compose(cw_geom_transform parent, cw_geom_transform child);
    CW_GEOM_API cw_geom_transform cw_geom_transform_inverse_unit(cw_geom_transform t);
    CW_GEOM_API cw_geom_status cw_geom_transform_inverse(cw_geom_transform t, cw_geom_transform* out);
    CW_GEOM_API cw_geom_vec3 cw_geom_transform_apply(cw_geom_transform t, cw_geom_vec3 point);
    CW_GEOM_API cw_geom_transform cw_geom_transform_relative(cw_geom_transform parent, cw_geom_transform child);

    /* --- geodetic ------------------------------------------------------------ */
    /*
     * See CONVENTIONS.md section 9. ENU (+x east, +y north, +z up) is the frame
     * every geodetic orientation here is expressed in; NED and compass bearings
     * are reachable only through the named adapters at the end of this block.
     */

    CW_GEOM_API cw_geom_status cw_geom_geo_validate(cw_geom_geodetic position);
    CW_GEOM_API cw_geom_status cw_geom_geo_validate_anchor(cw_geom_geo_anchor anchor);
    /* Wraps to [-180, 180) / [0, 360). Non-finite input passes through unchanged. */
    CW_GEOM_API double cw_geom_geo_normalize_longitude_deg(double longitude_deg);
    CW_GEOM_API double cw_geom_geo_normalize_bearing_deg(double bearing_deg);
    /* WGS-84 radii of curvature, not a spherical constant. Either out may be NULL. */
    CW_GEOM_API cw_geom_status cw_geom_geo_metres_per_degree(double latitude_deg, double* out_latitude,
                                                             double* out_longitude);

    /* Local tangent plane about `origin`. */
    CW_GEOM_API cw_geom_status cw_geom_geo_enu_between(cw_geom_geodetic origin, cw_geom_geodetic target,
                                                       cw_geom_vec3* out);
    CW_GEOM_API cw_geom_status cw_geom_geo_offset(cw_geom_geodetic origin, cw_geom_vec3 enu, cw_geom_geodetic* out);
    CW_GEOM_API cw_geom_status cw_geom_geo_ground_distance(cw_geom_geodetic a, cw_geom_geodetic b, double* out);

    /* Against a site's anchor: geodetic <-> environment frame. */
    CW_GEOM_API cw_geom_status cw_geom_geo_enu_to_local_rotation(cw_geom_geo_anchor anchor, cw_geom_quat* out);
    CW_GEOM_API cw_geom_status cw_geom_geo_to_local(cw_geom_geo_anchor anchor, cw_geom_geodetic position,
                                                    cw_geom_vec3* out);
    CW_GEOM_API cw_geom_status cw_geom_geo_from_local(cw_geom_geo_anchor anchor, cw_geom_vec3 position,
                                                      cw_geom_geodetic* out);
    /* The 6-DOF pair. */
    CW_GEOM_API cw_geom_status cw_geom_geo_to_local_pose(cw_geom_geo_anchor anchor, cw_geom_geo_pose pose,
                                                         cw_geom_transform* out);
    CW_GEOM_API cw_geom_status cw_geom_geo_from_local_pose(cw_geom_geo_anchor anchor, cw_geom_transform pose,
                                                           cw_geom_geo_pose* out);

    /* Foreign conventions, quarantined behind names that say which one they are.
     * A bearing is degrees clockwise from TRUE north; a NED attitude is radians. */
    CW_GEOM_API cw_geom_status cw_geom_geo_quat_from_compass_heading(double heading_deg, cw_geom_quat* out);
    CW_GEOM_API cw_geom_status cw_geom_geo_to_compass_heading_deg(cw_geom_quat enu_orientation, double* out);
    CW_GEOM_API cw_geom_status cw_geom_geo_quat_from_ned_rpy(double roll, double pitch, double yaw, cw_geom_quat* out);
    CW_GEOM_API cw_geom_status cw_geom_geo_to_ned_rpy(cw_geom_quat enu_orientation, double* out_roll, double* out_pitch,
                                                      double* out_yaw);

    /* --- forward kinematics -------------------------------------------------- */

    CW_GEOM_API int32_t cw_geom_joint_type_from_string(const char* text);
    CW_GEOM_API const char* cw_geom_joint_type_name(int32_t type);

    /* Accumulates a RobotDescription. Free with cw_geom_builder_destroy once the
     * tree is built; the tree does not reference the builder. */
    typedef struct cw_geom_builder cw_geom_builder;
    typedef struct cw_geom_tree cw_geom_tree;
    typedef struct cw_geom_pose cw_geom_pose;

    CW_GEOM_API cw_geom_builder* cw_geom_builder_create(void);
    CW_GEOM_API void cw_geom_builder_destroy(cw_geom_builder* builder);
    CW_GEOM_API void cw_geom_builder_add_link(cw_geom_builder* builder, const char* name);
    /* `mimic_source` NULL or empty means the joint has no mimic relation. */
    CW_GEOM_API void cw_geom_builder_add_joint(cw_geom_builder* builder, const char* name, const char* parent_link,
                                               const char* child_link, int32_t type, cw_geom_transform origin,
                                               cw_geom_vec3 axis, const char* mimic_source, double mimic_multiplier,
                                               double mimic_offset);
    CW_GEOM_API void cw_geom_builder_add_sensor(cw_geom_builder* builder, const char* name, const char* parent_link,
                                                cw_geom_transform extrinsic);

    CW_GEOM_API cw_geom_tree* cw_geom_tree_build(const cw_geom_builder* builder);
    CW_GEOM_API void cw_geom_tree_destroy(cw_geom_tree* tree);

    CW_GEOM_API const char* cw_geom_tree_root_link(const cw_geom_tree* tree);
    CW_GEOM_API const char* cw_geom_tree_base_frame(const cw_geom_tree* tree);
    CW_GEOM_API int32_t cw_geom_tree_is_usable(const cw_geom_tree* tree);
    CW_GEOM_API int32_t cw_geom_tree_is_articulated(const cw_geom_tree* tree);
    CW_GEOM_API size_t cw_geom_tree_frame_count(const cw_geom_tree* tree);
    CW_GEOM_API const char* cw_geom_tree_frame_name(const cw_geom_tree* tree, size_t index);
    CW_GEOM_API size_t cw_geom_tree_error_count(const cw_geom_tree* tree);
    /* `out_subject` / `out_detail` may be NULL. Returns CW_GEOM_OK for an
     * out-of-range index with the strings left untouched. */
    CW_GEOM_API cw_geom_status cw_geom_tree_error(const cw_geom_tree* tree, size_t index, const char** out_subject,
                                                  const char** out_detail);

    /* Joint state is two parallel arrays of `joint_count` entries. A name repeated
     * in the array keeps its last value, matching a dict literal. */
    CW_GEOM_API cw_geom_pose* cw_geom_compute_frame_pose(const cw_geom_tree* tree, const char* frame_name,
                                                         cw_geom_transform base, const char* const* joint_names,
                                                         const double* joint_values, size_t joint_count);
    CW_GEOM_API void cw_geom_pose_destroy(cw_geom_pose* pose);
    /* CW_GEOM_OK means the chain resolved; check cw_geom_pose_available() next. */
    CW_GEOM_API cw_geom_status cw_geom_pose_status(const cw_geom_pose* pose);
    CW_GEOM_API const char* cw_geom_pose_error_subject(const cw_geom_pose* pose);
    CW_GEOM_API const char* cw_geom_pose_error_detail(const cw_geom_pose* pose);
    /* 1 when the transform is meaningful; 0 when joint state was missing. */
    CW_GEOM_API int32_t cw_geom_pose_available(const cw_geom_pose* pose);
    CW_GEOM_API cw_geom_transform cw_geom_pose_transform(const cw_geom_pose* pose);
    CW_GEOM_API size_t cw_geom_pose_missing_count(const cw_geom_pose* pose);
    CW_GEOM_API const char* cw_geom_pose_missing_joint(const cw_geom_pose* pose, size_t index);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CYBERWAVE_GEOMETRY_C_API_H */
