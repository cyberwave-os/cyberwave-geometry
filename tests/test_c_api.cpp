// The C ABI is what Python, WASM and JNI actually call, so it gets its own
// coverage: handle lifetimes, null tolerance, and agreement with the C++ core.
#include "cyberwave/geometry/c_api.h"

#include "test_support.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

cw_geom_vec3 v3(double x, double y, double z) { return cw_geom_vec3{x, y, z}; }

cyberwave::geometry::Vector3 to_cpp(cw_geom_vec3 v) { return cyberwave::geometry::Vector3{v.x, v.y, v.z}; }

cyberwave::geometry::Quaternion to_cpp(cw_geom_quat q)
{
    return cyberwave::geometry::Quaternion{.x = q.x, .y = q.y, .z = q.z, .w = q.w};
}

/// The two-link arm the C++ FK tests use, built through the C builder.
cw_geom_tree* build_arm()
{
    cw_geom_builder* builder = cw_geom_builder_create();
    cw_geom_builder_add_link(builder, "base_link");
    cw_geom_builder_add_link(builder, "upper_arm");
    cw_geom_builder_add_link(builder, "forearm");

    cw_geom_transform shoulder_origin = cw_geom_transform_identity();
    shoulder_origin.translation = v3(0, 0, 1);
    cw_geom_builder_add_joint(builder, "shoulder", "base_link", "upper_arm", CW_GEOM_JOINT_REVOLUTE, shoulder_origin,
                              v3(0, 0, 1), nullptr, 1.0, 0.0);

    cw_geom_transform elbow_origin = cw_geom_transform_identity();
    elbow_origin.translation = v3(1, 0, 0);
    cw_geom_builder_add_joint(builder, "elbow", "upper_arm", "forearm", CW_GEOM_JOINT_REVOLUTE, elbow_origin,
                              v3(0, 1, 0), nullptr, 1.0, 0.0);

    cw_geom_transform camera = cw_geom_transform_identity();
    camera.translation = v3(0.1, 0, 0);
    cw_geom_builder_add_sensor(builder, "wrist_camera", "forearm", camera);

    cw_geom_tree* tree = cw_geom_tree_build(builder);
    // The tree copies what it needs; the builder is free to go immediately.
    cw_geom_builder_destroy(builder);
    return tree;
}

} // namespace

TEST(version_is_reported_consistently)
{
    int32_t major = -1;
    int32_t minor = -1;
    int32_t patch = -1;
    cw_geom_version_parts(&major, &minor, &patch);
    const std::string expected = std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    CHECK(expected == cw_geom_version());
}

TEST(version_parts_tolerates_null_outputs)
{
    cw_geom_version_parts(nullptr, nullptr, nullptr);
    CHECK(true); // reaching here without a crash is the assertion
}

TEST(error_names_are_stable_strings)
{
    CHECK(std::strcmp(cw_geom_error_name(CW_GEOM_OK), "ok") == 0);
    CHECK(std::strcmp(cw_geom_error_name(CW_GEOM_ERR_INVALID_QUATERNION), "invalid_quaternion") == 0);
    CHECK(std::strcmp(cw_geom_error_name(CW_GEOM_ERR_CYCLE), "cycle") == 0);
}

TEST(component_order_helpers_disagree_as_expected)
{
    const cw_geom_quat wxyz = cw_geom_quat_from_wxyz(0.1, 0.2, 0.3, 0.4);
    const cw_geom_quat xyzw = cw_geom_quat_from_xyzw(0.1, 0.2, 0.3, 0.4);
    CHECK(wxyz.w == 0.1);
    CHECK(xyzw.w == 0.4);

    double out[4] = {0, 0, 0, 0};
    cw_geom_quat_to_xyzw(wxyz, out);
    CHECK(out[0] == 0.2);
    CHECK(out[3] == 0.1);
}

TEST(quaternion_entry_points_agree_with_the_core)
{
    const cw_geom_quat a = cw_geom_quat_from_rpy(0.4, -0.5, 0.6);
    const cw_geom_quat b = cw_geom_quat_from_yaw(0.9);
    CHECK_NEAR(to_cpp(cw_geom_quat_multiply(a, b)), cyberwave::geometry::quat::multiply(to_cpp(a), to_cpp(b)),
               cwtest::kEps);

    cw_geom_quat normalized;
    CHECK(cw_geom_quat_normalize(cw_geom_quat_from_wxyz(2, 4, 6, 8), &normalized) == CW_GEOM_OK);
    CHECK_NEAR(cw_geom_quat_norm(normalized), 1.0, cwtest::kEps);
}

TEST(a_failing_call_returns_a_status_and_leaves_the_output_alone)
{
    cw_geom_quat out = cw_geom_quat_from_yaw(1.234);
    const cw_geom_quat before = out;
    const cw_geom_status status = cw_geom_quat_normalize(cw_geom_quat_from_wxyz(0, 0, 0, 0), &out);
    CHECK(status == CW_GEOM_ERR_INVALID_QUATERNION);
    CHECK(out.w == before.w && out.z == before.z);
}

TEST(out_parameters_may_be_null)
{
    CHECK(cw_geom_quat_normalize(cw_geom_quat_from_yaw(0.5), nullptr) == CW_GEOM_OK);
    CHECK(cw_geom_quat_to_rpy(cw_geom_quat_from_yaw(0.5), nullptr, nullptr, nullptr) == CW_GEOM_OK);
    cw_geom_quat_to_wxyz(cw_geom_quat_identity(), nullptr);
    CHECK(true);
}

TEST(matrix_round_trips_through_the_abi)
{
    const cw_geom_quat original = cw_geom_quat_from_rpy(0.1, 0.2, 0.3);
    double matrix[9];
    CHECK(cw_geom_quat_to_matrix(original, matrix) == CW_GEOM_OK);
    cw_geom_quat recovered;
    CHECK(cw_geom_quat_from_matrix(matrix, 1e-6, &recovered) == CW_GEOM_OK);
    CHECK(cwtest::same_rotation(to_cpp(recovered), to_cpp(original), 1e-12));
}

TEST(from_matrix_rejects_a_null_pointer_rather_than_dereferencing_it)
{
    cw_geom_quat out;
    CHECK(cw_geom_quat_from_matrix(nullptr, 1e-6, &out) == CW_GEOM_ERR_INVALID_ROTATION_MATRIX);
}

TEST(transform_entry_points_agree_with_the_core)
{
    cw_geom_transform parent = cw_geom_transform_identity();
    parent.translation = v3(1, 2, 3);
    parent.rotation = cw_geom_quat_from_rpy(0.2, -0.3, 0.4);
    cw_geom_transform child = cw_geom_transform_identity();
    child.translation = v3(-0.5, 0.25, 2.0);
    child.rotation = cw_geom_quat_from_yaw(0.7);

    const cw_geom_transform composed = cw_geom_transform_compose(parent, child);
    const cw_geom_transform recovered = cw_geom_transform_relative(parent, composed);
    CHECK_NEAR(to_cpp(recovered.translation), to_cpp(child.translation), 1e-12);
    CHECK(cwtest::same_rotation(to_cpp(recovered.rotation), to_cpp(child.rotation), 1e-12));
}

TEST(joint_type_parsing_crosses_the_abi)
{
    CHECK(cw_geom_joint_type_from_string(" Revolute ") == CW_GEOM_JOINT_REVOLUTE);
    CHECK(cw_geom_joint_type_from_string("floating") == CW_GEOM_JOINT_UNSUPPORTED);
    CHECK(cw_geom_joint_type_from_string(nullptr) == CW_GEOM_JOINT_UNSUPPORTED);
    CHECK(std::strcmp(cw_geom_joint_type_name(CW_GEOM_JOINT_PRISMATIC), "prismatic") == 0);
}

TEST(a_tree_built_through_the_abi_resolves_frames)
{
    cw_geom_tree* tree = build_arm();
    CHECK(tree != nullptr);
    CHECK(cw_geom_tree_is_usable(tree) == 1);
    CHECK(cw_geom_tree_is_articulated(tree) == 1);
    CHECK(std::strcmp(cw_geom_tree_base_frame(tree), "base_link") == 0);
    CHECK(cw_geom_tree_error_count(tree) == 0);

    CHECK(cw_geom_tree_frame_count(tree) == 4);
    std::vector<std::string> frames;
    for (size_t index = 0; index < cw_geom_tree_frame_count(tree); ++index)
    {
        frames.emplace_back(cw_geom_tree_frame_name(tree, index));
    }
    const std::vector<std::string> expected{"base_link", "forearm", "upper_arm", "wrist_camera"};
    CHECK(frames == expected);

    // Out of range yields an empty string rather than reading past the end.
    CHECK(std::strcmp(cw_geom_tree_frame_name(tree, 99), "") == 0);
    cw_geom_tree_destroy(tree);
}

TEST(a_frame_pose_crosses_the_abi_with_joint_state)
{
    cw_geom_tree* tree = build_arm();
    const char* names[] = {"shoulder", "elbow"};
    const double values[] = {kPi / 2, 0.0};

    cw_geom_pose* pose = cw_geom_compute_frame_pose(tree, "forearm", cw_geom_transform_identity(), names, values, 2);
    CHECK(cw_geom_pose_status(pose) == CW_GEOM_OK);
    CHECK(cw_geom_pose_available(pose) == 1);
    CHECK_NEAR(to_cpp(cw_geom_pose_transform(pose).translation), (cyberwave::geometry::Vector3{0, 1, 1}), 1e-12);
    cw_geom_pose_destroy(pose);
    cw_geom_tree_destroy(tree);
}

TEST(missing_joint_state_is_reported_across_the_abi)
{
    cw_geom_tree* tree = build_arm();
    cw_geom_pose* pose = cw_geom_compute_frame_pose(tree, "forearm", cw_geom_transform_identity(), nullptr, nullptr, 0);
    CHECK(cw_geom_pose_status(pose) == CW_GEOM_OK);
    CHECK(cw_geom_pose_available(pose) == 0);
    CHECK(cw_geom_pose_missing_count(pose) == 2);
    CHECK(std::strcmp(cw_geom_pose_missing_joint(pose, 0), "elbow") == 0);
    CHECK(std::strcmp(cw_geom_pose_missing_joint(pose, 1), "shoulder") == 0);
    cw_geom_pose_destroy(pose);
    cw_geom_tree_destroy(tree);
}

TEST(a_structural_failure_carries_its_subject_across_the_abi)
{
    cw_geom_tree* tree = build_arm();
    cw_geom_pose* pose = cw_geom_compute_frame_pose(tree, "nope", cw_geom_transform_identity(), nullptr, nullptr, 0);
    CHECK(cw_geom_pose_status(pose) == CW_GEOM_ERR_UNKNOWN_FRAME);
    CHECK(std::strcmp(cw_geom_pose_error_subject(pose), "nope") == 0);
    CHECK(std::strlen(cw_geom_pose_error_detail(pose)) > 0);
    cw_geom_pose_destroy(pose);
    cw_geom_tree_destroy(tree);
}

TEST(a_null_tree_yields_a_failed_pose_rather_than_a_crash)
{
    cw_geom_pose* pose =
        cw_geom_compute_frame_pose(nullptr, "forearm", cw_geom_transform_identity(), nullptr, nullptr, 0);
    CHECK(pose != nullptr);
    CHECK(cw_geom_pose_status(pose) != CW_GEOM_OK);
    cw_geom_pose_destroy(pose);
}

TEST(destroying_a_null_handle_is_a_no_op)
{
    cw_geom_builder_destroy(nullptr);
    cw_geom_tree_destroy(nullptr);
    cw_geom_pose_destroy(nullptr);
    CHECK(true);
}

TEST(accessors_tolerate_a_null_handle)
{
    CHECK(std::strcmp(cw_geom_tree_base_frame(nullptr), "") == 0);
    CHECK(cw_geom_tree_frame_count(nullptr) == 0);
    CHECK(cw_geom_tree_is_usable(nullptr) == 0);
    CHECK(cw_geom_pose_missing_count(nullptr) == 0);
}

TEST(a_repeated_joint_name_keeps_its_last_value)
{
    cw_geom_tree* tree = build_arm();
    const char* names[] = {"shoulder", "elbow", "shoulder"};
    const double values[] = {0.0, 0.0, kPi / 2};
    cw_geom_pose* pose = cw_geom_compute_frame_pose(tree, "forearm", cw_geom_transform_identity(), names, values, 3);
    CHECK(cw_geom_pose_available(pose) == 1);
    CHECK_NEAR(to_cpp(cw_geom_pose_transform(pose).translation), (cyberwave::geometry::Vector3{0, 1, 1}), 1e-12);
    cw_geom_pose_destroy(pose);
    cw_geom_tree_destroy(tree);
}

TEST(the_geodetic_abi_agrees_with_the_core)
{
    const cw_geom_geo_anchor anchor{cw_geom_geodetic{47.3769, 8.5417, 408.0}, 30.0};
    const cw_geom_geo_pose pose{cw_geom_geodetic{47.3801, 8.5500, 421.0},
                                cw_geom_quat{.x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0}};

    cw_geom_transform local{};
    CHECK(cw_geom_geo_to_local_pose(anchor, pose, &local) == CW_GEOM_OK);

    const auto expected = cyberwave::geometry::geo::to_local_pose(
        cyberwave::geometry::GeoAnchor{cyberwave::geometry::Geodetic{47.3769, 8.5417, 408.0}, 30.0},
        cyberwave::geometry::GeoPose{cyberwave::geometry::Geodetic{47.3801, 8.5500, 421.0},
                                     cyberwave::geometry::Quaternion::identity()});
    CHECK(expected.ok());
    CHECK_NEAR(to_cpp(local.translation), expected.value().translation, 1e-12);
    CHECK_NEAR(to_cpp(local.rotation), expected.value().rotation, 1e-12);

    cw_geom_geo_pose recovered{};
    CHECK(cw_geom_geo_from_local_pose(anchor, local, &recovered) == CW_GEOM_OK);
    CHECK(cwtest::near(recovered.position.latitude_deg, pose.position.latitude_deg, 1e-11));
    CHECK(cwtest::near(recovered.position.longitude_deg, pose.position.longitude_deg, 1e-11));
    CHECK(cwtest::near(recovered.position.altitude_m, pose.position.altitude_m, 1e-9));
}

TEST(the_geodetic_abi_reports_failures_as_a_status_and_tolerates_null_outs)
{
    const cw_geom_geodetic off_ellipsoid{91.0, 0.0, 0.0};
    CHECK(cw_geom_geo_validate(off_ellipsoid) == CW_GEOM_ERR_INVALID_GEODETIC);
    CHECK(cw_geom_geo_validate(cw_geom_geodetic{47.0, 8.0, 0.0}) == CW_GEOM_OK);

    const cw_geom_geo_anchor polar{cw_geom_geodetic{89.95, 0.0, 0.0}, 0.0};
    CHECK(cw_geom_geo_validate_anchor(polar) == CW_GEOM_ERR_INVALID_GEO_ANCHOR);
    CHECK(cw_geom_geo_to_local(polar, polar.origin, nullptr) == CW_GEOM_ERR_INVALID_GEO_ANCHOR);

    // Every out pointer is optional, like the rest of the ABI.
    CHECK(cw_geom_geo_metres_per_degree(0.0, nullptr, nullptr) == CW_GEOM_OK);
    CHECK(cw_geom_geo_quat_from_compass_heading(90.0, nullptr) == CW_GEOM_OK);
    CHECK(cw_geom_geo_to_ned_rpy(cw_geom_quat{.x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0}, nullptr, nullptr, nullptr) ==
          CW_GEOM_OK);

    // A bearing round trip through the ABI, so a marshalling slip in the degree
    // conversion cannot hide behind the C++ tests.
    cw_geom_quat north{};
    CHECK(cw_geom_geo_quat_from_compass_heading(0.0, &north) == CW_GEOM_OK);
    double bearing = -1.0;
    CHECK(cw_geom_geo_to_compass_heading_deg(north, &bearing) == CW_GEOM_OK);
    CHECK(cwtest::near(bearing, 0.0, 1e-9));
}

int main() { return cwtest::run_all("c_api"); }
