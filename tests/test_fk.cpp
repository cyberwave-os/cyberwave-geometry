// Forward kinematics: tree construction, chain resolution, joint semantics and
// every failure mode the strict core is expected to name rather than guess at.
#include "test_support.hpp"

#include <cmath>
#include <limits>

using namespace cyberwave::geometry;
using fk::JointDescription;
using fk::JointType;
using fk::RobotDescription;
using fk::RobotTree;
using fk::SensorDescription;

namespace
{

constexpr double kPi = 3.14159265358979323846;

JointDescription make_joint(const std::string& name, const std::string& parent, const std::string& child,
                            JointType type, Transform origin = Transform::identity(), Vector3 axis = Vector3{0, 0, 1})
{
    JointDescription joint;
    joint.name = name;
    joint.parent_link = parent;
    joint.child_link = child;
    joint.type = type;
    joint.origin = origin;
    joint.axis = axis;
    return joint;
}

/// base_link -> shoulder (revolute about +Z, offset 1m up) -> elbow (revolute
/// about +Y, offset 1m along +X) -> a camera on the elbow.
RobotDescription two_link_arm()
{
    RobotDescription description;
    description.links = {"base_link", "upper_arm", "forearm"};
    description.joints = {
        make_joint("shoulder", "base_link", "upper_arm", JointType::kRevolute,
                   Transform{Vector3{0, 0, 1}, Quaternion::identity()}, Vector3{0, 0, 1}),
        make_joint("elbow", "upper_arm", "forearm", JointType::kRevolute,
                   Transform{Vector3{1, 0, 0}, Quaternion::identity()}, Vector3{0, 1, 0}),
    };
    SensorDescription camera;
    camera.name = "wrist_camera";
    camera.parent_link = "forearm";
    camera.extrinsic = Transform{Vector3{0.1, 0, 0}, Quaternion::identity()};
    description.sensors = {camera};
    return description;
}

} // namespace

// --- joint type parsing -----------------------------------------------------

TEST(joint_type_parsing_is_case_and_whitespace_insensitive)
{
    CHECK(fk::joint_type_from_string("  REVOLUTE ") == JointType::kRevolute);
    CHECK(fk::joint_type_from_string("Continuous") == JointType::kContinuous);
    CHECK(fk::joint_type_from_string("prismatic") == JointType::kPrismatic);
    CHECK(fk::joint_type_from_string("fixed") == JointType::kFixed);
}

TEST(an_unknown_joint_type_is_unsupported_not_fixed)
{
    // Defaulting an unrecognized type to `fixed` would silently freeze a
    // floating joint at its origin and report the pose as real.
    CHECK(fk::joint_type_from_string("floating") == JointType::kUnsupported);
    CHECK(fk::joint_type_from_string("") == JointType::kUnsupported);
    CHECK(!fk::joint_type_is_supported(JointType::kUnsupported));
}

TEST(only_moving_joints_are_actuated)
{
    CHECK(!fk::joint_type_is_actuated(JointType::kFixed));
    CHECK(fk::joint_type_is_actuated(JointType::kRevolute));
    CHECK(fk::joint_type_is_actuated(JointType::kContinuous));
    CHECK(fk::joint_type_is_actuated(JointType::kPrismatic));
}

// --- tree construction ------------------------------------------------------

TEST(build_finds_the_root_and_base_frame)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    CHECK(tree.errors().empty());
    CHECK(tree.root_link() == "base_link");
    CHECK(tree.base_frame() == "base_link");
    CHECK(tree.is_usable());
    CHECK(tree.is_articulated());
}

TEST(base_frame_descends_through_a_synthetic_world_anchor)
{
    // Real schemas anchor the robot to a `world` link that is not itself
    // declared. The robot's own pose topics describe base_link, not world.
    RobotDescription description = two_link_arm();
    description.joints.push_back(make_joint("base_joint", "world", "base_link", JointType::kFixed));
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.root_link() == "world");
    CHECK(tree.base_frame() == "base_link");
}

TEST(base_frame_stays_at_the_root_when_the_anchor_forks)
{
    // Two fixed children give no unambiguous body to call "the base".
    RobotDescription description = two_link_arm();
    description.links.push_back("other_base");
    description.joints.push_back(make_joint("base_joint", "world", "base_link", JointType::kFixed));
    description.joints.push_back(make_joint("other_joint", "world", "other_base", JointType::kFixed));
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.root_link() == "world");
    CHECK(tree.base_frame() == "world");
}

TEST(frame_names_lists_links_and_sensors_sorted)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const std::vector<std::string> expected{"base_link", "forearm", "upper_arm", "wrist_camera"};
    CHECK(tree.frame_names() == expected);
}

TEST(a_joint_with_a_degenerate_origin_is_reported_and_dropped)
{
    RobotDescription description = two_link_arm();
    description.joints[1].origin.rotation = Quaternion{.x = 0, .y = 0, .z = 0, .w = 0};
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.errors()[0].code == ErrorCode::kInvalidJointPose);
    CHECK(tree.errors()[0].subject == "elbow");
    CHECK(tree.joint_by_name("elbow") == nullptr);

    // Dropping the joint orphans everything below it, so the forearm becomes a
    // second root and that cascade is reported too. A caller asking for the
    // forearm needs to know it is unreachable, not only that some joint
    // elsewhere was malformed.
    CHECK(tree.errors().size() == 2);
    CHECK(tree.errors()[1].code == ErrorCode::kMultipleRoots);
    CHECK(!fk::resolve_chain(tree, "forearm").ok());

    // The rest of the robot still works.
    CHECK(tree.base_frame() == "base_link");
    CHECK(tree.joint_by_name("shoulder") != nullptr);
    CHECK(fk::resolve_chain(tree, "upper_arm").ok());
}

TEST(a_sensor_with_a_degenerate_extrinsic_is_reported_and_dropped)
{
    RobotDescription description = two_link_arm();
    description.sensors[0].extrinsic.rotation = Quaternion{.x = 0, .y = 0, .z = 0, .w = 0};
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.errors().size() == 1);
    CHECK(tree.errors()[0].code == ErrorCode::kInvalidSensorPose);
    CHECK(tree.sensor_by_name("wrist_camera") == nullptr);
}

TEST(build_normalizes_an_unnormalized_joint_origin)
{
    RobotDescription description = two_link_arm();
    const Quaternion unit = quat::from_yaw(0.5);
    description.joints[0].origin.rotation =
        Quaternion{.x = unit.x * 3, .y = unit.y * 3, .z = unit.z * 3, .w = unit.w * 3};
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.errors().empty());
    CHECK_NEAR(quat::norm(tree.joint_by_name("shoulder")->origin.rotation), 1.0, 1e-12);
}

TEST(two_joints_claiming_one_child_is_an_ambiguous_parent)
{
    RobotDescription description = two_link_arm();
    description.joints.push_back(make_joint("rogue", "base_link", "forearm", JointType::kFixed));
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.errors().size() == 1);
    CHECK(tree.errors()[0].code == ErrorCode::kAmbiguousParent);
    CHECK(tree.errors()[0].subject == "forearm");
    // The first joint wins, so the chain stays the one the schema declared.
    CHECK(tree.joint_by_child("forearm")->name == "elbow");
}

TEST(a_disconnected_description_reports_multiple_roots)
{
    RobotDescription description = two_link_arm();
    description.links.push_back("orphan_child");
    description.joints.push_back(make_joint("orphan_joint", "orphan_base", "orphan_child", JointType::kFixed));
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.errors().size() == 1);
    CHECK(tree.errors()[0].code == ErrorCode::kMultipleRoots);
    // The lowest-sorting root wins, deterministically across languages.
    CHECK(tree.root_link() == "base_link");
    CHECK(tree.is_usable());
}

TEST(an_all_cycle_description_has_no_root)
{
    RobotDescription description;
    description.links = {"a", "b"};
    description.joints = {
        make_joint("ab", "a", "b", JointType::kFixed),
        make_joint("ba", "b", "a", JointType::kFixed),
    };
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.errors().size() == 1);
    CHECK(tree.errors()[0].code == ErrorCode::kNoRoot);
    CHECK(!tree.is_usable());
}

TEST(an_empty_description_reports_no_links)
{
    const RobotTree tree = RobotTree::build(RobotDescription{});
    CHECK(tree.errors().size() == 1);
    CHECK(tree.errors()[0].code == ErrorCode::kNoLinks);
    CHECK(!tree.is_usable());
}

TEST(a_link_only_description_is_usable_but_not_articulated)
{
    RobotDescription description;
    description.links = {"base_link"};
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.errors().empty());
    CHECK(tree.base_frame() == "base_link");
    CHECK(!tree.is_articulated());
}

// --- chain resolution -------------------------------------------------------

TEST(resolve_chain_orders_joints_root_side_first)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const auto chain = fk::resolve_chain(tree, "forearm");
    CHECK(chain.ok());
    CHECK(chain.value().joint_indices.size() == 2);
    CHECK(tree.joints()[chain.value().joint_indices[0]].name == "shoulder");
    CHECK(tree.joints()[chain.value().joint_indices[1]].name == "elbow");
}

TEST(resolve_chain_reports_required_joints_sorted)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const auto chain = fk::resolve_chain(tree, "wrist_camera");
    CHECK(chain.ok());
    CHECK(chain.value().has_sensor);
    CHECK(chain.value().sensor_name == "wrist_camera");
    const std::vector<std::string> expected{"elbow", "shoulder"};
    CHECK(chain.value().required_joints == expected);
}

TEST(resolve_chain_rejects_an_unknown_frame)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const auto chain = fk::resolve_chain(tree, "nope");
    CHECK(!chain.ok());
    CHECK(chain.error().code == ErrorCode::kUnknownFrame);
}

TEST(resolve_chain_rejects_a_frame_behind_an_unsupported_joint)
{
    RobotDescription description = two_link_arm();
    description.joints[1].type = JointType::kUnsupported;
    const RobotTree tree = RobotTree::build(description);
    const auto chain = fk::resolve_chain(tree, "forearm");
    CHECK(!chain.ok());
    CHECK(chain.error().code == ErrorCode::kUnsupportedJointType);
    CHECK(chain.error().subject == "elbow");
    // The link above it is unaffected.
    CHECK(fk::resolve_chain(tree, "upper_arm").ok());
}

TEST(resolve_chain_reports_a_disconnected_frame)
{
    RobotDescription description = two_link_arm();
    description.links.push_back("orphan_child");
    description.joints.push_back(make_joint("orphan_joint", "orphan_base", "orphan_child", JointType::kFixed));
    const RobotTree tree = RobotTree::build(description);
    const auto chain = fk::resolve_chain(tree, "orphan_child");
    CHECK(!chain.ok());
    CHECK(chain.error().code == ErrorCode::kNoRoot);
}

TEST(resolve_chain_detects_a_cycle_that_does_not_reach_the_base)
{
    // b -> c -> b is a closed loop with `a` as the root, so the tree builds,
    // but walking up from c never arrives at the base frame.
    RobotDescription description;
    description.links = {"a", "b", "c"};
    description.joints = {
        make_joint("bc", "b", "c", JointType::kFixed),
        make_joint("cb", "c", "b", JointType::kFixed),
    };
    const RobotTree tree = RobotTree::build(description);
    CHECK(tree.is_usable());
    const auto chain = fk::resolve_chain(tree, "c");
    CHECK(!chain.ok());
    CHECK(chain.error().code == ErrorCode::kCycle);
}

// --- joint transforms -------------------------------------------------------

TEST(a_revolute_joint_applies_its_origin_before_its_motion)
{
    // Origin is a 90-degree yaw; motion is a turn about the joint's own +X.
    // Origin-first puts the motion about world +Y. The other order would put it
    // about world +X, which is the classic wrist-camera-in-the-wrong-place bug.
    fk::Joint joint;
    joint.name = "j";
    joint.type = JointType::kRevolute;
    joint.origin = Transform{Vector3::zero(), quat::from_yaw(kPi / 2)};
    joint.axis = Vector3{1, 0, 0};

    const auto step = fk::joint_transform(joint, kPi / 2);
    CHECK(step.ok());

    // Origin first: the origin has already swung the joint's own +X onto world
    // +Y, so the motion turns about world +Y and world +Z lands on world +X.
    const Vector3 moved = tf::apply(step.value(), Vector3{0, 0, 1});
    CHECK_NEAR(moved, (Vector3{1, 0, 0}), 1e-12);

    // Motion first would turn about world +X instead, landing on world -Y. The
    // two orders disagree by 90 degrees, which is why this is pinned by a test
    // rather than left to the reader of the composition order.
    const Transform motion{Vector3::zero(), quat::from_axis_angle(joint.axis, kPi / 2).value()};
    const Transform motion_first = tf::compose(motion, joint.origin);
    CHECK_NEAR((tf::apply(motion_first, Vector3{0, 0, 1})), (Vector3{0, -1, 0}), 1e-12);
}

TEST(a_prismatic_joint_translates_along_its_normalized_axis)
{
    fk::Joint joint;
    joint.name = "j";
    joint.type = JointType::kPrismatic;
    // Deliberately not unit: 0.5m of travel must be 0.5m, not 1.5m.
    joint.axis = Vector3{3, 0, 0};

    const auto step = fk::joint_transform(joint, 0.5);
    CHECK(step.ok());
    CHECK_NEAR(step.value().translation, (Vector3{0.5, 0, 0}), 1e-12);
    CHECK_NEAR(step.value().rotation, Quaternion::identity(), cwtest::kEps);
}

TEST(a_revolute_joint_normalizes_its_axis_too)
{
    fk::Joint scaled;
    scaled.type = JointType::kRevolute;
    scaled.axis = Vector3{0, 0, 7};
    fk::Joint unit;
    unit.type = JointType::kRevolute;
    unit.axis = Vector3{0, 0, 1};
    CHECK_NEAR(fk::joint_transform(scaled, 0.6).value().rotation, fk::joint_transform(unit, 0.6).value().rotation,
               1e-15);
}

TEST(a_fixed_joint_ignores_any_reported_position)
{
    fk::Joint joint;
    joint.type = JointType::kFixed;
    joint.origin = Transform{Vector3{1, 2, 3}, quat::from_yaw(0.3)};
    CHECK_NEAR(fk::joint_transform(joint, 99.0).value(), joint.origin, cwtest::kEps);
}

TEST(a_degenerate_axis_makes_the_joint_transform_fail)
{
    fk::Joint revolute;
    revolute.name = "j";
    revolute.type = JointType::kRevolute;
    revolute.axis = Vector3{0, 0, 0};
    CHECK(!fk::joint_transform(revolute, 0.5).ok());
    CHECK(fk::joint_transform(revolute, 0.5).error().code == ErrorCode::kInvalidAxis);

    fk::Joint prismatic = revolute;
    prismatic.type = JointType::kPrismatic;
    CHECK(!fk::joint_transform(prismatic, 0.5).ok());
}

TEST(a_non_finite_joint_position_is_rejected)
{
    fk::Joint joint;
    joint.name = "j";
    joint.type = JointType::kRevolute;
    joint.axis = Vector3{0, 0, 1};
    const auto result = fk::joint_transform(joint, std::nan(""));
    CHECK(!result.ok());
    CHECK(result.error().code == ErrorCode::kNonFiniteValue);
}

// --- frame poses ------------------------------------------------------------

TEST(the_base_frame_pose_is_the_base_transform_itself)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const Transform base{Vector3{5, 6, 7}, quat::from_yaw(0.4)};
    const auto pose = fk::compute_frame_pose(tree, "base_link", base, {});
    CHECK(pose.ok());
    CHECK(pose.value().available);
    CHECK_NEAR(pose.value().transform, base, cwtest::kEps);
}

TEST(a_frame_pose_composes_the_whole_chain)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    // Shoulder yaws 90 degrees; elbow stays put. The forearm sits 1m up and,
    // after the yaw, 1m along world +Y instead of +X.
    const fk::JointPositions positions{{"shoulder", kPi / 2}, {"elbow", 0.0}};
    const auto pose = fk::compute_frame_pose(tree, "forearm", Transform::identity(), positions);
    CHECK(pose.ok());
    CHECK(pose.value().available);
    CHECK_NEAR(pose.value().transform.translation, (Vector3{0, 1, 1}), 1e-12);
}

TEST(a_sensor_frame_applies_its_extrinsic_last)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const fk::JointPositions positions{{"shoulder", 0.0}, {"elbow", 0.0}};
    const auto forearm = fk::compute_frame_pose(tree, "forearm", Transform::identity(), positions);
    const auto camera = fk::compute_frame_pose(tree, "wrist_camera", Transform::identity(), positions);
    CHECK(forearm.ok() && camera.ok());
    CHECK_NEAR(camera.value().transform.translation,
               (vec::add(forearm.value().transform.translation, Vector3{0.1, 0, 0})), 1e-12);
}

TEST(the_base_transform_is_applied_to_every_frame)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const fk::JointPositions positions{{"shoulder", 0.3}, {"elbow", -0.2}};
    const Transform base{Vector3{10, 0, 0}, quat::from_yaw(kPi)};
    const auto at_origin = fk::compute_frame_pose(tree, "forearm", Transform::identity(), positions);
    const auto at_base = fk::compute_frame_pose(tree, "forearm", base, positions);
    CHECK(at_origin.ok() && at_base.ok());
    CHECK_NEAR(at_base.value().transform, tf::compose(base, at_origin.value().transform), 1e-12);
}

TEST(a_non_unit_base_rotation_is_normalized_rather_than_scaling_the_result)
{
    // quat::rotate_unit reduces to s^2 * R(v) + (1 - s^2) * v for q = s * q_hat,
    // a blend between the true rotation and none at all -- so a base that is
    // merely off-scale, not degenerate, silently moves the whole robot.
    const RobotTree tree = RobotTree::build(two_link_arm());
    const fk::JointPositions positions{{"shoulder", 0.3}, {"elbow", -0.2}};
    const Transform unit{Vector3{10, 0, 0}, quat::from_yaw(0.7)};
    const auto expected = fk::compute_frame_pose(tree, "forearm", unit, positions);
    CHECK(expected.ok());

    for (double scale : {0.5, 2.0, 1e-5})
    {
        const Quaternion q = unit.rotation;
        const Transform scaled{unit.translation,
                               Quaternion{.x = scale * q.x, .y = scale * q.y, .z = scale * q.z, .w = scale * q.w}};
        const auto pose = fk::compute_frame_pose(tree, "forearm", scaled, positions);
        CHECK(pose.ok());
        CHECK(pose.value().available);
        CHECK_NEAR(pose.value().transform, expected.value().transform, 1e-12);
    }
}

TEST(a_degenerate_base_transform_is_an_error_not_an_available_pose)
{
    // An all-zero rotation is the shape an unset proto3 rotation field arrives
    // in. Treating it as identity here would stamp a confidently wrong
    // orientation `available`; the lenient reading belongs in compat, not here.
    const RobotTree tree = RobotTree::build(two_link_arm());
    const fk::JointPositions positions{{"shoulder", 0.0}, {"elbow", 0.0}};
    const double nan = std::numeric_limits<double>::quiet_NaN();

    const Transform zero_rotation{Vector3::zero(), Quaternion{.x = 0, .y = 0, .z = 0, .w = 0}};
    const auto degenerate = fk::compute_frame_pose(tree, "forearm", zero_rotation, positions);
    CHECK(!degenerate.ok());
    CHECK(degenerate.error().code == ErrorCode::kInvalidQuaternion);
    CHECK(degenerate.error().subject == "forearm");

    const Transform nan_translation{Vector3{nan, 0, 0}, Quaternion::identity()};
    const auto non_finite = fk::compute_frame_pose(tree, "forearm", nan_translation, positions);
    CHECK(!non_finite.ok());
    CHECK(non_finite.error().code == ErrorCode::kNonFiniteValue);

    const Transform nan_rotation{Vector3::zero(), Quaternion{.x = 0, .y = 0, .z = 0, .w = nan}};
    const auto non_finite_rotation = fk::compute_frame_pose(tree, "forearm", nan_rotation, positions);
    CHECK(!non_finite_rotation.ok());

    // The base frame short-circuits before chain resolution; it is validated too.
    const auto base_frame = fk::compute_frame_pose(tree, "base_link", zero_rotation, positions);
    CHECK(!base_frame.ok());
    CHECK(base_frame.error().code == ErrorCode::kInvalidQuaternion);
}

TEST(a_missing_joint_position_makes_the_frame_unavailable_not_wrong)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const fk::JointPositions positions{{"shoulder", 0.3}};
    const auto pose = fk::compute_frame_pose(tree, "forearm", Transform::identity(), positions);
    CHECK(pose.ok());
    CHECK(!pose.value().available);
    const std::vector<std::string> expected{"elbow"};
    CHECK(pose.value().missing_joints == expected);
}

TEST(missing_joints_are_reported_sorted_and_complete)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const auto pose = fk::compute_frame_pose(tree, "forearm", Transform::identity(), {});
    CHECK(pose.ok());
    const std::vector<std::string> expected{"elbow", "shoulder"};
    CHECK(pose.value().missing_joints == expected);
}

TEST(a_fixed_only_chain_needs_no_joint_state)
{
    RobotDescription description;
    description.links = {"base_link", "mount"};
    description.joints = {make_joint("mount_joint", "base_link", "mount", JointType::kFixed,
                                     Transform{Vector3{0, 0, 0.25}, Quaternion::identity()})};
    const RobotTree tree = RobotTree::build(description);
    const auto pose = fk::compute_frame_pose(tree, "mount", Transform::identity(), {});
    CHECK(pose.ok());
    CHECK(pose.value().available);
    CHECK_NEAR(pose.value().transform.translation, (Vector3{0, 0, 0.25}), cwtest::kEps);
}

// --- mimic joints -----------------------------------------------------------

TEST(a_mimic_joint_follows_its_source)
{
    RobotDescription description;
    description.links = {"base_link", "left_finger", "right_finger"};
    description.joints = {
        make_joint("left_finger_joint", "base_link", "left_finger", JointType::kPrismatic, Transform::identity(),
                   Vector3{1, 0, 0}),
        make_joint("right_finger_joint", "base_link", "right_finger", JointType::kPrismatic, Transform::identity(),
                   Vector3{1, 0, 0}),
    };
    description.joints[1].has_mimic = true;
    description.joints[1].mimic.source_joint = "left_finger_joint";
    description.joints[1].mimic.multiplier = -1.0;
    description.joints[1].mimic.offset = 0.01;

    const RobotTree tree = RobotTree::build(description);
    const fk::JointPositions positions{{"left_finger_joint", 0.02}};
    const auto right = fk::compute_frame_pose(tree, "right_finger", Transform::identity(), positions);
    CHECK(right.ok());
    CHECK(right.value().available);
    // -1 * 0.02 + 0.01 = -0.01
    CHECK_NEAR(right.value().transform.translation, (Vector3{-0.01, 0, 0}), 1e-12);
}

TEST(a_mimic_joint_requires_its_source_not_itself)
{
    RobotDescription description;
    description.links = {"base_link", "finger"};
    description.joints = {make_joint("finger_joint", "base_link", "finger", JointType::kPrismatic,
                                     Transform::identity(), Vector3{1, 0, 0})};
    description.joints[0].has_mimic = true;
    description.joints[0].mimic.source_joint = "driver_joint";

    const RobotTree tree = RobotTree::build(description);
    const auto chain = fk::resolve_chain(tree, "finger");
    CHECK(chain.ok());
    const std::vector<std::string> expected{"driver_joint"};
    CHECK(chain.value().required_joints == expected);

    // Reporting the mimic joint's own name does not satisfy it.
    const auto wrong = fk::compute_frame_pose(tree, "finger", Transform::identity(), {{"finger_joint", 0.05}});
    CHECK(wrong.ok());
    CHECK(!wrong.value().available);
    CHECK(wrong.value().missing_joints == expected);

    const auto right = fk::compute_frame_pose(tree, "finger", Transform::identity(), {{"driver_joint", 0.05}});
    CHECK(right.ok());
    CHECK(right.value().available);
}

// --- batch ------------------------------------------------------------------

TEST(compute_frame_poses_matches_calling_one_at_a_time)
{
    const RobotTree tree = RobotTree::build(two_link_arm());
    const fk::JointPositions positions{{"shoulder", 0.3}, {"elbow", -0.2}};
    const std::vector<std::string> frames{"base_link", "forearm", "wrist_camera", "nope"};
    const auto batch = fk::compute_frame_poses(tree, frames, Transform::identity(), positions);
    CHECK(batch.size() == frames.size());
    for (std::size_t index = 0; index < frames.size(); ++index)
    {
        const auto single = fk::compute_frame_pose(tree, frames[index], Transform::identity(), positions);
        CHECK(batch[index].ok() == single.ok());
        if (single.ok())
        {
            CHECK(batch[index].value().available == single.value().available);
            CHECK_NEAR(batch[index].value().transform, single.value().transform, cwtest::kEps);
        }
        else
        {
            CHECK(batch[index].error().code == single.error().code);
        }
    }
}

int main() { return cwtest::run_all("fk"); }
