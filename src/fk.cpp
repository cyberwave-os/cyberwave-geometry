#include "cyberwave/geometry/fk.hpp"

#include "cyberwave/geometry/quaternion.hpp"
#include "cyberwave/geometry/transform.hpp"
#include "cyberwave/geometry/vector.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <set>
#include <unordered_set>

namespace cyberwave
{
namespace geometry
{
namespace fk
{
namespace
{

std::string trimmed_lower(const std::string& text)
{
    std::size_t begin = 0;
    std::size_t end = text.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(text[begin])) != 0)
    {
        ++begin;
    }
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0)
    {
        --end;
    }
    std::string out = text.substr(begin, end - begin);
    for (char& character : out)
    {
        character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    }
    return out;
}

/// Sorted, de-duplicated copy. Every list the core hands back is ordered so
/// that "which frames" and "which joints are missing" do not depend on the
/// caller's hash seed.
std::vector<std::string> sorted_unique(std::vector<std::string> values)
{
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
    return values;
}

} // namespace

JointType joint_type_from_string(const std::string& text)
{
    const std::string key = trimmed_lower(text);
    if (key == "fixed")
    {
        return JointType::kFixed;
    }
    if (key == "revolute")
    {
        return JointType::kRevolute;
    }
    if (key == "continuous")
    {
        return JointType::kContinuous;
    }
    if (key == "prismatic")
    {
        return JointType::kPrismatic;
    }
    return JointType::kUnsupported;
}

const char* joint_type_name(JointType type)
{
    switch (type)
    {
        case JointType::kFixed:
            return "fixed";
        case JointType::kRevolute:
            return "revolute";
        case JointType::kContinuous:
            return "continuous";
        case JointType::kPrismatic:
            return "prismatic";
        case JointType::kUnsupported:
            return "unsupported";
    }
    return "unsupported";
}

bool joint_type_is_actuated(JointType type)
{
    return type == JointType::kRevolute || type == JointType::kContinuous || type == JointType::kPrismatic;
}

bool joint_type_is_supported(JointType type) { return type != JointType::kUnsupported; }

// ---------------------------------------------------------------------------
// Tree construction
// ---------------------------------------------------------------------------

namespace
{

/// Descend through fixed joints from the topological root until reaching a link
/// the description actually declares.
///
/// The topological root is often a synthetic world anchor: real schemas carry a
/// `fixed` base joint from a `world` link that is not itself declared. The
/// robot's own position/rotation topics refer to its root *body*, not to that
/// anchor, so the base frame is the first declared link below it. An anchor
/// with zero or several fixed children gives no unambiguous body to call "the
/// base", and the topological root stands.
std::string resolve_base_frame(const std::string& root_link, const std::unordered_set<std::string>& declared_links,
                               const std::unordered_map<std::string, std::vector<std::size_t>>& joint_indices_by_parent,
                               const std::vector<Joint>& joints)
{
    if (declared_links.count(root_link) != 0)
    {
        return root_link;
    }
    std::string current = root_link;
    std::unordered_set<std::string> seen{current};
    while (true)
    {
        const auto found = joint_indices_by_parent.find(current);
        if (found == joint_indices_by_parent.end())
        {
            return root_link;
        }
        std::vector<std::size_t> fixed_children;
        for (std::size_t index : found->second)
        {
            if (joints[index].type == JointType::kFixed)
            {
                fixed_children.push_back(index);
            }
        }
        if (fixed_children.size() != 1)
        {
            return root_link;
        }
        current = joints[fixed_children[0]].child_link;
        if (seen.count(current) != 0)
        {
            return root_link;
        }
        seen.insert(current);
        if (declared_links.count(current) != 0)
        {
            return current;
        }
    }
}

} // namespace

RobotTree RobotTree::build(const RobotDescription& description)
{
    RobotTree tree;

    std::unordered_set<std::string> declared_links;
    std::vector<std::string> declared_list;
    for (const std::string& link : description.links)
    {
        if (link.empty())
        {
            continue;
        }
        if (declared_links.insert(link).second)
        {
            declared_list.push_back(link);
        }
    }
    tree.links_ = sorted_unique(std::move(declared_list));

    // --- joints -------------------------------------------------------------
    for (const JointDescription& raw : description.joints)
    {
        // A joint missing any of its three identifiers cannot take part in the
        // tree at all; it is dropped rather than reported, matching the schema
        // readers that never emit one.
        if (raw.name.empty() || raw.parent_link.empty() || raw.child_link.empty())
        {
            continue;
        }
        const Result<Transform> origin = tf::normalize(raw.origin);
        if (!origin)
        {
            tree.errors_.emplace_back(ErrorCode::kInvalidJointPose, raw.name,
                                      "joint origin quaternion has a near-zero norm");
            continue;
        }
        Joint joint;
        joint.name = raw.name;
        joint.parent_link = raw.parent_link;
        joint.child_link = raw.child_link;
        joint.type = raw.type;
        joint.origin = origin.value();
        joint.axis = raw.axis;
        joint.has_mimic = raw.has_mimic && !raw.mimic.source_joint.empty();
        joint.mimic = raw.mimic;
        tree.joints_.push_back(std::move(joint));
    }

    // --- sensors ------------------------------------------------------------
    for (const SensorDescription& raw : description.sensors)
    {
        if (raw.name.empty() || raw.parent_link.empty())
        {
            continue;
        }
        const Result<Transform> extrinsic = tf::normalize(raw.extrinsic);
        if (!extrinsic)
        {
            tree.errors_.emplace_back(ErrorCode::kInvalidSensorPose, raw.name,
                                      "sensor extrinsic quaternion has a near-zero norm");
            continue;
        }
        Sensor sensor;
        sensor.name = raw.name;
        sensor.parent_link = raw.parent_link;
        sensor.extrinsic = extrinsic.value();
        tree.sensors_.push_back(std::move(sensor));
    }
    std::sort(tree.sensors_.begin(), tree.sensors_.end(),
              [](const Sensor& a, const Sensor& b) { return a.name < b.name; });

    for (std::size_t index = 0; index < tree.joints_.size(); ++index)
    {
        tree.joint_index_by_name_[tree.joints_[index].name] = index;
    }
    for (std::size_t index = 0; index < tree.sensors_.size(); ++index)
    {
        tree.sensor_index_by_name_[tree.sensors_[index].name] = index;
    }

    // --- topology -----------------------------------------------------------
    std::unordered_map<std::string, std::vector<std::size_t>> joint_indices_by_parent;
    for (std::size_t index = 0; index < tree.joints_.size(); ++index)
    {
        const Joint& joint = tree.joints_[index];
        const auto existing = tree.joint_index_by_child_.find(joint.child_link);
        if (existing != tree.joint_index_by_child_.end())
        {
            // Two joints claim the same child, so there is no unique chain to
            // it. The first one wins and the second is reported; picking either
            // silently would make the pose depend on schema ordering.
            tree.errors_.emplace_back(ErrorCode::kAmbiguousParent, joint.child_link,
                                      "link is the child of both '" + tree.joints_[existing->second].name + "' and '" +
                                          joint.name + "'");
            continue;
        }
        tree.joint_index_by_child_[joint.child_link] = index;
        joint_indices_by_parent[joint.parent_link].push_back(index);
    }

    // Every link mentioned anywhere, so a synthetic `world` parent still
    // participates in root detection.
    std::set<std::string> all_links(tree.links_.begin(), tree.links_.end());
    for (const auto& entry : tree.joint_index_by_child_)
    {
        all_links.insert(entry.first);
    }
    for (const auto& entry : joint_indices_by_parent)
    {
        all_links.insert(entry.first);
    }
    if (all_links.empty())
    {
        tree.errors_.emplace_back(ErrorCode::kNoLinks, "", "description declares no links");
        return tree;
    }

    std::vector<std::string> roots;
    for (const std::string& link : all_links)
    {
        if (tree.joint_index_by_child_.count(link) == 0)
        {
            roots.push_back(link);
        }
    }
    if (roots.empty())
    {
        tree.errors_.emplace_back(ErrorCode::kNoRoot, "", "every link has a parent joint, so the tree is cyclic");
        return tree;
    }

    // `all_links` is a std::set, so `roots` is already sorted: the chosen root
    // is stable across runs and across languages.
    tree.root_link_ = roots.front();
    if (roots.size() > 1)
    {
        // Disconnected sub-trees. The first root still serves base telemetry;
        // frames under the others resolve as unavailable with this reason.
        std::string joined;
        for (std::size_t index = 0; index < roots.size(); ++index)
        {
            joined += (index == 0 ? "" : ", ") + roots[index];
        }
        tree.errors_.emplace_back(ErrorCode::kMultipleRoots, tree.root_link_,
                                  "description is disconnected; roots: " + joined);
    }
    tree.base_frame_ = resolve_base_frame(tree.root_link_, declared_links, joint_indices_by_parent, tree.joints_);
    return tree;
}

std::vector<std::string> RobotTree::frame_names() const
{
    std::vector<std::string> names = links_;
    for (const Sensor& sensor : sensors_)
    {
        names.push_back(sensor.name);
    }
    return sorted_unique(std::move(names));
}

bool RobotTree::is_articulated() const
{
    for (const Joint& joint : joints_)
    {
        if (joint.is_actuated())
        {
            return true;
        }
    }
    return false;
}

const Joint* RobotTree::joint_by_name(const std::string& name) const
{
    const auto found = joint_index_by_name_.find(name);
    return found == joint_index_by_name_.end() ? nullptr : &joints_[found->second];
}

const Joint* RobotTree::joint_by_child(const std::string& child_link) const
{
    const auto found = joint_index_by_child_.find(child_link);
    return found == joint_index_by_child_.end() ? nullptr : &joints_[found->second];
}

const Sensor* RobotTree::sensor_by_name(const std::string& name) const
{
    const auto found = sensor_index_by_name_.find(name);
    return found == sensor_index_by_name_.end() ? nullptr : &sensors_[found->second];
}

std::size_t RobotTree::joint_index_for_child(const std::string& child_link) const
{
    const auto found = joint_index_by_child_.find(child_link);
    return found == joint_index_by_child_.end() ? kNoJoint : found->second;
}

// ---------------------------------------------------------------------------
// Chain resolution
// ---------------------------------------------------------------------------

Result<ChainResolution> resolve_chain(const RobotTree& tree, const std::string& frame_name)
{
    if (!tree.is_usable())
    {
        return Result<ChainResolution>::failure(ErrorCode::kNoRoot, frame_name, "robot tree has no base frame");
    }

    const Sensor* sensor = tree.sensor_by_name(frame_name);
    const std::string target_link = sensor != nullptr ? sensor->parent_link : frame_name;

    if (sensor == nullptr && !std::binary_search(tree.links().begin(), tree.links().end(), frame_name) &&
        tree.joint_by_child(frame_name) == nullptr)
    {
        return Result<ChainResolution>::failure(ErrorCode::kUnknownFrame, frame_name,
                                                "frame is not a link or sensor in the robot description");
    }

    std::vector<std::size_t> reversed_chain;
    std::unordered_set<std::string> seen;
    std::string current = target_link;
    while (current != tree.base_frame())
    {
        if (seen.count(current) != 0)
        {
            return Result<ChainResolution>::failure(ErrorCode::kCycle, frame_name,
                                                    "cycle through link '" + current + "'");
        }
        seen.insert(current);
        const Joint* joint = tree.joint_by_child(current);
        if (joint == nullptr)
        {
            return Result<ChainResolution>::failure(ErrorCode::kNoRoot, frame_name,
                                                    "link '" + current + "' is not connected to base frame '" +
                                                        tree.base_frame() + "'");
        }
        if (!joint->is_supported())
        {
            return Result<ChainResolution>::failure(
                ErrorCode::kUnsupportedJointType, joint->name,
                "joint type has no single-scalar transform; frames below it cannot be derived");
        }
        reversed_chain.push_back(tree.joint_index_for_child(current));
        current = joint->parent_link;
    }

    ChainResolution resolution;
    resolution.joint_indices.assign(reversed_chain.rbegin(), reversed_chain.rend());
    if (sensor != nullptr)
    {
        resolution.has_sensor = true;
        resolution.sensor_name = sensor->name;
    }

    std::vector<std::string> required;
    for (std::size_t index : resolution.joint_indices)
    {
        const Joint& joint = tree.joints()[index];
        if (!joint.is_actuated())
        {
            continue;
        }
        // A mimic joint's own position is never reported; its source's is.
        required.push_back(joint.has_mimic ? joint.mimic.source_joint : joint.name);
    }
    resolution.required_joints = sorted_unique(std::move(required));
    return resolution;
}

// ---------------------------------------------------------------------------
// Pose computation
// ---------------------------------------------------------------------------

bool joint_position_for(const Joint& joint, const JointPositions& positions, double* out_position)
{
    if (!joint.is_actuated())
    {
        *out_position = 0.0;
        return true;
    }
    if (joint.has_mimic)
    {
        const auto found = positions.find(joint.mimic.source_joint);
        if (found == positions.end())
        {
            return false;
        }
        *out_position = joint.mimic.apply(found->second);
        return true;
    }
    const auto found = positions.find(joint.name);
    if (found == positions.end())
    {
        return false;
    }
    *out_position = found->second;
    return true;
}

Result<Transform> joint_transform(const Joint& joint, double position)
{
    if (!joint.is_actuated())
    {
        return joint.origin;
    }
    if (!std::isfinite(position))
    {
        return Result<Transform>::failure(ErrorCode::kNonFiniteValue, joint.name, "joint position is not finite");
    }
    if (joint.type == JointType::kPrismatic)
    {
        const Result<Vector3> axis = vec::normalize_axis(joint.axis);
        if (!axis)
        {
            return Result<Transform>::failure(ErrorCode::kInvalidAxis, joint.name,
                                              "joint axis is degenerate and cannot be normalized");
        }
        const Transform motion{vec::scale(axis.value(), position), Quaternion::identity()};
        return tf::compose(joint.origin, motion);
    }
    const Result<Quaternion> rotation = quat::from_axis_angle(joint.axis, position);
    if (!rotation)
    {
        return Result<Transform>::failure(ErrorCode::kInvalidAxis, joint.name,
                                          "joint axis is degenerate and cannot be normalized");
    }
    const Transform motion{Vector3::zero(), rotation.value()};
    return tf::compose(joint.origin, motion);
}

Result<FramePose> compute_frame_pose(const RobotTree& tree, const std::string& frame_name, const Transform& base,
                                     const JointPositions& positions)
{
    // `base` is the one geometric input this engine takes from the caller
    // rather than from the validated tree, and everything below -- tf::compose,
    // quat::rotate_unit -- documents a normalized rotation as a precondition.
    // Checking it here is what makes that precondition true rather than hoped
    // for, and keeps a degenerate or non-finite base an honest error instead of
    // a pose reported as available.
    const Result<Transform> anchor = tf::normalize(base);
    if (!anchor)
    {
        return Result<FramePose>::failure(anchor.error().code, frame_name,
                                          "base transform is not a valid pose: " + anchor.error().detail);
    }

    if (frame_name == tree.base_frame())
    {
        FramePose pose;
        pose.available = true;
        pose.transform = anchor.value();
        return pose;
    }

    const Result<ChainResolution> resolution = resolve_chain(tree, frame_name);
    if (!resolution)
    {
        return Result<FramePose>(resolution.error());
    }

    std::vector<std::string> missing;
    for (const std::string& name : resolution.value().required_joints)
    {
        if (positions.count(name) == 0)
        {
            missing.push_back(name);
        }
    }
    if (!missing.empty())
    {
        FramePose pose;
        pose.available = false;
        pose.missing_joints = std::move(missing);
        return pose;
    }

    Transform current = anchor.value();
    for (std::size_t index : resolution.value().joint_indices)
    {
        const Joint& joint = tree.joints()[index];
        double position = 0.0;
        if (!joint_position_for(joint, positions, &position))
        {
            // Unreachable via the `missing` check above; kept so a future caller
            // cannot reach the composition with an unresolved joint.
            FramePose pose;
            pose.available = false;
            pose.missing_joints.push_back(joint.name);
            return pose;
        }
        const Result<Transform> step = joint_transform(joint, position);
        if (!step)
        {
            return Result<FramePose>(step.error());
        }
        current = tf::compose(current, step.value());
    }

    if (resolution.value().has_sensor)
    {
        const Sensor* sensor = tree.sensor_by_name(resolution.value().sensor_name);
        if (sensor != nullptr)
        {
            current = tf::compose(current, sensor->extrinsic);
        }
    }

    FramePose pose;
    pose.available = true;
    pose.transform = current;
    return pose;
}

std::vector<Result<FramePose>> compute_frame_poses(const RobotTree& tree, const std::vector<std::string>& frame_names,
                                                   const Transform& base, const JointPositions& positions)
{
    std::vector<Result<FramePose>> results;
    results.reserve(frame_names.size());
    for (const std::string& frame_name : frame_names)
    {
        results.push_back(compute_frame_pose(tree, frame_name, base, positions));
    }
    return results;
}

} // namespace fk
} // namespace geometry
} // namespace cyberwave
