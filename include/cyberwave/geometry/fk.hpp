// Serial-chain forward kinematics over a parsed robot description.
//
// This module deliberately does *not* parse the Universal Robot Schema, URDF,
// MJCF or USD. Each language keeps its own schema reader -- that is where the
// format quirks and the telemetry policy live -- and hands the result here as
// a RobotDescription. Everything downstream of that (validation, root
// detection, chain resolution, transform composition) happens once, in this
// file, so a frame pose computed in the backend, the SDK, the browser and an
// edge driver is the same pose.
//
// URDF conventions, all of them:
//
//  * A joint's origin is applied *before* its motion.
//  * `fixed` contributes its origin only.
//  * `revolute`/`continuous` rotate about the normalized axis by the reported
//    position, in radians.
//  * `prismatic` translates along the normalized axis by the reported position,
//    in metres.
//  * A near-zero origin quaternion is an error, not an identity.
#ifndef CYBERWAVE_GEOMETRY_FK_HPP
#define CYBERWAVE_GEOMETRY_FK_HPP

#include "cyberwave/geometry/errors.hpp"
#include "cyberwave/geometry/types.hpp"

#include <cstddef>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace cyberwave
{
namespace geometry
{
namespace fk
{

/// Joint kinds whose motion is a single scalar the core can compose. Anything
/// else (floating, planar, a custom vendor type) parses to kUnsupported, and
/// frames below it resolve as an error rather than as a guess.
enum class JointType : std::int32_t
{
    kFixed = 0,
    kRevolute = 1,
    kContinuous = 2,
    kPrismatic = 3,
    kUnsupported = 4,
};

/// Case- and whitespace-insensitive, matching the schema readers that feed it.
/// An unrecognized or empty string yields kUnsupported; supplying the URDF
/// default of "fixed" for an absent `type` is the schema reader's job, not this
/// function's.
JointType joint_type_from_string(const std::string& text);
const char* joint_type_name(JointType type);

/// True when the joint's transform depends on a reported position.
bool joint_type_is_actuated(JointType type);
bool joint_type_is_supported(JointType type);

/// A joint whose position is a linear function of another joint's.
///
/// Real robots use these for gripper linkages. Ignoring them would leave every
/// finger-tip frame at its home position while the gripper visibly opens and
/// closes, so FK resolves them before composing the chain.
struct MimicSpec
{
    std::string source_joint;
    double multiplier = 1.0;
    double offset = 0.0;

    double apply(double source_position) const { return multiplier * source_position + offset; }
};

/// One joint as the caller's schema reader found it. `origin` and `axis` are
/// raw: RobotTree::build normalizes them and reports the ones it cannot.
struct JointDescription
{
    std::string name;
    std::string parent_link;
    std::string child_link;
    JointType type = JointType::kFixed;
    Transform origin = Transform::identity();
    /// URDF's default joint axis.
    Vector3 axis{0.0, 0.0, 1.0};
    bool has_mimic = false;
    MimicSpec mimic;
};

/// A frame rigidly attached to a link: a camera, an IMU, a tool tip.
struct SensorDescription
{
    std::string name;
    std::string parent_link;
    Transform extrinsic = Transform::identity();
};

struct RobotDescription
{
    /// Links the schema declares. A link mentioned only as a joint's parent
    /// (the usual synthetic `world` anchor) still participates in root
    /// detection but is not itself selectable as a frame.
    std::vector<std::string> links;
    std::vector<JointDescription> joints;
    std::vector<SensorDescription> sensors;
};

/// A joint that survived validation. Its origin rotation is normalized.
struct Joint
{
    std::string name;
    std::string parent_link;
    std::string child_link;
    JointType type = JointType::kFixed;
    Transform origin = Transform::identity();
    Vector3 axis{0.0, 0.0, 1.0};
    bool has_mimic = false;
    MimicSpec mimic;

    bool is_actuated() const { return joint_type_is_actuated(type); }
    bool is_supported() const { return joint_type_is_supported(type); }
};

struct Sensor
{
    std::string name;
    std::string parent_link;
    Transform extrinsic = Transform::identity();
};

/// A validated kinematic view of one robot description.
///
/// Structural problems are *collected*, not raised: a robot whose wrist camera
/// is unreachable must still report its base pose and joint state, so the
/// caller decides per frame what is unavailable.
class RobotTree
{
public:
    static RobotTree build(const RobotDescription& description);

    /// Topological root: a link that is never a joint's child.
    const std::string& root_link() const { return root_link_; }

    /// The frame the robot's own position/rotation topics describe. Often not
    /// the topological root, which is frequently a synthetic `world` anchor.
    const std::string& base_frame() const { return base_frame_; }

    /// Declared links, sorted.
    const std::vector<std::string>& links() const { return links_; }
    const std::vector<Joint>& joints() const { return joints_; }
    const std::vector<Sensor>& sensors() const { return sensors_; }
    const std::vector<Error>& errors() const { return errors_; }

    /// Every selectable frame: declared links plus sensor frames, sorted.
    std::vector<std::string> frame_names() const;

    /// True once a base frame could be identified -- the tree can serve base
    /// telemetry even when some frames are unreachable.
    bool is_usable() const { return !base_frame_.empty(); }
    bool is_articulated() const;

    const Joint* joint_by_name(const std::string& name) const;
    const Joint* joint_by_child(const std::string& child_link) const;
    const Sensor* sensor_by_name(const std::string& name) const;

    /// Sentinel returned by joint_index_for_child() when there is no such joint.
    static constexpr std::size_t kNoJoint = static_cast<std::size_t>(-1);

    /// Index into joints() of the joint whose child is `child_link`. Chains are
    /// carried as indices rather than pointers so that a ChainResolution stays
    /// valid across the C ABI and can be handed to a binding unchanged.
    std::size_t joint_index_for_child(const std::string& child_link) const;

private:
    std::string root_link_;
    std::string base_frame_;
    std::vector<std::string> links_;
    std::vector<Joint> joints_;
    std::vector<Sensor> sensors_;
    std::vector<Error> errors_;
    std::unordered_map<std::string, std::size_t> joint_index_by_name_;
    std::unordered_map<std::string, std::size_t> joint_index_by_child_;
    std::unordered_map<std::string, std::size_t> sensor_index_by_name_;
};

/// The unique base-to-frame path.
struct ChainResolution
{
    /// Indices into RobotTree::joints(), root-side first.
    std::vector<std::size_t> joint_indices;
    /// Set when the target frame is a sensor rather than a link.
    bool has_sensor = false;
    std::string sensor_name;
    /// Joint names whose reported positions this frame depends on, with mimic
    /// joints replaced by their sources. Sorted.
    std::vector<std::string> required_joints;
};

/// Walk the unique chain from the tree's base frame to `frame_name`.
///
/// Returns an error rather than a guess for a frame that is unknown,
/// disconnected, cyclic, or gated behind a joint type this engine cannot
/// compose: a default transform persisted as if real is far worse than an
/// honestly missing frame.
Result<ChainResolution> resolve_chain(const RobotTree& tree, const std::string& frame_name);

/// Reported joint positions, in radians for revolute/continuous and metres for
/// prismatic. Ordered so that "which joints are missing" is deterministic.
using JointPositions = std::map<std::string, double>;

struct FramePose
{
    /// False when the chain resolved but the state to evaluate it had not been
    /// reported; `missing_joints` then says which.
    bool available = false;
    Transform transform = Transform::identity();
    /// Sorted names of joints the chain needed that had no reported position.
    std::vector<std::string> missing_joints;
};

/// One joint's effective position, applying any mimic relation. Returns false
/// when the position (or its mimic source) has not been reported.
bool joint_position_for(const Joint& joint, const JointPositions& positions, double* out_position);

/// A joint's origin composed with its motion, in URDF order: the origin places
/// the joint frame in its parent, then the motion happens about or along the
/// axis *in that frame*. The other order puts a wrist camera in visibly the
/// wrong place whenever the joint origin carries a rotation.
Result<Transform> joint_transform(const Joint& joint, double position);

/// World pose of `frame_name`.
///
/// `base` is the already-world-resolved pose of `tree.base_frame()`: the caller
/// applies the environment's navigation transform before calling, so this stays
/// a pure function of the robot tree and the joint state.
Result<FramePose> compute_frame_pose(const RobotTree& tree, const std::string& frame_name, const Transform& base,
                                     const JointPositions& positions);

/// Several frames of the *same* robot in one call.
///
/// This is call amortization, not vectorization: the loop is the same scalar
/// code, it just crosses the FFI boundary once for a telemetry tick that
/// records twenty frames instead of twenty times. Results are positional,
/// one per requested frame.
std::vector<Result<FramePose>> compute_frame_poses(const RobotTree& tree, const std::vector<std::string>& frame_names,
                                                   const Transform& base, const JointPositions& positions);

} // namespace fk
} // namespace geometry
} // namespace cyberwave

#endif // CYBERWAVE_GEOMETRY_FK_HPP
