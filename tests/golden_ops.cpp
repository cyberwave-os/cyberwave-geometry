#include "golden_ops.hpp"

#include "cyberwave/geometry/geometry.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace golden
{
namespace
{

using namespace cyberwave::geometry;
using minijson::Json;

constexpr double kPi = 3.14159265358979323846;

// --- value <-> json ---------------------------------------------------------

Json to_json(const Vector3& v)
{
    Json out = Json::object();
    out.set("x", Json::number(v.x));
    out.set("y", Json::number(v.y));
    out.set("z", Json::number(v.z));
    return out;
}

Json to_json(const Quaternion& q)
{
    // Named components, never a bare four-element array: the golden file is
    // read by four languages and a positional order would be guessed wrong.
    Json out = Json::object();
    out.set("x", Json::number(q.x));
    out.set("y", Json::number(q.y));
    out.set("z", Json::number(q.z));
    out.set("w", Json::number(q.w));
    return out;
}

Json to_json(const Geodetic& g)
{
    // Named fields, and the same spelling the `twin/{uuid}/gps` payload uses, so
    // a runner in another language cannot read latitude as longitude.
    Json out = Json::object();
    out.set("latitude", Json::number(g.latitude_deg));
    out.set("longitude", Json::number(g.longitude_deg));
    out.set("altitude", Json::number(g.altitude_m));
    return out;
}

Json to_json(const Transform& t)
{
    Json out = Json::object();
    out.set("translation", to_json(t.translation));
    out.set("rotation", to_json(t.rotation));
    return out;
}

double number_at(const Json& node, const std::string& key, double fallback = 0.0)
{
    const Json& value = node.get(key);
    return value.type() == Json::Type::kNumber ? value.as_number() : fallback;
}

Vector3 vector_from(const Json& node)
{
    return Vector3{number_at(node, "x"), number_at(node, "y"), number_at(node, "z")};
}

Quaternion quaternion_from(const Json& node)
{
    return Quaternion{.x = number_at(node, "x"),
                      .y = number_at(node, "y"),
                      .z = number_at(node, "z"),
                      .w = number_at(node, "w", 1.0)};
}

Geodetic geodetic_from(const Json& node)
{
    return Geodetic{number_at(node, "latitude"), number_at(node, "longitude"), number_at(node, "altitude")};
}

GeoAnchor anchor_from(const Json& node) { return GeoAnchor{geodetic_from(node), number_at(node, "heading_deg")}; }

Transform transform_from(const Json& node)
{
    return Transform{vector_from(node.get("translation")), quaternion_from(node.get("rotation"))};
}

Json ok_status()
{
    Json out = Json::object();
    out.set("status", Json::string("ok"));
    return out;
}

Json error_status(ErrorCode code)
{
    Json out = Json::object();
    out.set("status", Json::string(error_code_name(code)));
    return out;
}

// --- robot descriptions -----------------------------------------------------

Json joint_json(const std::string& name, const std::string& parent, const std::string& child, const std::string& type,
                const Transform& origin, const Vector3& axis)
{
    Json out = Json::object();
    out.set("name", Json::string(name));
    out.set("parent_link", Json::string(parent));
    out.set("child_link", Json::string(child));
    out.set("type", Json::string(type));
    out.set("origin", to_json(origin));
    out.set("axis", to_json(axis));
    return out;
}

fk::JointDescription joint_from(const Json& node)
{
    fk::JointDescription joint;
    joint.name = node.get("name").as_string();
    joint.parent_link = node.get("parent_link").as_string();
    joint.child_link = node.get("child_link").as_string();
    joint.type = fk::joint_type_from_string(node.get("type").as_string());
    joint.origin = transform_from(node.get("origin"));
    joint.axis = vector_from(node.get("axis"));
    if (node.has("mimic"))
    {
        const Json& mimic = node.get("mimic");
        joint.has_mimic = true;
        joint.mimic.source_joint = mimic.get("source_joint").as_string();
        joint.mimic.multiplier = number_at(mimic, "multiplier", 1.0);
        joint.mimic.offset = number_at(mimic, "offset", 0.0);
    }
    return joint;
}

fk::RobotDescription robot_from(const Json& node)
{
    fk::RobotDescription description;
    const Json& links = node.get("links");
    for (std::size_t index = 0; index < links.size(); ++index)
    {
        description.links.push_back(links.at(index).as_string());
    }
    const Json& joints = node.get("joints");
    for (std::size_t index = 0; index < joints.size(); ++index)
    {
        description.joints.push_back(joint_from(joints.at(index)));
    }
    const Json& sensors = node.get("sensors");
    for (std::size_t index = 0; index < sensors.size(); ++index)
    {
        const Json& raw = sensors.at(index);
        fk::SensorDescription sensor;
        sensor.name = raw.get("name").as_string();
        sensor.parent_link = raw.get("parent_link").as_string();
        // `pose` is the pre-0.2.0 spelling of the same field.
        sensor.extrinsic = transform_from(raw.has("extrinsic") ? raw.get("extrinsic") : raw.get("pose"));
        description.sensors.push_back(sensor);
    }
    return description;
}

Json string_array(const std::vector<std::string>& values)
{
    Json out = Json::array();
    for (const std::string& value : values)
    {
        out.push_back(Json::string(value));
    }
    return out;
}

/// The robot descriptions every fk case refers to by name.
Json build_robots()
{
    Json robots = Json::object();

    {
        // base_link -> upper_arm (revolute +Z, 1m up) -> forearm (revolute +Y,
        // 1m along +X), with a camera on the forearm.
        Json robot = Json::object();
        robot.set("links", string_array({"base_link", "upper_arm", "forearm"}));
        Json joints = Json::array();
        joints.push_back(joint_json("shoulder", "base_link", "upper_arm", "revolute",
                                    Transform{Vector3{0, 0, 1}, Quaternion::identity()}, Vector3{0, 0, 1}));
        joints.push_back(joint_json("elbow", "upper_arm", "forearm", "revolute",
                                    Transform{Vector3{1, 0, 0}, Quaternion::identity()}, Vector3{0, 1, 0}));
        robot.set("joints", joints);
        Json sensors = Json::array();
        Json camera = Json::object();
        camera.set("name", Json::string("wrist_camera"));
        camera.set("parent_link", Json::string("forearm"));
        camera.set("extrinsic", to_json(Transform{Vector3{0.1, 0, 0}, quat::from_rpy(0, -kPi / 2, 0)}));
        sensors.push_back(camera);
        robot.set("sensors", sensors);
        robots.set("two_link_arm", robot);
    }

    {
        // A rotated joint origin, so origin-before-motion is observable, plus a
        // prismatic joint and a synthetic `world` anchor above the base.
        Json robot = Json::object();
        robot.set("links", string_array({"base_link", "tilted", "slider"}));
        Json joints = Json::array();
        joints.push_back(
            joint_json("base_joint", "world", "base_link", "fixed", Transform::identity(), Vector3{0, 0, 1}));
        joints.push_back(joint_json("tilt", "base_link", "tilted", "revolute",
                                    Transform{Vector3{0, 0, 0.5}, quat::from_rpy(0, 0, kPi / 2)}, Vector3{1, 0, 0}));
        joints.push_back(joint_json("extend", "tilted", "slider", "prismatic",
                                    Transform{Vector3{0.2, 0, 0}, quat::from_rpy(0.1, 0.2, 0.3)}, Vector3{0, 0, 3}));
        robot.set("joints", joints);
        robot.set("sensors", Json::array());
        robots.set("tilted_slider", robot);
    }

    {
        // A gripper whose right finger mimics the left with a negative
        // multiplier and an offset.
        Json robot = Json::object();
        robot.set("links", string_array({"palm", "left_finger", "right_finger"}));
        Json joints = Json::array();
        joints.push_back(joint_json("left_finger_joint", "palm", "left_finger", "prismatic", Transform::identity(),
                                    Vector3{1, 0, 0}));
        Json right = joint_json("right_finger_joint", "palm", "right_finger", "prismatic", Transform::identity(),
                                Vector3{1, 0, 0});
        Json mimic = Json::object();
        mimic.set("source_joint", Json::string("left_finger_joint"));
        mimic.set("multiplier", Json::number(-1.0));
        mimic.set("offset", Json::number(0.01));
        right.set("mimic", mimic);
        joints.push_back(right);
        robot.set("joints", joints);
        robot.set("sensors", Json::array());
        robots.set("mimic_gripper", robot);
    }

    {
        // Structurally broken on purpose: a degenerate joint origin, and a
        // second joint claiming a child that already has a parent.
        Json robot = Json::object();
        robot.set("links", string_array({"base_link", "a", "b"}));
        Json joints = Json::array();
        Json broken = joint_json("broken", "base_link", "a", "revolute", Transform::identity(), Vector3{0, 0, 1});
        Json zero = Json::object();
        zero.set("w", Json::number(0.0));
        zero.set("x", Json::number(0.0));
        zero.set("y", Json::number(0.0));
        zero.set("z", Json::number(0.0));
        Json broken_origin = Json::object();
        broken_origin.set("translation", to_json(Vector3::zero()));
        broken_origin.set("rotation", zero);
        broken.set("origin", broken_origin);
        joints.push_back(broken);
        joints.push_back(joint_json("first", "base_link", "b", "fixed", Transform::identity(), Vector3{0, 0, 1}));
        joints.push_back(joint_json("second", "base_link", "b", "fixed", Transform::identity(), Vector3{0, 0, 1}));
        robot.set("joints", joints);
        robot.set("sensors", Json::array());
        robots.set("broken_tree", robot);
    }

    {
        // A joint type the core cannot compose, so everything below it must
        // resolve as an error rather than as a frozen guess.
        Json robot = Json::object();
        robot.set("links", string_array({"base_link", "floater"}));
        Json joints = Json::array();
        joints.push_back(
            joint_json("float_joint", "base_link", "floater", "floating", Transform::identity(), Vector3{0, 0, 1}));
        robot.set("joints", joints);
        robot.set("sensors", Json::array());
        robots.set("floating_joint", robot);
    }

    return robots;
}

// --- op dispatch ------------------------------------------------------------

Json evaluate_fk_tree(const Json& robots, const Json& input)
{
    const fk::RobotTree tree = fk::RobotTree::build(robot_from(robots.get(input.get("robot").as_string())));
    Json out = Json::object();
    out.set("root_link", Json::string(tree.root_link()));
    out.set("base_frame", Json::string(tree.base_frame()));
    out.set("usable", Json::boolean(tree.is_usable()));
    out.set("articulated", Json::boolean(tree.is_articulated()));
    out.set("frames", string_array(tree.frame_names()));
    Json errors = Json::array();
    for (const Error& error : tree.errors())
    {
        Json entry = Json::object();
        entry.set("code", Json::string(error_code_name(error.code)));
        entry.set("subject", Json::string(error.subject));
        errors.push_back(entry);
    }
    out.set("errors", errors);
    return out;
}

Json evaluate_fk_frame_pose(const Json& robots, const Json& input)
{
    const fk::RobotTree tree = fk::RobotTree::build(robot_from(robots.get(input.get("robot").as_string())));
    fk::JointPositions positions;
    const Json& reported = input.get("joint_positions");
    for (const auto& entry : reported.entries())
    {
        positions[entry.first] = entry.second.as_number();
    }
    const Transform base = input.has("base") ? transform_from(input.get("base")) : Transform::identity();

    const auto result = fk::compute_frame_pose(tree, input.get("frame").as_string(), base, positions);
    if (!result)
    {
        Json out = error_status(result.error().code);
        out.set("error_subject", Json::string(result.error().subject));
        return out;
    }
    Json out = ok_status();
    out.set("available", Json::boolean(result.value().available));
    if (result.value().available)
    {
        out.set("transform", to_json(result.value().transform));
    }
    else
    {
        out.set("missing_joints", string_array(result.value().missing_joints));
    }
    return out;
}

} // namespace

Json evaluate(const Json& robots, const std::string& op, const Json& input)
{
    if (op == "quat_normalize")
    {
        const auto result = quat::normalize(quaternion_from(input.get("q")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("quaternion", to_json(result.value()));
        return out;
    }
    if (op == "quat_multiply")
    {
        Json out = ok_status();
        out.set("quaternion",
                to_json(quat::multiply(quaternion_from(input.get("a")), quaternion_from(input.get("b")))));
        return out;
    }
    if (op == "quat_conjugate")
    {
        Json out = ok_status();
        out.set("quaternion", to_json(quat::conjugate(quaternion_from(input.get("q")))));
        return out;
    }
    if (op == "quat_inverse")
    {
        const auto result = quat::inverse(quaternion_from(input.get("q")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("quaternion", to_json(result.value()));
        return out;
    }
    if (op == "quat_rotate")
    {
        const auto result = quat::rotate(quaternion_from(input.get("q")), vector_from(input.get("v")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("vector", to_json(result.value()));
        return out;
    }
    if (op == "quat_slerp" || op == "quat_nlerp")
    {
        const auto result =
            op == "quat_slerp"
                ? quat::slerp(quaternion_from(input.get("a")), quaternion_from(input.get("b")), number_at(input, "t"))
                : quat::nlerp(quaternion_from(input.get("a")), quaternion_from(input.get("b")), number_at(input, "t"));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("quaternion", to_json(result.value()));
        return out;
    }
    if (op == "quat_from_axis_angle")
    {
        const auto result = quat::from_axis_angle(vector_from(input.get("axis")), number_at(input, "angle"));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("quaternion", to_json(result.value()));
        return out;
    }
    if (op == "quat_to_axis_angle")
    {
        const auto result = quat::to_axis_angle(quaternion_from(input.get("q")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("axis", to_json(result.value().axis));
        out.set("angle", Json::number(result.value().angle));
        return out;
    }
    if (op == "quat_from_rpy")
    {
        Json out = ok_status();
        out.set("quaternion",
                to_json(quat::from_rpy(number_at(input, "roll"), number_at(input, "pitch"), number_at(input, "yaw"))));
        return out;
    }
    if (op == "quat_to_rpy")
    {
        const auto result = quat::to_rpy(quaternion_from(input.get("q")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("roll", Json::number(result.value().roll));
        out.set("pitch", Json::number(result.value().pitch));
        out.set("yaw", Json::number(result.value().yaw));
        return out;
    }
    if (op == "quat_from_yaw")
    {
        Json out = ok_status();
        out.set("quaternion", to_json(quat::from_yaw(number_at(input, "yaw"))));
        return out;
    }
    if (op == "quat_to_yaw")
    {
        const auto result = quat::to_yaw(quaternion_from(input.get("q")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("yaw", Json::number(result.value()));
        return out;
    }
    if (op == "quat_to_matrix")
    {
        const auto result = quat::to_matrix(quaternion_from(input.get("q")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json matrix = Json::array();
        for (double component : result.value().m)
        {
            matrix.push_back(Json::number(component));
        }
        Json out = ok_status();
        out.set("matrix", matrix);
        return out;
    }
    if (op == "quat_from_matrix")
    {
        Matrix3 matrix;
        const Json& raw = input.get("matrix");
        for (std::size_t index = 0; index < 9 && index < raw.size(); ++index)
        {
            matrix.m[index] = raw.at(index).as_number();
        }
        const auto result = quat::from_matrix(matrix);
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("quaternion", to_json(result.value()));
        return out;
    }
    if (op == "transform_normalize")
    {
        const auto result = tf::normalize(transform_from(input.get("t")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("transform", to_json(result.value()));
        return out;
    }
    if (op == "transform_compose")
    {
        Json out = ok_status();
        out.set("transform",
                to_json(tf::compose(transform_from(input.get("parent")), transform_from(input.get("child")))));
        return out;
    }
    if (op == "transform_inverse")
    {
        const auto result = tf::inverse(transform_from(input.get("t")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("transform", to_json(result.value()));
        return out;
    }
    if (op == "transform_apply")
    {
        Json out = ok_status();
        out.set("vector", to_json(tf::apply(transform_from(input.get("t")), vector_from(input.get("point")))));
        return out;
    }
    if (op == "transform_relative")
    {
        Json out = ok_status();
        out.set("transform",
                to_json(tf::relative(transform_from(input.get("parent")), transform_from(input.get("child")))));
        return out;
    }
    if (op == "joint_type_from_string")
    {
        Json out = ok_status();
        out.set("joint_type",
                Json::string(fk::joint_type_name(fk::joint_type_from_string(input.get("text").as_string()))));
        return out;
    }
    if (op == "geo_metres_per_degree")
    {
        const auto result = geo::metres_per_degree(number_at(input, "latitude_deg"));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("metres_per_degree_latitude", Json::number(result.value().latitude));
        out.set("metres_per_degree_longitude", Json::number(result.value().longitude));
        return out;
    }
    if (op == "geo_normalize_longitude_deg")
    {
        Json out = ok_status();
        out.set("longitude_deg", Json::number(geo::normalize_longitude_deg(number_at(input, "longitude_deg"))));
        return out;
    }
    if (op == "geo_enu_between")
    {
        const auto result = geo::enu_between(geodetic_from(input.get("origin")), geodetic_from(input.get("target")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("vector", to_json(result.value()));
        return out;
    }
    if (op == "geo_offset")
    {
        const auto result = geo::offset(geodetic_from(input.get("origin")), vector_from(input.get("enu")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("geodetic", to_json(result.value()));
        return out;
    }
    if (op == "geo_ground_distance")
    {
        const auto result = geo::ground_distance(geodetic_from(input.get("a")), geodetic_from(input.get("b")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("distance", Json::number(result.value()));
        return out;
    }
    if (op == "geo_to_local")
    {
        const auto result = geo::to_local(anchor_from(input.get("anchor")), geodetic_from(input.get("position")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("vector", to_json(result.value()));
        return out;
    }
    if (op == "geo_from_local")
    {
        const auto result = geo::from_local(anchor_from(input.get("anchor")), vector_from(input.get("position")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("geodetic", to_json(result.value()));
        return out;
    }
    if (op == "geo_to_local_pose")
    {
        const GeoPose pose{geodetic_from(input.get("pose")), quaternion_from(input.get("pose").get("rotation"))};
        const auto result = geo::to_local_pose(anchor_from(input.get("anchor")), pose);
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("transform", to_json(result.value()));
        return out;
    }
    if (op == "geo_from_local_pose")
    {
        const auto result = geo::from_local_pose(anchor_from(input.get("anchor")), transform_from(input.get("pose")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("geodetic", to_json(result.value().position));
        out.set("quaternion", to_json(result.value().orientation));
        return out;
    }
    if (op == "geo_quat_from_compass_heading")
    {
        const auto result = geo::quat_from_compass_heading(number_at(input, "heading_deg"));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("quaternion", to_json(result.value()));
        return out;
    }
    if (op == "geo_to_compass_heading_deg")
    {
        const auto result = geo::to_compass_heading_deg(quaternion_from(input.get("q")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("heading_deg", Json::number(result.value()));
        return out;
    }
    if (op == "geo_quat_from_ned_rpy")
    {
        const auto result =
            geo::quat_from_ned_rpy(number_at(input, "roll"), number_at(input, "pitch"), number_at(input, "yaw"));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("quaternion", to_json(result.value()));
        return out;
    }
    if (op == "geo_to_ned_rpy")
    {
        const auto result = geo::to_ned_rpy(quaternion_from(input.get("q")));
        if (!result)
        {
            return error_status(result.error().code);
        }
        Json out = ok_status();
        out.set("roll", Json::number(result.value().roll));
        out.set("pitch", Json::number(result.value().pitch));
        out.set("yaw", Json::number(result.value().yaw));
        return out;
    }
    if (op == "fk_tree")
    {
        return evaluate_fk_tree(robots, input);
    }
    if (op == "fk_frame_pose")
    {
        return evaluate_fk_frame_pose(robots, input);
    }

    Json out = Json::object();
    out.set("status", Json::string("unknown_op"));
    return out;
}

namespace
{

/// A stable, readable id fragment for a double.
///
/// `std::to_string` pads everything to six decimals -- `0.000000`, `-33.868800`
/// -- which makes the case ids unreadable in a diff, and a diff of this file is
/// the whole review mechanism for a behaviour change.
std::string to_id(double value)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%g", value);
    return std::string(buffer);
}

void add(Json& cases, const Json& robots, const std::string& id, const std::string& op, const Json& input)
{
    Json entry = Json::object();
    entry.set("id", Json::string(id));
    entry.set("op", Json::string(op));
    entry.set("input", input);
    entry.set("expect", evaluate(robots, op, input));
    cases.push_back(entry);
}

Json quaternion_input(const std::string& key, const Quaternion& q)
{
    Json out = Json::object();
    out.set(key, to_json(q));
    return out;
}

/// Every quaternion the algebra cases are exercised against: identity, a
/// general rotation, a half turn, a tiny rotation, an unnormalized one, and a
/// degenerate one the strict core must reject.
struct NamedQuaternion
{
    const char* name;
    Quaternion value;
};

std::vector<NamedQuaternion> sample_quaternions()
{
    return {
        {"identity", Quaternion::identity()},
        {"general", quat::from_rpy(0.4, -0.5, 0.6)},
        {"half_turn_x", quat::from_axis_angle(Vector3{1, 0, 0}, kPi).value()},
        {"half_turn_y", quat::from_axis_angle(Vector3{0, 1, 0}, kPi).value()},
        {"half_turn_z", quat::from_axis_angle(Vector3{0, 0, 1}, kPi).value()},
        {"tiny", quat::from_axis_angle(Vector3{1, 1, 1}, 1e-7).value()},
        {"negative_w", Quaternion{.x = 0.5, .y = 0.5, .z = 0.5, .w = -0.5}},
        // Gimbal lock, both poles. Absent until now, which meant the whole
        // degenerate branch of to_rpy -- the one place the scalar core and the
        // batched torch kernel could silently disagree -- was unpinned.
        {"gimbal_lock_up", quat::from_rpy(0.3, kPi / 2, 1.2)},
        {"gimbal_lock_down", quat::from_rpy(0.3, -kPi / 2, 1.2)},
        // Deliberately no *near*-pole case here, though that is where the
        // singularity test earns its keep (CYB-3869). 1e-7 rad off vertical the
        // roll lives in the last few bits of 1 - 2(y^2 + z^2), which float32
        // does not have -- so a golden case would pin a number the batched
        // kernel cannot produce at that dtype, and the cross-language file is
        // the wrong place to encode a per-precision exception. It is pinned per
        // language instead: rpy_keeps_the_roll_of_an_attitude_merely_near_the_pole
        // in test_quaternion.cpp, and its float64 counterparts in the binding
        // and torch tests.
        {"unnormalized", Quaternion{.x = 4.0, .y = 6.0, .z = 8.0, .w = 2.0}},
        {"degenerate", Quaternion{.x = 0.0, .y = 0.0, .z = 0.0, .w = 0.0}},
    };
}

void add_quaternion_cases(Json& cases, const Json& robots)
{
    for (const NamedQuaternion& sample : sample_quaternions())
    {
        const std::string suffix = std::string(".") + sample.name;
        add(cases, robots, "quat.normalize" + suffix, "quat_normalize", quaternion_input("q", sample.value));
        add(cases, robots, "quat.conjugate" + suffix, "quat_conjugate", quaternion_input("q", sample.value));
        add(cases, robots, "quat.inverse" + suffix, "quat_inverse", quaternion_input("q", sample.value));
        add(cases, robots, "quat.to_axis_angle" + suffix, "quat_to_axis_angle", quaternion_input("q", sample.value));
        add(cases, robots, "quat.to_rpy" + suffix, "quat_to_rpy", quaternion_input("q", sample.value));
        add(cases, robots, "quat.to_yaw" + suffix, "quat_to_yaw", quaternion_input("q", sample.value));
        add(cases, robots, "quat.to_matrix" + suffix, "quat_to_matrix", quaternion_input("q", sample.value));

        Json rotate = quaternion_input("q", sample.value);
        rotate.set("v", to_json(Vector3{1.0, -2.0, 3.5}));
        add(cases, robots, "quat.rotate" + suffix, "quat_rotate", rotate);
    }

    // Multiplication is where a JPL-vs-Hamilton mix-up shows up, so every
    // ordered pair of a few distinct rotations is pinned.
    const std::vector<NamedQuaternion> factors = {
        {"identity", Quaternion::identity()},
        {"general", quat::from_rpy(0.4, -0.5, 0.6)},
        {"yaw90", quat::from_yaw(kPi / 2)},
        {"half_turn_x", quat::from_axis_angle(Vector3{1, 0, 0}, kPi).value()},
    };
    for (const NamedQuaternion& left : factors)
    {
        for (const NamedQuaternion& right : factors)
        {
            Json input = Json::object();
            input.set("a", to_json(left.value));
            input.set("b", to_json(right.value));
            add(cases, robots, std::string("quat.multiply.") + left.name + "_x_" + right.name, "quat_multiply", input);
        }
    }

    // Conversions in the constructing direction.
    const double rpy_samples[][3] = {
        {0.0, 0.0, 0.0},      {0.3, -0.4, 1.2},    {kPi, 0.0, 0.0},  {0.0, kPi / 2, 0.0},
        {0.0, -kPi / 2, 0.0}, {0.3, kPi / 2, 1.2}, {-2.5, 0.9, 3.0},
    };
    int index = 0;
    for (const auto& rpy : rpy_samples)
    {
        Json input = Json::object();
        input.set("roll", Json::number(rpy[0]));
        input.set("pitch", Json::number(rpy[1]));
        input.set("yaw", Json::number(rpy[2]));
        add(cases, robots, "quat.from_rpy." + std::to_string(index), "quat_from_rpy", input);
        ++index;
    }

    const double yaw_samples[] = {0.0, 0.75, -2.5, kPi, 2.0 * kPi};
    index = 0;
    for (double yaw : yaw_samples)
    {
        Json input = Json::object();
        input.set("yaw", Json::number(yaw));
        add(cases, robots, "quat.from_yaw." + std::to_string(index), "quat_from_yaw", input);
        ++index;
    }

    struct AxisAngleSample
    {
        const char* name;
        Vector3 axis;
        double angle;
    };
    const std::vector<AxisAngleSample> axis_samples = {
        {"z_quarter", Vector3{0, 0, 1}, kPi / 2},   {"unnormalized_axis", Vector3{0, 0, 5}, kPi / 2},
        {"diagonal", Vector3{1, 2, -2}, 1.1},       {"negative_angle", Vector3{1, 0, 0}, -0.7},
        {"full_turn", Vector3{0, 1, 0}, 2.0 * kPi}, {"degenerate_axis", Vector3{0, 0, 0}, 1.0},
    };
    for (const AxisAngleSample& sample : axis_samples)
    {
        Json input = Json::object();
        input.set("axis", to_json(sample.axis));
        input.set("angle", Json::number(sample.angle));
        add(cases, robots, std::string("quat.from_axis_angle.") + sample.name, "quat_from_axis_angle", input);
    }

    // from_matrix: one per Shepperd branch, plus the two rejections.
    const std::vector<NamedQuaternion> matrix_samples = {
        {"general", quat::from_rpy(0.1, 0.2, 0.3)},
        {"half_turn_x", quat::from_axis_angle(Vector3{1, 0, 0}, kPi).value()},
        {"half_turn_y", quat::from_axis_angle(Vector3{0, 1, 0}, kPi).value()},
        {"half_turn_z", quat::from_axis_angle(Vector3{0, 0, 1}, kPi).value()},
    };
    for (const NamedQuaternion& sample : matrix_samples)
    {
        // Materialized into a named local first: iterating `result.value().m`
        // directly walks a member of a temporary whose lifetime the range-for
        // does not extend before C++23, which silently yields freed memory.
        const Matrix3 rotation = quat::to_matrix(sample.value).value();
        Json matrix = Json::array();
        for (double component : rotation.m)
        {
            matrix.push_back(Json::number(component));
        }
        Json input = Json::object();
        input.set("matrix", matrix);
        add(cases, robots, std::string("quat.from_matrix.") + sample.name, "quat_from_matrix", input);
    }
    {
        const Matrix3 rotation = quat::to_matrix(quat::from_rpy(0.1, 0.2, 0.3)).value();
        Json scaled = Json::array();
        for (double component : rotation.m)
        {
            scaled.push_back(Json::number(component * 2.0));
        }
        Json input = Json::object();
        input.set("matrix", scaled);
        add(cases, robots, "quat.from_matrix.scaled_is_rejected", "quat_from_matrix", input);

        Matrix3 mirrored = rotation;
        mirrored.at(0, 0) = -mirrored.at(0, 0);
        mirrored.at(1, 0) = -mirrored.at(1, 0);
        mirrored.at(2, 0) = -mirrored.at(2, 0);
        Json mirrored_json = Json::array();
        for (double component : mirrored.m)
        {
            mirrored_json.push_back(Json::number(component));
        }
        Json mirrored_input = Json::object();
        mirrored_input.set("matrix", mirrored_json);
        add(cases, robots, "quat.from_matrix.mirrored_is_rejected", "quat_from_matrix", mirrored_input);
    }
}

void add_interpolation_cases(Json& cases, const Json& robots)
{
    struct Pair
    {
        const char* name;
        Quaternion a;
        Quaternion b;
    };
    const Quaternion near_parallel = quat::from_yaw(1e-4);
    const Quaternion far_apart = quat::from_rpy(-1.2, 0.9, -2.0);
    const Quaternion quarter = quat::from_yaw(kPi / 2);
    // Stored as an antipode: the shortest-arc flip is what stops this taking
    // the long way round, and it is the bug most slerp implementations have.
    const Quaternion antipodal{.x = -quarter.x, .y = -quarter.y, .z = -quarter.z, .w = -quarter.w};

    const std::vector<Pair> pairs = {
        {"identity_to_yaw", Quaternion::identity(), quat::from_yaw(1.0)},
        {"general", quat::from_rpy(0.1, 0.2, 0.3), far_apart},
        {"antipodal", Quaternion::identity(), antipodal},
        {"near_parallel", Quaternion::identity(), near_parallel},
        {"identical", far_apart, far_apart},
        {"degenerate_b", Quaternion::identity(), Quaternion{.x = 0.0, .y = 0.0, .z = 0.0, .w = 0.0}},
    };
    const double parameters[] = {0.0, 0.25, 0.5, 0.75, 1.0, -0.5, 1.5};

    for (const Pair& pair : pairs)
    {
        for (double t : parameters)
        {
            Json input = Json::object();
            input.set("a", to_json(pair.a));
            input.set("b", to_json(pair.b));
            input.set("t", Json::number(t));
            char label[32];
            std::snprintf(label, sizeof(label), "%g", t);
            add(cases, robots, std::string("quat.slerp.") + pair.name + "@" + label, "quat_slerp", input);
            add(cases, robots, std::string("quat.nlerp.") + pair.name + "@" + label, "quat_nlerp", input);
        }
    }
}

void add_transform_cases(Json& cases, const Json& robots)
{
    const Transform a{Vector3{1.0, 2.0, 3.0}, quat::from_rpy(0.2, -0.3, 0.4)};
    const Transform b{Vector3{-0.5, 0.25, 2.0}, quat::from_rpy(-0.1, 0.7, 0.15)};
    const Transform identity = Transform::identity();
    const Transform unnormalized{Vector3{1, 2, 3}, Quaternion{.x = 0, .y = 0, .z = 0, .w = 2}};
    const Transform degenerate{Vector3{1, 2, 3}, Quaternion{.x = 0, .y = 0, .z = 0, .w = 0}};

    struct Pair
    {
        const char* name;
        Transform parent;
        Transform child;
    };
    const std::vector<Pair> pairs = {
        {"a_b", a, b},
        {"b_a", b, a},
        {"a_identity", a, identity},
        {"identity_a", identity, a},
    };
    for (const Pair& pair : pairs)
    {
        Json input = Json::object();
        input.set("parent", to_json(pair.parent));
        input.set("child", to_json(pair.child));
        add(cases, robots, std::string("transform.compose.") + pair.name, "transform_compose", input);
        add(cases, robots, std::string("transform.relative.") + pair.name, "transform_relative", input);
    }

    struct Single
    {
        const char* name;
        Transform value;
    };
    const std::vector<Single> singles = {
        {"a", a}, {"b", b}, {"identity", identity}, {"unnormalized", unnormalized}, {"degenerate", degenerate},
    };
    for (const Single& single : singles)
    {
        Json input = Json::object();
        input.set("t", to_json(single.value));
        add(cases, robots, std::string("transform.inverse.") + single.name, "transform_inverse", input);
        add(cases, robots, std::string("transform.normalize.") + single.name, "transform_normalize", input);

        Json apply_input = Json::object();
        apply_input.set("t", to_json(single.value));
        apply_input.set("point", to_json(Vector3{0.7, -1.4, 2.1}));
        add(cases, robots, std::string("transform.apply.") + single.name, "transform_apply", apply_input);
    }
}

void add_fk_cases(Json& cases, const Json& robots)
{
    for (const auto& entry : robots.entries())
    {
        Json input = Json::object();
        input.set("robot", Json::string(entry.first));
        add(cases, robots, "fk.tree." + entry.first, "fk_tree", input);
    }

    struct FrameCase
    {
        const char* id;
        const char* robot;
        const char* frame;
        std::vector<std::pair<std::string, double>> positions;
        bool with_base;
    };
    const Transform base{Vector3{10.0, -3.0, 0.5}, quat::from_rpy(0.05, -0.1, 2.2)};

    const std::vector<FrameCase> frame_cases = {
        {"two_link_arm.base", "two_link_arm", "base_link", {}, false},
        {"two_link_arm.base_with_base_transform", "two_link_arm", "base_link", {}, true},
        {"two_link_arm.upper_arm", "two_link_arm", "upper_arm", {{"shoulder", 0.9}}, false},
        {"two_link_arm.forearm_zero", "two_link_arm", "forearm", {{"shoulder", 0.0}, {"elbow", 0.0}}, false},
        {"two_link_arm.forearm_quarter_turn",
         "two_link_arm",
         "forearm",
         {{"shoulder", kPi / 2}, {"elbow", 0.0}},
         false},
        {"two_link_arm.forearm_general", "two_link_arm", "forearm", {{"shoulder", 0.3}, {"elbow", -0.7}}, false},
        {"two_link_arm.forearm_with_base", "two_link_arm", "forearm", {{"shoulder", 0.3}, {"elbow", -0.7}}, true},
        {"two_link_arm.wrist_camera", "two_link_arm", "wrist_camera", {{"shoulder", 0.3}, {"elbow", -0.7}}, true},
        {"two_link_arm.missing_elbow", "two_link_arm", "forearm", {{"shoulder", 0.3}}, false},
        {"two_link_arm.missing_all", "two_link_arm", "forearm", {}, false},
        {"two_link_arm.unknown_frame", "two_link_arm", "nope", {}, false},
        // Rotated joint origins, so origin-before-motion is observable.
        {"tilted_slider.tilted", "tilted_slider", "tilted", {{"tilt", 0.6}}, false},
        {"tilted_slider.slider", "tilted_slider", "slider", {{"tilt", 0.6}, {"extend", 0.25}}, false},
        {"tilted_slider.slider_with_base", "tilted_slider", "slider", {{"tilt", -1.2}, {"extend", -0.4}}, true},
        {"tilted_slider.base_link", "tilted_slider", "base_link", {}, false},
        // Mimic joints.
        {"mimic_gripper.left", "mimic_gripper", "left_finger", {{"left_finger_joint", 0.02}}, false},
        {"mimic_gripper.right", "mimic_gripper", "right_finger", {{"left_finger_joint", 0.02}}, false},
        {"mimic_gripper.right_needs_source", "mimic_gripper", "right_finger", {{"right_finger_joint", 0.02}}, false},
        // Failure modes.
        {"broken_tree.a", "broken_tree", "a", {}, false},
        {"broken_tree.b", "broken_tree", "b", {}, false},
        {"floating_joint.floater", "floating_joint", "floater", {}, false},
    };

    for (const FrameCase& frame_case : frame_cases)
    {
        Json input = Json::object();
        input.set("robot", Json::string(frame_case.robot));
        input.set("frame", Json::string(frame_case.frame));
        Json positions = Json::object();
        for (const auto& position : frame_case.positions)
        {
            positions.set(position.first, Json::number(position.second));
        }
        input.set("joint_positions", positions);
        if (frame_case.with_base)
        {
            input.set("base", to_json(base));
        }
        add(cases, robots, std::string("fk.pose.") + frame_case.id, "fk_frame_pose", input);
    }

    const char* type_samples[] = {"fixed",     "revolute", "  REVOLUTE ", "Continuous",
                                  "prismatic", "floating", "planar",      ""};
    for (const char* text : type_samples)
    {
        Json input = Json::object();
        input.set("text", Json::string(text));
        add(cases, robots, std::string("fk.joint_type.'") + text + "'", "joint_type_from_string", input);
    }

    // --- geodetic ----------------------------------------------------------
    //
    // The cases that make CONVENTIONS.md section 9 executable. Two anchors: one
    // whose heading is zero (environment == ENU) and one turned 30 degrees, so a
    // binding that drops the heading rotation entirely still fails.

    struct AnchorSample
    {
        const char* name;
        GeoAnchor anchor;
    };
    const AnchorSample anchor_samples[] = {
        {"enu", GeoAnchor{Geodetic{47.3769, 8.5417, 408.0}, 0.0}},
        {"turned", GeoAnchor{Geodetic{47.3769, 8.5417, 408.0}, 30.0}},
        // Southern hemisphere and negative longitude, because a sign dropped in
        // one quadrant is invisible in the other three.
        {"south", GeoAnchor{Geodetic{-33.8688, 151.2093, 58.0}, 215.0}},
        // Near the antimeridian, where a raw longitude subtraction reports most
        // of the way round the planet.
        {"antimeridian", GeoAnchor{Geodetic{-16.5, 179.999, 3.0}, 0.0}},
    };

    const Geodetic position_samples[] = {
        Geodetic{47.3801, 8.5500, 421.0},
        Geodetic{-33.8700, 151.2050, 51.5},
        Geodetic{-16.5, -179.999, 12.0},
        Geodetic{0.0, 0.0, 0.0},
    };

    for (const AnchorSample& anchor_sample : anchor_samples)
    {
        Json anchor_json = to_json(anchor_sample.anchor.origin);
        anchor_json.set("heading_deg", Json::number(anchor_sample.anchor.heading_deg));

        for (std::size_t index = 0; index < sizeof(position_samples) / sizeof(position_samples[0]); ++index)
        {
            Json input = Json::object();
            input.set("anchor", anchor_json);
            input.set("position", to_json(position_samples[index]));
            add(cases, robots, std::string("geo.to_local.") + anchor_sample.name + "." + std::to_string(index),
                "geo_to_local", input);

            Json enu_input = Json::object();
            enu_input.set("origin", to_json(anchor_sample.anchor.origin));
            enu_input.set("target", to_json(position_samples[index]));
            add(cases, robots, std::string("geo.enu_between.") + anchor_sample.name + "." + std::to_string(index),
                "geo_enu_between", enu_input);

            Json distance_input = Json::object();
            distance_input.set("a", to_json(anchor_sample.anchor.origin));
            distance_input.set("b", to_json(position_samples[index]));
            add(cases, robots, std::string("geo.ground_distance.") + anchor_sample.name + "." + std::to_string(index),
                "geo_ground_distance", distance_input);
        }

        const Vector3 local_samples[] = {Vector3{0.0, 0.0, 0.0}, Vector3{412.5, -118.25, 7.5},
                                         Vector3{-2500.0, 3100.0, -40.0}};
        for (std::size_t index = 0; index < sizeof(local_samples) / sizeof(local_samples[0]); ++index)
        {
            Json input = Json::object();
            input.set("anchor", anchor_json);
            input.set("position", to_json(local_samples[index]));
            add(cases, robots, std::string("geo.from_local.") + anchor_sample.name + "." + std::to_string(index),
                "geo_from_local", input);

            Json offset_input = Json::object();
            offset_input.set("origin", to_json(anchor_sample.anchor.origin));
            offset_input.set("enu", to_json(local_samples[index]));
            add(cases, robots, std::string("geo.offset.") + anchor_sample.name + "." + std::to_string(index),
                "geo_offset", offset_input);
        }

        // The 6-DOF pair, which is the whole point of the type: position and
        // orientation converted together.
        const Quaternion orientation_samples[] = {Quaternion::identity(), quat::from_rpy(0.12, -0.34, 1.05),
                                                  quat::from_yaw(-2.6)};
        for (std::size_t index = 0; index < sizeof(orientation_samples) / sizeof(orientation_samples[0]); ++index)
        {
            Json pose = to_json(position_samples[0]);
            pose.set("rotation", to_json(orientation_samples[index]));
            Json input = Json::object();
            input.set("anchor", anchor_json);
            input.set("pose", pose);
            add(cases, robots, std::string("geo.to_local_pose.") + anchor_sample.name + "." + std::to_string(index),
                "geo_to_local_pose", input);

            Json back = Json::object();
            back.set("anchor", anchor_json);
            Transform local{Vector3{321.0, -87.5, 14.25}, orientation_samples[index]};
            back.set("pose", to_json(local));
            add(cases, robots, std::string("geo.from_local_pose.") + anchor_sample.name + "." + std::to_string(index),
                "geo_from_local_pose", back);
        }
    }

    // Strictness: an out-of-range coordinate and a polar anchor are errors, not
    // clamps. A binding that quietly clamps passes every case above.
    {
        Json input = Json::object();
        Json anchor_json = to_json(Geodetic{47.3769, 8.5417, 408.0});
        anchor_json.set("heading_deg", Json::number(0.0));
        input.set("anchor", anchor_json);
        input.set("position", to_json(Geodetic{91.0, 0.0, 0.0}));
        add(cases, robots, "geo.to_local.latitude_off_the_ellipsoid", "geo_to_local", input);

        Json polar = to_json(Geodetic{89.95, 0.0, 0.0});
        polar.set("heading_deg", Json::number(0.0));
        Json polar_input = Json::object();
        polar_input.set("anchor", polar);
        polar_input.set("position", to_json(Geodetic{89.95, 0.0, 0.0}));
        add(cases, robots, "geo.to_local.polar_anchor", "geo_to_local", polar_input);

        Json degenerate_pose = to_json(Geodetic{47.3769, 8.5417, 408.0});
        degenerate_pose.set("rotation", to_json(Quaternion{.x = 0.0, .y = 0.0, .z = 0.0, .w = 0.0}));
        Json degenerate = Json::object();
        degenerate.set("anchor", anchor_json);
        degenerate.set("pose", degenerate_pose);
        add(cases, robots, "geo.to_local_pose.degenerate_orientation", "geo_to_local_pose", degenerate);
    }

    for (const double latitude : {0.0, 45.0, -33.8688, 78.0, 89.9, -89.9})
    {
        Json input = Json::object();
        input.set("latitude_deg", Json::number(latitude));
        add(cases, robots, "geo.metres_per_degree." + to_id(latitude), "geo_metres_per_degree", input);
    }

    for (const double longitude : {0.0, 179.999, -179.999, 180.0, 181.0, -180.0, -181.0, 540.0})
    {
        Json input = Json::object();
        input.set("longitude_deg", Json::number(longitude));
        add(cases, robots, "geo.normalize_longitude." + to_id(longitude), "geo_normalize_longitude_deg", input);
    }

    // Bearings. The cardinal headings are what separates a clockwise-from-north
    // reading from a counter-clockwise-from-east one; 45 and 225 agree under
    // both, so they are deliberately not the only samples.
    for (const double heading : {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 359.5, 450.0, -30.0})
    {
        Json input = Json::object();
        input.set("heading_deg", Json::number(heading));
        add(cases, robots, "geo.quat_from_compass_heading." + to_id(heading), "geo_quat_from_compass_heading", input);

        const auto q = geo::quat_from_compass_heading(heading);
        if (q)
        {
            add(cases, robots, "geo.to_compass_heading_deg." + to_id(heading), "geo_to_compass_heading_deg",
                quaternion_input("q", q.value()));
        }
    }
    add(cases, robots, "geo.to_compass_heading_deg.identity", "geo_to_compass_heading_deg",
        quaternion_input("q", Quaternion::identity()));
    add(cases, robots, "geo.to_compass_heading_deg.degenerate", "geo_to_compass_heading_deg",
        quaternion_input("q", Quaternion{.x = 0.0, .y = 0.0, .z = 0.0, .w = 0.0}));

    // NED, the other quarantined convention. The pitch samples matter most: NED
    // positive is nose-up, ENU/FLU positive is nose-down.
    struct NedSample
    {
        double roll;
        double pitch;
        double yaw;
    };
    const NedSample ned_samples[] = {
        {0.0, 0.0, 0.0}, {0.0, 0.0, kPi / 2}, {0.0, 0.0, kPi},     {0.0, 0.0, -kPi / 2},
        {0.0, 0.5, 0.0}, {0.0, -0.5, 0.0},    {0.21, -0.44, 2.05}, {-1.2, 0.9, -3.0},
    };
    for (std::size_t index = 0; index < sizeof(ned_samples) / sizeof(ned_samples[0]); ++index)
    {
        Json input = Json::object();
        input.set("roll", Json::number(ned_samples[index].roll));
        input.set("pitch", Json::number(ned_samples[index].pitch));
        input.set("yaw", Json::number(ned_samples[index].yaw));
        add(cases, robots, "geo.quat_from_ned_rpy." + std::to_string(index), "geo_quat_from_ned_rpy", input);

        const auto q =
            geo::quat_from_ned_rpy(ned_samples[index].roll, ned_samples[index].pitch, ned_samples[index].yaw);
        if (q)
        {
            add(cases, robots, "geo.to_ned_rpy." + std::to_string(index), "geo_to_ned_rpy",
                quaternion_input("q", q.value()));
        }
    }
}

/// Recursive comparison with a numeric tolerance. Reports the first difference
/// under each path rather than only "not equal".
bool compare(const Json& expected, const Json& actual, const std::string& path, std::string* report)
{
    if (expected.type() != actual.type())
    {
        *report += "      " + path + ": type mismatch\n";
        return false;
    }
    switch (expected.type())
    {
        case Json::Type::kNull:
            return true;
        case Json::Type::kBool:
            if (expected.as_bool() != actual.as_bool())
            {
                *report += "      " + path + ": expected " + (expected.as_bool() ? "true" : "false") + ", got " +
                           (actual.as_bool() ? "true" : "false") + "\n";
                return false;
            }
            return true;
        case Json::Type::kNumber:
        {
            const double difference = std::abs(expected.as_number() - actual.as_number());
            if (!(difference <= golden_budget(expected.as_number())))
            {
                char buffer[160];
                std::snprintf(buffer, sizeof(buffer), "      %s: expected %.17g, got %.17g (|d|=%.3g)\n", path.c_str(),
                              expected.as_number(), actual.as_number(), difference);
                *report += buffer;
                return false;
            }
            return true;
        }
        case Json::Type::kString:
            if (expected.as_string() != actual.as_string())
            {
                *report += "      " + path + ": expected \"" + expected.as_string() + "\", got \"" +
                           actual.as_string() + "\"\n";
                return false;
            }
            return true;
        case Json::Type::kArray:
        {
            if (expected.size() != actual.size())
            {
                *report += "      " + path + ": expected " + std::to_string(expected.size()) + " items, got " +
                           std::to_string(actual.size()) + "\n";
                return false;
            }
            bool equal = true;
            for (std::size_t index = 0; index < expected.size(); ++index)
            {
                equal &=
                    compare(expected.at(index), actual.at(index), path + "[" + std::to_string(index) + "]", report);
            }
            return equal;
        }
        case Json::Type::kObject:
        {
            bool equal = true;
            for (const auto& entry : expected.entries())
            {
                if (!actual.has(entry.first))
                {
                    *report += "      " + path + "." + entry.first + ": missing\n";
                    equal = false;
                    continue;
                }
                equal &= compare(entry.second, actual.get(entry.first), path + "." + entry.first, report);
            }
            for (const auto& entry : actual.entries())
            {
                if (!expected.has(entry.first))
                {
                    *report += "      " + path + "." + entry.first + ": unexpected\n";
                    equal = false;
                }
            }
            return equal;
        }
    }
    return true;
}

} // namespace

Json build_document()
{
    const Json robots = build_robots();

    Json document = Json::object();
    document.set("$schema_note", Json::string("Golden vectors for the Cyberwave geometry core. Quaternions are "
                                              "objects with named x/y/z/w; vectors with named x/y/z; matrices are "
                                              "row-major 9-element arrays. Regenerate with `golden_gen --write`."));
    document.set("version", Json::string(version_string()));
    document.set("tolerance", Json::number(kTolerance));
    document.set("robots", robots);

    Json cases = Json::array();
    add_quaternion_cases(cases, robots);
    add_interpolation_cases(cases, robots);
    add_transform_cases(cases, robots);
    add_fk_cases(cases, robots);
    document.set("cases", cases);
    return document;
}

int verify_document(const Json& document, std::string* report)
{
    const Json& robots = document.get("robots");
    const Json& cases = document.get("cases");
    int mismatches = 0;
    for (std::size_t index = 0; index < cases.size(); ++index)
    {
        const Json& entry = cases.at(index);
        const std::string id = entry.get("id").as_string();
        const Json recomputed = evaluate(robots, entry.get("op").as_string(), entry.get("input"));
        std::string detail;
        if (!compare(entry.get("expect"), recomputed, "expect", &detail))
        {
            ++mismatches;
            *report += "  MISMATCH " + id + "\n" + detail;
        }
    }
    return mismatches;
}

} // namespace golden
