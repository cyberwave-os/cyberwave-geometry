// Emscripten/embind surface for the shared geometry core.
//
// Values cross as plain JS objects with named fields -- `{w, x, y, z}` for a
// quaternion, `{x, y, z}` for a vector -- matching the golden vectors and the
// Python binding, so a TypeScript conformance runner reads the same file the
// C++ and Python ones do.
//
// Errors cross as thrown JS objects `{code, subject, detail}` rather than as
// a status field, because that is what a `try`/`catch` in the frontend
// expects and it keeps the strict/lenient split visible at the call site.
//
// Verified by bindings/wasm/tests/run_golden.mjs: all 420 golden cases at 1e-12.
#include "cyberwave/geometry/geometry.hpp"

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <string>
#include <vector>

using namespace cyberwave::geometry;
using emscripten::val;

namespace
{

[[noreturn]] void throw_error(const Error& error)
{
    // A real `Error`, not a plain object: the frontend gets `instanceof Error`,
    // a stack, and a message that reads in a console, plus the structured fields
    // next to it. Constructed here rather than decorated afterwards because
    // `Error.captureStackTrace` is V8-only and does nothing in Safari.
    const std::string code = error_code_name(error.code);
    std::string message = "cyberwave-geometry: " + code;
    if (!error.subject.empty())
    {
        message += " [" + error.subject + "]";
    }
    if (!error.detail.empty())
    {
        message += ": " + error.detail;
    }

    val payload = val::global("Error").new_(message);
    payload.set("name", std::string("CyberwaveGeometryError"));
    payload.set("code", code);
    payload.set("codeValue", static_cast<int>(error.code));
    payload.set("subject", error.subject);
    payload.set("detail", error.detail);

    // `throw payload` does NOT work: a C++-thrown `val` surfaces in JS as the raw
    // exception pointer (a number), so every `catch (e) { e.code }` reads
    // undefined. `val::throw_()` is the supported way to raise a JS value.
    payload.throw_();
}

template <typename T>
const T& unwrap(const Result<T>& result)
{
    if (!result)
    {
        throw_error(result.error());
    }
    return result.value();
}

val to_js(const Vector3& v)
{
    val out = val::object();
    out.set("x", v.x);
    out.set("y", v.y);
    out.set("z", v.z);
    return out;
}

val to_js(const Quaternion& q)
{
    // x, y, z, w -- the core's order. Read by name either way, but JS preserves
    // insertion order, so a serialized rotation should not depend on which
    // binding produced it.
    val out = val::object();
    out.set("x", q.x);
    out.set("y", q.y);
    out.set("z", q.z);
    out.set("w", q.w);
    return out;
}

val to_js(const Geodetic& g)
{
    // The `twin/{uuid}/gps` spelling, so the browser reads a fix with no rename.
    val out = val::object();
    out.set("latitude", g.latitude_deg);
    out.set("longitude", g.longitude_deg);
    out.set("altitude", g.altitude_m);
    return out;
}

val to_js(const Transform& t)
{
    val out = val::object();
    out.set("translation", to_js(t.translation));
    out.set("rotation", to_js(t.rotation));
    return out;
}

double number_at(const val& node, const char* key, double fallback)
{
    const val value = node[key];
    return value.isUndefined() || value.isNull() ? fallback : value.as<double>();
}

Vector3 vector_from(const val& node)
{
    return Vector3{number_at(node, "x", 0.0), number_at(node, "y", 0.0), number_at(node, "z", 0.0)};
}

Quaternion quaternion_from(const val& node)
{
    return Quaternion{.x = number_at(node, "x", 0.0),
                      .y = number_at(node, "y", 0.0),
                      .z = number_at(node, "z", 0.0),
                      .w = number_at(node, "w", 1.0)};
}

Geodetic geodetic_from(const val& node)
{
    return Geodetic{number_at(node, "latitude", 0.0), number_at(node, "longitude", 0.0),
                    number_at(node, "altitude", 0.0)};
}

GeoAnchor anchor_from(const val& node) { return GeoAnchor{geodetic_from(node), number_at(node, "heading_deg", 0.0)}; }

Transform transform_from(const val& node)
{
    if (node.isUndefined() || node.isNull())
    {
        return Transform::identity();
    }
    return Transform{vector_from(node["translation"]), quaternion_from(node["rotation"])};
}

// --- geodetic ---------------------------------------------------------------
//
// See CONVENTIONS.md section 9. Positions are degrees and metres; an orientation
// maps the body frame into the local ENU frame at its own position.

val geo_validate(val position) { return to_js(unwrap(geo::validate(geodetic_from(position)))); }
val geo_validate_anchor(val anchor)
{
    const GeoAnchor value = unwrap(geo::validate(anchor_from(anchor)));
    val out = to_js(value.origin);
    out.set("heading_deg", value.heading_deg);
    return out;
}
bool geo_is_valid(val position) { return geo::validate(geodetic_from(position)).ok(); }
bool geo_is_anchor_valid(val anchor) { return geo::validate(anchor_from(anchor)).ok(); }

val geo_metres_per_degree(double latitude_deg)
{
    const MetresPerDegree scale = unwrap(geo::metres_per_degree(latitude_deg));
    val out = val::object();
    out.set("latitude", scale.latitude);
    out.set("longitude", scale.longitude);
    return out;
}
double geo_normalize_longitude_deg(double longitude_deg) { return geo::normalize_longitude_deg(longitude_deg); }
double geo_normalize_bearing_deg(double bearing_deg) { return geo::normalize_bearing_deg(bearing_deg); }
val geo_enu_between(val origin, val target)
{
    return to_js(unwrap(geo::enu_between(geodetic_from(origin), geodetic_from(target))));
}
val geo_offset(val origin, val enu) { return to_js(unwrap(geo::offset(geodetic_from(origin), vector_from(enu)))); }
double geo_ground_distance(val a, val b) { return unwrap(geo::ground_distance(geodetic_from(a), geodetic_from(b))); }
val geo_enu_to_local_rotation(val anchor) { return to_js(unwrap(geo::enu_to_local_rotation(anchor_from(anchor)))); }
val geo_to_local(val anchor, val position)
{
    return to_js(unwrap(geo::to_local(anchor_from(anchor), geodetic_from(position))));
}
val geo_from_local(val anchor, val position)
{
    return to_js(unwrap(geo::from_local(anchor_from(anchor), vector_from(position))));
}
val geo_to_local_pose(val anchor, val pose)
{
    const GeoPose value{geodetic_from(pose), quaternion_from(pose["rotation"])};
    return to_js(unwrap(geo::to_local_pose(anchor_from(anchor), value)));
}
val geo_from_local_pose(val anchor, val pose)
{
    const GeoPose result = unwrap(geo::from_local_pose(anchor_from(anchor), transform_from(pose)));
    val out = to_js(result.position);
    out.set("rotation", to_js(result.orientation));
    return out;
}
val geo_quat_from_compass_heading(double heading_deg)
{
    return to_js(unwrap(geo::quat_from_compass_heading(heading_deg)));
}
double geo_to_compass_heading_deg(val q) { return unwrap(geo::to_compass_heading_deg(quaternion_from(q))); }
val geo_quat_from_ned_rpy(double roll, double pitch, double yaw)
{
    return to_js(unwrap(geo::quat_from_ned_rpy(roll, pitch, yaw)));
}
val geo_to_ned_rpy(val q)
{
    const Rpy result = unwrap(geo::to_ned_rpy(quaternion_from(q)));
    val out = val::object();
    out.set("roll", result.roll);
    out.set("pitch", result.pitch);
    out.set("yaw", result.yaw);
    return out;
}

// --- quaternion -------------------------------------------------------------

val quat_normalize(val q) { return to_js(unwrap(quat::normalize(quaternion_from(q)))); }
val quat_multiply(val a, val b) { return to_js(quat::multiply(quaternion_from(a), quaternion_from(b))); }
val quat_conjugate(val q) { return to_js(quat::conjugate(quaternion_from(q))); }
val quat_inverse(val q) { return to_js(unwrap(quat::inverse(quaternion_from(q)))); }
val quat_rotate(val q, val v) { return to_js(unwrap(quat::rotate(quaternion_from(q), vector_from(v)))); }
val quat_rotate_unit(val q, val v) { return to_js(quat::rotate_unit(quaternion_from(q), vector_from(v))); }
val quat_slerp(val a, val b, double t) { return to_js(unwrap(quat::slerp(quaternion_from(a), quaternion_from(b), t))); }
val quat_nlerp(val a, val b, double t) { return to_js(unwrap(quat::nlerp(quaternion_from(a), quaternion_from(b), t))); }
val quat_from_axis_angle(val axis, double angle)
{
    return to_js(unwrap(quat::from_axis_angle(vector_from(axis), angle)));
}
val quat_to_axis_angle(val q)
{
    const AxisAngle result = unwrap(quat::to_axis_angle(quaternion_from(q)));
    val out = val::object();
    out.set("axis", to_js(result.axis));
    out.set("angle", result.angle);
    return out;
}
val quat_from_rpy(double roll, double pitch, double yaw) { return to_js(quat::from_rpy(roll, pitch, yaw)); }
val quat_to_rpy(val q)
{
    const Rpy result = unwrap(quat::to_rpy(quaternion_from(q)));
    val out = val::object();
    out.set("roll", result.roll);
    out.set("pitch", result.pitch);
    out.set("yaw", result.yaw);
    return out;
}
val quat_from_yaw(double yaw) { return to_js(quat::from_yaw(yaw)); }
double quat_to_yaw(val q) { return unwrap(quat::to_yaw(quaternion_from(q))); }
double quat_norm(val q) { return quat::norm(quaternion_from(q)); }
double quat_dot(val a, val b) { return quat::dot(quaternion_from(a), quaternion_from(b)); }

val quat_to_matrix(val q)
{
    const Matrix3 matrix = unwrap(quat::to_matrix(quaternion_from(q)));
    val out = val::array();
    for (std::size_t index = 0; index < matrix.m.size(); ++index)
    {
        out.set(static_cast<int>(index), matrix.m[index]);
    }
    return out;
}

val quat_from_matrix(val components, double tolerance)
{
    Matrix3 matrix;
    const int length = components["length"].as<int>();
    if (length != 9)
    {
        throw_error(Error(ErrorCode::kInvalidRotationMatrix, "", "expected 9 matrix components"));
    }
    for (int index = 0; index < 9; ++index)
    {
        matrix.m[static_cast<std::size_t>(index)] = components[index].as<double>();
    }
    return to_js(unwrap(quat::from_matrix(matrix, tolerance)));
}

// --- transform --------------------------------------------------------------

val transform_normalize(val t) { return to_js(unwrap(tf::normalize(transform_from(t)))); }
val transform_compose(val parent, val child)
{
    return to_js(tf::compose(transform_from(parent), transform_from(child)));
}
val transform_inverse(val t) { return to_js(unwrap(tf::inverse(transform_from(t)))); }
val transform_inverse_unit(val t) { return to_js(tf::inverse_unit(transform_from(t))); }
val transform_apply(val t, val point) { return to_js(tf::apply(transform_from(t), vector_from(point))); }
val transform_relative(val parent, val child)
{
    return to_js(tf::relative(transform_from(parent), transform_from(child)));
}

// --- forward kinematics -----------------------------------------------------

std::string joint_type_from_string(std::string text) { return fk::joint_type_name(fk::joint_type_from_string(text)); }

/// Owns a built tree. JS must call `.delete()` on it -- embind does not garbage
/// collect C++ objects, and a leaked tree is a leaked WASM heap allocation.
class RobotTreeHandle
{
public:
    explicit RobotTreeHandle(val description) { tree_ = fk::RobotTree::build(parse(description)); }

    std::string root_link() const { return tree_.root_link(); }
    std::string base_frame() const { return tree_.base_frame(); }
    bool is_usable() const { return tree_.is_usable(); }
    bool is_articulated() const { return tree_.is_articulated(); }

    val frames() const
    {
        val out = val::array();
        const auto names = tree_.frame_names();
        for (std::size_t index = 0; index < names.size(); ++index)
        {
            out.set(static_cast<int>(index), names[index]);
        }
        return out;
    }

    val errors() const
    {
        val out = val::array();
        const auto& collected = tree_.errors();
        for (std::size_t index = 0; index < collected.size(); ++index)
        {
            val entry = val::object();
            entry.set("code", std::string(error_code_name(collected[index].code)));
            entry.set("subject", collected[index].subject);
            entry.set("detail", collected[index].detail);
            out.set(static_cast<int>(index), entry);
        }
        return out;
    }

    /// Throws for a structural problem; returns `{available, transform?,
    /// missingJoints?}` otherwise -- the same three outcomes as every other
    /// binding.
    val frame_pose(std::string frame_name, val base, val joint_positions) const
    {
        fk::JointPositions positions;
        if (!joint_positions.isUndefined() && !joint_positions.isNull())
        {
            const val keys = val::global("Object").call<val>("keys", joint_positions);
            const int count = keys["length"].as<int>();
            for (int index = 0; index < count; ++index)
            {
                const std::string name = keys[index].as<std::string>();
                positions[name] = joint_positions[name].as<double>();
            }
        }

        const auto result = fk::compute_frame_pose(tree_, frame_name, transform_from(base), positions);
        if (!result)
        {
            throw_error(result.error());
        }
        val out = val::object();
        out.set("available", result.value().available);
        if (result.value().available)
        {
            out.set("transform", to_js(result.value().transform));
        }
        else
        {
            val missing = val::array();
            for (std::size_t index = 0; index < result.value().missing_joints.size(); ++index)
            {
                missing.set(static_cast<int>(index), result.value().missing_joints[index]);
            }
            out.set("missingJoints", missing);
        }
        return out;
    }

private:
    static fk::RobotDescription parse(const val& description)
    {
        fk::RobotDescription out;

        const val links = description["links"];
        const int link_count = links.isUndefined() ? 0 : links["length"].as<int>();
        for (int index = 0; index < link_count; ++index)
        {
            out.links.push_back(links[index].as<std::string>());
        }

        const val joints = description["joints"];
        const int joint_count = joints.isUndefined() ? 0 : joints["length"].as<int>();
        for (int index = 0; index < joint_count; ++index)
        {
            const val raw = joints[index];
            fk::JointDescription joint;
            joint.name = raw["name"].as<std::string>();
            joint.parent_link = raw["parent_link"].as<std::string>();
            joint.child_link = raw["child_link"].as<std::string>();
            joint.type = fk::joint_type_from_string(raw["type"].as<std::string>());
            joint.origin = transform_from(raw["origin"]);
            joint.axis = vector_from(raw["axis"]);
            const val mimic = raw["mimic"];
            if (!mimic.isUndefined() && !mimic.isNull())
            {
                joint.has_mimic = true;
                joint.mimic.source_joint = mimic["source_joint"].as<std::string>();
                joint.mimic.multiplier = number_at(mimic, "multiplier", 1.0);
                joint.mimic.offset = number_at(mimic, "offset", 0.0);
            }
            out.joints.push_back(std::move(joint));
        }

        const val sensors = description["sensors"];
        const int sensor_count = sensors.isUndefined() ? 0 : sensors["length"].as<int>();
        for (int index = 0; index < sensor_count; ++index)
        {
            const val raw = sensors[index];
            fk::SensorDescription sensor;
            sensor.name = raw["name"].as<std::string>();
            sensor.parent_link = raw["parent_link"].as<std::string>();
            const val extrinsic = raw["extrinsic"];
            sensor.extrinsic = transform_from(extrinsic.isUndefined() ? raw["pose"] : extrinsic);
            out.sensors.push_back(std::move(sensor));
        }
        return out;
    }

    fk::RobotTree tree_;
};

std::string core_version() { return version_string(); }

} // namespace

EMSCRIPTEN_BINDINGS(cyberwave_geometry)
{
    emscripten::function("coreVersion", &core_version);

    emscripten::function("quatNormalize", &quat_normalize);
    emscripten::function("quatMultiply", &quat_multiply);
    emscripten::function("quatConjugate", &quat_conjugate);
    emscripten::function("quatInverse", &quat_inverse);
    emscripten::function("quatRotate", &quat_rotate);
    emscripten::function("quatRotateUnit", &quat_rotate_unit);
    emscripten::function("quatSlerp", &quat_slerp);
    emscripten::function("quatNlerp", &quat_nlerp);
    emscripten::function("quatNorm", &quat_norm);
    emscripten::function("quatDot", &quat_dot);
    emscripten::function("quatFromAxisAngle", &quat_from_axis_angle);
    emscripten::function("quatToAxisAngle", &quat_to_axis_angle);
    emscripten::function("quatFromRpy", &quat_from_rpy);
    emscripten::function("quatToRpy", &quat_to_rpy);
    emscripten::function("quatFromYaw", &quat_from_yaw);
    emscripten::function("quatToYaw", &quat_to_yaw);
    emscripten::function("quatToMatrix", &quat_to_matrix);
    emscripten::function("quatFromMatrix", &quat_from_matrix);

    emscripten::function("transformNormalize", &transform_normalize);
    emscripten::function("transformCompose", &transform_compose);
    emscripten::function("transformInverse", &transform_inverse);
    emscripten::function("transformInverseUnit", &transform_inverse_unit);
    emscripten::function("transformApply", &transform_apply);
    emscripten::function("transformRelative", &transform_relative);

    emscripten::function("geoValidate", &geo_validate);
    emscripten::function("geoValidateAnchor", &geo_validate_anchor);
    emscripten::function("geoIsValid", &geo_is_valid);
    emscripten::function("geoIsAnchorValid", &geo_is_anchor_valid);
    emscripten::function("geoMetresPerDegree", &geo_metres_per_degree);
    emscripten::function("geoNormalizeLongitudeDeg", &geo_normalize_longitude_deg);
    emscripten::function("geoNormalizeBearingDeg", &geo_normalize_bearing_deg);
    emscripten::function("geoEnuBetween", &geo_enu_between);
    emscripten::function("geoOffset", &geo_offset);
    emscripten::function("geoGroundDistance", &geo_ground_distance);
    emscripten::function("geoEnuToLocalRotation", &geo_enu_to_local_rotation);
    emscripten::function("geoToLocal", &geo_to_local);
    emscripten::function("geoFromLocal", &geo_from_local);
    emscripten::function("geoToLocalPose", &geo_to_local_pose);
    emscripten::function("geoFromLocalPose", &geo_from_local_pose);
    emscripten::function("geoQuatFromCompassHeading", &geo_quat_from_compass_heading);
    emscripten::function("geoToCompassHeadingDeg", &geo_to_compass_heading_deg);
    emscripten::function("geoQuatFromNedRpy", &geo_quat_from_ned_rpy);
    emscripten::function("geoToNedRpy", &geo_to_ned_rpy);

    emscripten::function("jointTypeFromString", &joint_type_from_string);

    emscripten::class_<RobotTreeHandle>("RobotTree")
        .constructor<val>()
        .function("rootLink", &RobotTreeHandle::root_link)
        .function("baseFrame", &RobotTreeHandle::base_frame)
        .function("isUsable", &RobotTreeHandle::is_usable)
        .function("isArticulated", &RobotTreeHandle::is_articulated)
        .function("frames", &RobotTreeHandle::frames)
        .function("errors", &RobotTreeHandle::errors)
        .function("framePose", &RobotTreeHandle::frame_pose);
}
