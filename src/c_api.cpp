// Marshalling layer for the stable C ABI. No geometry logic lives here: every
// entry point converts, delegates to the C++ core and converts back.
#include "cyberwave/geometry/c_api.h"

#include "cyberwave/geometry/geometry.hpp"

#include <new>
#include <string>
#include <vector>

namespace
{

using cyberwave::geometry::ErrorCode;
using cyberwave::geometry::GeoAnchor;
using cyberwave::geometry::Geodetic;
using cyberwave::geometry::GeoPose;
using cyberwave::geometry::Matrix3;
using cyberwave::geometry::Quaternion;
using cyberwave::geometry::Transform;
using cyberwave::geometry::Vector3;
namespace fk = cyberwave::geometry::fk;
namespace geo = cyberwave::geometry::geo;
namespace quat = cyberwave::geometry::quat;
namespace tf = cyberwave::geometry::tf;

Vector3 to_cpp(cw_geom_vec3 v) { return Vector3{v.x, v.y, v.z}; }
cw_geom_vec3 to_c(const Vector3& v) { return cw_geom_vec3{v.x, v.y, v.z}; }
Quaternion to_cpp(cw_geom_quat q) { return Quaternion{.x = q.x, .y = q.y, .z = q.z, .w = q.w}; }
cw_geom_quat to_c(const Quaternion& q) { return cw_geom_quat{.x = q.x, .y = q.y, .z = q.z, .w = q.w}; }

Transform to_cpp(cw_geom_transform t) { return Transform{to_cpp(t.translation), to_cpp(t.rotation)}; }

cw_geom_transform to_c(const Transform& t) { return cw_geom_transform{to_c(t.translation), to_c(t.rotation)}; }

Geodetic to_cpp(cw_geom_geodetic g) { return Geodetic{g.latitude_deg, g.longitude_deg, g.altitude_m}; }

cw_geom_geodetic to_c(const Geodetic& g)
{
    return cw_geom_geodetic{
        .latitude_deg = g.latitude_deg, .longitude_deg = g.longitude_deg, .altitude_m = g.altitude_m};
}

GeoAnchor to_cpp(cw_geom_geo_anchor a) { return GeoAnchor{to_cpp(a.origin), a.heading_deg}; }

GeoPose to_cpp(cw_geom_geo_pose p) { return GeoPose{to_cpp(p.position), to_cpp(p.orientation)}; }

cw_geom_geo_pose to_c(const GeoPose& p) { return cw_geom_geo_pose{to_c(p.position), to_c(p.orientation)}; }

cw_geom_status to_status(ErrorCode code) { return static_cast<cw_geom_status>(code); }

/// Empty rather than NULL for an absent string: a caller that forgets to check
/// gets a harmless empty string instead of a segfault.
const char* kEmpty = "";

} // namespace

struct cw_geom_builder
{
    fk::RobotDescription description;
};

struct cw_geom_tree
{
    fk::RobotTree tree;
    // frame_names() rebuilds a vector each call; the ABI hands out pointers
    // into storage the handle owns, so it is materialized once here.
    std::vector<std::string> frames;
};

struct cw_geom_pose
{
    cw_geom_status status = CW_GEOM_OK;
    std::string error_subject;
    std::string error_detail;
    bool available = false;
    Transform transform = Transform::identity();
    std::vector<std::string> missing;
};

extern "C"
{

    const char* cw_geom_version(void) { return cyberwave::geometry::version_string(); }

    void cw_geom_version_parts(int32_t* major, int32_t* minor, int32_t* patch)
    {
        if (major != nullptr)
        {
            *major = cyberwave::geometry::kVersionMajor;
        }
        if (minor != nullptr)
        {
            *minor = cyberwave::geometry::kVersionMinor;
        }
        if (patch != nullptr)
        {
            *patch = cyberwave::geometry::kVersionPatch;
        }
    }

    const char* cw_geom_error_name(cw_geom_status status)
    {
        return cyberwave::geometry::error_code_name(static_cast<ErrorCode>(status));
    }

    /* --- quaternion ---------------------------------------------------------- */

    cw_geom_quat cw_geom_quat_identity(void) { return to_c(Quaternion::identity()); }

    cw_geom_quat cw_geom_quat_from_wxyz(double w, double x, double y, double z)
    {
        return to_c(quat::from_wxyz(w, x, y, z));
    }

    cw_geom_quat cw_geom_quat_from_xyzw(double x, double y, double z, double w)
    {
        return to_c(quat::from_xyzw(x, y, z, w));
    }

    void cw_geom_quat_to_wxyz(cw_geom_quat q, double* out4)
    {
        if (out4 == nullptr)
        {
            return;
        }
        const auto components = quat::to_wxyz(to_cpp(q));
        for (int index = 0; index < 4; ++index)
        {
            out4[index] = components[static_cast<std::size_t>(index)];
        }
    }

    void cw_geom_quat_to_xyzw(cw_geom_quat q, double* out4)
    {
        if (out4 == nullptr)
        {
            return;
        }
        const auto components = quat::to_xyzw(to_cpp(q));
        for (int index = 0; index < 4; ++index)
        {
            out4[index] = components[static_cast<std::size_t>(index)];
        }
    }

    double cw_geom_quat_norm(cw_geom_quat q) { return quat::norm(to_cpp(q)); }

    double cw_geom_quat_dot(cw_geom_quat a, cw_geom_quat b) { return quat::dot(to_cpp(a), to_cpp(b)); }

    cw_geom_status cw_geom_quat_normalize(cw_geom_quat q, cw_geom_quat* out)
    {
        const auto result = quat::normalize(to_cpp(q));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_quat cw_geom_quat_multiply(cw_geom_quat a, cw_geom_quat b)
    {
        return to_c(quat::multiply(to_cpp(a), to_cpp(b)));
    }

    cw_geom_quat cw_geom_quat_conjugate(cw_geom_quat q) { return to_c(quat::conjugate(to_cpp(q))); }

    cw_geom_status cw_geom_quat_inverse(cw_geom_quat q, cw_geom_quat* out)
    {
        const auto result = quat::inverse(to_cpp(q));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_vec3 cw_geom_quat_rotate_unit(cw_geom_quat q, cw_geom_vec3 v)
    {
        return to_c(quat::rotate_unit(to_cpp(q), to_cpp(v)));
    }

    cw_geom_status cw_geom_quat_rotate(cw_geom_quat q, cw_geom_vec3 v, cw_geom_vec3* out)
    {
        const auto result = quat::rotate(to_cpp(q), to_cpp(v));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_quat_from_axis_angle(cw_geom_vec3 axis, double angle, cw_geom_quat* out)
    {
        const auto result = quat::from_axis_angle(to_cpp(axis), angle);
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_quat_to_axis_angle(cw_geom_quat q, cw_geom_vec3* out_axis, double* out_angle)
    {
        const auto result = quat::to_axis_angle(to_cpp(q));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out_axis != nullptr)
        {
            *out_axis = to_c(result.value().axis);
        }
        if (out_angle != nullptr)
        {
            *out_angle = result.value().angle;
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_quat_slerp(cw_geom_quat a, cw_geom_quat b, double t, cw_geom_quat* out)
    {
        const auto result = quat::slerp(to_cpp(a), to_cpp(b), t);
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_quat_nlerp(cw_geom_quat a, cw_geom_quat b, double t, cw_geom_quat* out)
    {
        const auto result = quat::nlerp(to_cpp(a), to_cpp(b), t);
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_quat cw_geom_quat_from_rpy(double roll, double pitch, double yaw)
    {
        return to_c(quat::from_rpy(roll, pitch, yaw));
    }

    cw_geom_status cw_geom_quat_to_rpy(cw_geom_quat q, double* out_roll, double* out_pitch, double* out_yaw)
    {
        const auto result = quat::to_rpy(to_cpp(q));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out_roll != nullptr)
        {
            *out_roll = result.value().roll;
        }
        if (out_pitch != nullptr)
        {
            *out_pitch = result.value().pitch;
        }
        if (out_yaw != nullptr)
        {
            *out_yaw = result.value().yaw;
        }
        return CW_GEOM_OK;
    }

    cw_geom_quat cw_geom_quat_from_yaw(double yaw) { return to_c(quat::from_yaw(yaw)); }

    cw_geom_status cw_geom_quat_to_yaw(cw_geom_quat q, double* out)
    {
        const auto result = quat::to_yaw(to_cpp(q));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = result.value();
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_quat_to_matrix(cw_geom_quat q, double* out9)
    {
        const auto result = quat::to_matrix(to_cpp(q));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out9 != nullptr)
        {
            for (std::size_t index = 0; index < 9; ++index)
            {
                out9[index] = result.value().m[index];
            }
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_quat_from_matrix(const double* m9, double tolerance, cw_geom_quat* out)
    {
        if (m9 == nullptr)
        {
            return CW_GEOM_ERR_INVALID_ROTATION_MATRIX;
        }
        Matrix3 matrix;
        for (std::size_t index = 0; index < 9; ++index)
        {
            matrix.m[index] = m9[index];
        }
        const auto result = quat::from_matrix(matrix, tolerance);
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    /* --- transform ----------------------------------------------------------- */

    cw_geom_transform cw_geom_transform_identity(void) { return to_c(Transform::identity()); }

    cw_geom_status cw_geom_transform_normalize(cw_geom_transform t, cw_geom_transform* out)
    {
        const auto result = tf::normalize(to_cpp(t));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_transform cw_geom_transform_compose(cw_geom_transform parent, cw_geom_transform child)
    {
        return to_c(tf::compose(to_cpp(parent), to_cpp(child)));
    }

    cw_geom_transform cw_geom_transform_inverse_unit(cw_geom_transform t) { return to_c(tf::inverse_unit(to_cpp(t))); }

    cw_geom_status cw_geom_transform_inverse(cw_geom_transform t, cw_geom_transform* out)
    {
        const auto result = tf::inverse(to_cpp(t));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_vec3 cw_geom_transform_apply(cw_geom_transform t, cw_geom_vec3 point)
    {
        return to_c(tf::apply(to_cpp(t), to_cpp(point)));
    }

    cw_geom_transform cw_geom_transform_relative(cw_geom_transform parent, cw_geom_transform child)
    {
        return to_c(tf::relative(to_cpp(parent), to_cpp(child)));
    }

    /* --- geodetic ------------------------------------------------------------ */

    cw_geom_status cw_geom_geo_validate(cw_geom_geodetic position)
    {
        return to_status(geo::validate(to_cpp(position)).error().code);
    }

    cw_geom_status cw_geom_geo_validate_anchor(cw_geom_geo_anchor anchor)
    {
        return to_status(geo::validate(to_cpp(anchor)).error().code);
    }

    double cw_geom_geo_normalize_longitude_deg(double longitude_deg)
    {
        return geo::normalize_longitude_deg(longitude_deg);
    }

    double cw_geom_geo_normalize_bearing_deg(double bearing_deg) { return geo::normalize_bearing_deg(bearing_deg); }

    cw_geom_status cw_geom_geo_metres_per_degree(double latitude_deg, double* out_latitude, double* out_longitude)
    {
        const auto result = geo::metres_per_degree(latitude_deg);
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out_latitude != nullptr)
        {
            *out_latitude = result.value().latitude;
        }
        if (out_longitude != nullptr)
        {
            *out_longitude = result.value().longitude;
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_enu_between(cw_geom_geodetic origin, cw_geom_geodetic target, cw_geom_vec3* out)
    {
        const auto result = geo::enu_between(to_cpp(origin), to_cpp(target));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_offset(cw_geom_geodetic origin, cw_geom_vec3 enu, cw_geom_geodetic* out)
    {
        const auto result = geo::offset(to_cpp(origin), to_cpp(enu));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_ground_distance(cw_geom_geodetic a, cw_geom_geodetic b, double* out)
    {
        const auto result = geo::ground_distance(to_cpp(a), to_cpp(b));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = result.value();
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_enu_to_local_rotation(cw_geom_geo_anchor anchor, cw_geom_quat* out)
    {
        const auto result = geo::enu_to_local_rotation(to_cpp(anchor));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_to_local(cw_geom_geo_anchor anchor, cw_geom_geodetic position, cw_geom_vec3* out)
    {
        const auto result = geo::to_local(to_cpp(anchor), to_cpp(position));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_from_local(cw_geom_geo_anchor anchor, cw_geom_vec3 position, cw_geom_geodetic* out)
    {
        const auto result = geo::from_local(to_cpp(anchor), to_cpp(position));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_to_local_pose(cw_geom_geo_anchor anchor, cw_geom_geo_pose pose, cw_geom_transform* out)
    {
        const auto result = geo::to_local_pose(to_cpp(anchor), to_cpp(pose));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_from_local_pose(cw_geom_geo_anchor anchor, cw_geom_transform pose, cw_geom_geo_pose* out)
    {
        const auto result = geo::from_local_pose(to_cpp(anchor), to_cpp(pose));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_quat_from_compass_heading(double heading_deg, cw_geom_quat* out)
    {
        const auto result = geo::quat_from_compass_heading(heading_deg);
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_to_compass_heading_deg(cw_geom_quat enu_orientation, double* out)
    {
        const auto result = geo::to_compass_heading_deg(to_cpp(enu_orientation));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = result.value();
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_quat_from_ned_rpy(double roll, double pitch, double yaw, cw_geom_quat* out)
    {
        const auto result = geo::quat_from_ned_rpy(roll, pitch, yaw);
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out != nullptr)
        {
            *out = to_c(result.value());
        }
        return CW_GEOM_OK;
    }

    cw_geom_status cw_geom_geo_to_ned_rpy(cw_geom_quat enu_orientation, double* out_roll, double* out_pitch,
                                          double* out_yaw)
    {
        const auto result = geo::to_ned_rpy(to_cpp(enu_orientation));
        if (!result)
        {
            return to_status(result.error().code);
        }
        if (out_roll != nullptr)
        {
            *out_roll = result.value().roll;
        }
        if (out_pitch != nullptr)
        {
            *out_pitch = result.value().pitch;
        }
        if (out_yaw != nullptr)
        {
            *out_yaw = result.value().yaw;
        }
        return CW_GEOM_OK;
    }

    /* --- forward kinematics -------------------------------------------------- */

    int32_t cw_geom_joint_type_from_string(const char* text)
    {
        return static_cast<int32_t>(fk::joint_type_from_string(text == nullptr ? "" : text));
    }

    const char* cw_geom_joint_type_name(int32_t type) { return fk::joint_type_name(static_cast<fk::JointType>(type)); }

    cw_geom_builder* cw_geom_builder_create(void) { return new (std::nothrow) cw_geom_builder(); }

    void cw_geom_builder_destroy(cw_geom_builder* builder) { delete builder; }

    void cw_geom_builder_add_link(cw_geom_builder* builder, const char* name)
    {
        if (builder == nullptr || name == nullptr)
        {
            return;
        }
        builder->description.links.emplace_back(name);
    }

    void cw_geom_builder_add_joint(cw_geom_builder* builder, const char* name, const char* parent_link,
                                   const char* child_link, int32_t type, cw_geom_transform origin, cw_geom_vec3 axis,
                                   const char* mimic_source, double mimic_multiplier, double mimic_offset)
    {
        if (builder == nullptr)
        {
            return;
        }
        fk::JointDescription joint;
        joint.name = name == nullptr ? "" : name;
        joint.parent_link = parent_link == nullptr ? "" : parent_link;
        joint.child_link = child_link == nullptr ? "" : child_link;
        joint.type = static_cast<fk::JointType>(type);
        joint.origin = to_cpp(origin);
        joint.axis = to_cpp(axis);
        if (mimic_source != nullptr && mimic_source[0] != '\0')
        {
            joint.has_mimic = true;
            joint.mimic.source_joint = mimic_source;
            joint.mimic.multiplier = mimic_multiplier;
            joint.mimic.offset = mimic_offset;
        }
        builder->description.joints.push_back(std::move(joint));
    }

    void cw_geom_builder_add_sensor(cw_geom_builder* builder, const char* name, const char* parent_link,
                                    cw_geom_transform extrinsic)
    {
        if (builder == nullptr)
        {
            return;
        }
        fk::SensorDescription sensor;
        sensor.name = name == nullptr ? "" : name;
        sensor.parent_link = parent_link == nullptr ? "" : parent_link;
        sensor.extrinsic = to_cpp(extrinsic);
        builder->description.sensors.push_back(std::move(sensor));
    }

    cw_geom_tree* cw_geom_tree_build(const cw_geom_builder* builder)
    {
        if (builder == nullptr)
        {
            return nullptr;
        }
        cw_geom_tree* handle = new (std::nothrow) cw_geom_tree();
        if (handle == nullptr)
        {
            return nullptr;
        }
        handle->tree = fk::RobotTree::build(builder->description);
        handle->frames = handle->tree.frame_names();
        return handle;
    }

    void cw_geom_tree_destroy(cw_geom_tree* tree) { delete tree; }

    const char* cw_geom_tree_root_link(const cw_geom_tree* tree)
    {
        return tree == nullptr ? kEmpty : tree->tree.root_link().c_str();
    }

    const char* cw_geom_tree_base_frame(const cw_geom_tree* tree)
    {
        return tree == nullptr ? kEmpty : tree->tree.base_frame().c_str();
    }

    int32_t cw_geom_tree_is_usable(const cw_geom_tree* tree)
    {
        return (tree != nullptr && tree->tree.is_usable()) ? 1 : 0;
    }

    int32_t cw_geom_tree_is_articulated(const cw_geom_tree* tree)
    {
        return (tree != nullptr && tree->tree.is_articulated()) ? 1 : 0;
    }

    size_t cw_geom_tree_frame_count(const cw_geom_tree* tree) { return tree == nullptr ? 0 : tree->frames.size(); }

    const char* cw_geom_tree_frame_name(const cw_geom_tree* tree, size_t index)
    {
        if (tree == nullptr || index >= tree->frames.size())
        {
            return kEmpty;
        }
        return tree->frames[index].c_str();
    }

    size_t cw_geom_tree_error_count(const cw_geom_tree* tree)
    {
        return tree == nullptr ? 0 : tree->tree.errors().size();
    }

    cw_geom_status cw_geom_tree_error(const cw_geom_tree* tree, size_t index, const char** out_subject,
                                      const char** out_detail)
    {
        if (tree == nullptr || index >= tree->tree.errors().size())
        {
            return CW_GEOM_OK;
        }
        const auto& error = tree->tree.errors()[index];
        if (out_subject != nullptr)
        {
            *out_subject = error.subject.c_str();
        }
        if (out_detail != nullptr)
        {
            *out_detail = error.detail.c_str();
        }
        return to_status(error.code);
    }

    cw_geom_pose* cw_geom_compute_frame_pose(const cw_geom_tree* tree, const char* frame_name, cw_geom_transform base,
                                             const char* const* joint_names, const double* joint_values,
                                             size_t joint_count)
    {
        cw_geom_pose* handle = new (std::nothrow) cw_geom_pose();
        if (handle == nullptr)
        {
            return nullptr;
        }
        if (tree == nullptr)
        {
            handle->status = CW_GEOM_ERR_NO_ROOT;
            handle->error_detail = "tree handle is null";
            return handle;
        }

        fk::JointPositions positions;
        if (joint_names != nullptr && joint_values != nullptr)
        {
            for (size_t index = 0; index < joint_count; ++index)
            {
                if (joint_names[index] == nullptr)
                {
                    continue;
                }
                positions[joint_names[index]] = joint_values[index];
            }
        }

        const auto result =
            fk::compute_frame_pose(tree->tree, frame_name == nullptr ? "" : frame_name, to_cpp(base), positions);
        if (!result)
        {
            handle->status = to_status(result.error().code);
            handle->error_subject = result.error().subject;
            handle->error_detail = result.error().detail;
            return handle;
        }
        handle->available = result.value().available;
        handle->transform = result.value().transform;
        handle->missing = result.value().missing_joints;
        return handle;
    }

    void cw_geom_pose_destroy(cw_geom_pose* pose) { delete pose; }

    cw_geom_status cw_geom_pose_status(const cw_geom_pose* pose) { return pose == nullptr ? CW_GEOM_OK : pose->status; }

    const char* cw_geom_pose_error_subject(const cw_geom_pose* pose)
    {
        return pose == nullptr ? kEmpty : pose->error_subject.c_str();
    }

    const char* cw_geom_pose_error_detail(const cw_geom_pose* pose)
    {
        return pose == nullptr ? kEmpty : pose->error_detail.c_str();
    }

    int32_t cw_geom_pose_available(const cw_geom_pose* pose) { return (pose != nullptr && pose->available) ? 1 : 0; }

    cw_geom_transform cw_geom_pose_transform(const cw_geom_pose* pose)
    {
        return pose == nullptr ? to_c(Transform::identity()) : to_c(pose->transform);
    }

    size_t cw_geom_pose_missing_count(const cw_geom_pose* pose) { return pose == nullptr ? 0 : pose->missing.size(); }

    const char* cw_geom_pose_missing_joint(const cw_geom_pose* pose, size_t index)
    {
        if (pose == nullptr || index >= pose->missing.size())
        {
            return kEmpty;
        }
        return pose->missing[index].c_str();
    }

} // extern "C"
