#include "cyberwave/geometry/errors.hpp"

namespace cyberwave
{
namespace geometry
{

const char* error_code_name(ErrorCode code)
{
    switch (code)
    {
        case ErrorCode::kOk:
            return "ok";
        case ErrorCode::kInvalidQuaternion:
            return "invalid_quaternion";
        case ErrorCode::kInvalidAxis:
            return "invalid_axis";
        case ErrorCode::kNonFiniteValue:
            return "non_finite_value";
        case ErrorCode::kInvalidRotationMatrix:
            return "invalid_rotation_matrix";
        case ErrorCode::kInvalidGeodetic:
            return "invalid_geodetic";
        case ErrorCode::kInvalidGeoAnchor:
            return "invalid_geo_anchor";
        case ErrorCode::kNoLinks:
            return "no_links";
        case ErrorCode::kNoRoot:
            return "no_root";
        case ErrorCode::kMultipleRoots:
            return "multiple_roots";
        case ErrorCode::kAmbiguousParent:
            return "ambiguous_parent";
        case ErrorCode::kCycle:
            return "cycle";
        case ErrorCode::kInvalidJointPose:
            return "invalid_joint_pose";
        case ErrorCode::kInvalidSensorPose:
            return "invalid_sensor_pose";
        case ErrorCode::kUnsupportedJointType:
            return "unsupported_joint_type";
        case ErrorCode::kMissingJoint:
            return "missing_joint";
        case ErrorCode::kUnknownFrame:
            return "unknown_frame";
    }
    return "unknown";
}

} // namespace geometry
} // namespace cyberwave
