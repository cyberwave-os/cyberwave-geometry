// Structured errors and the Result type the strict core returns.
//
// The core never falls back to identity, never guesses a default and never
// throws: an operation that cannot be carried out returns an Error naming the
// subject it failed on. Compatibility adapters at parsing boundaries are the
// layer allowed to turn an Error back into a lenient default, and they have to
// do it visibly.
#ifndef CYBERWAVE_GEOMETRY_ERRORS_HPP
#define CYBERWAVE_GEOMETRY_ERRORS_HPP

#include <cstdint>
#include <string>
#include <utility>

namespace cyberwave
{
namespace geometry
{

/// Stable numeric codes: these cross the C ABI and are mirrored by every
/// binding, so values are append-only and never reused.
enum class ErrorCode : std::int32_t
{
    kOk = 0,

    // --- value-level ---
    /// Quaternion is non-finite or its norm is below kMinQuaternionNorm.
    kInvalidQuaternion = 1,
    /// Axis is non-finite or its norm is below kMinAxisNorm.
    kInvalidAxis = 2,
    /// An input component was NaN or infinite.
    kNonFiniteValue = 3,
    /// A matrix offered as a rotation is not right-handed orthonormal.
    kInvalidRotationMatrix = 4,
    /// A geodetic coordinate is non-finite, or outside [-90, 90] / [-180, 180].
    kInvalidGeodetic = 5,
    /// A geo anchor is unusable: bad origin, non-finite heading, or an origin so
    /// close to a pole that the tangent plane's longitude scale degenerates.
    kInvalidGeoAnchor = 6,

    // --- tree-level ---
    /// The description declares no links at all.
    kNoLinks = 10,
    /// Every link has a parent joint, so there is no topological root.
    kNoRoot = 11,
    /// The description is disconnected: several links have no parent joint.
    kMultipleRoots = 12,
    /// Two joints claim the same child link, so the chain to it is ambiguous.
    kAmbiguousParent = 13,
    /// A cycle was walked while resolving a chain.
    kCycle = 14,
    /// A joint origin quaternion could not be normalized.
    kInvalidJointPose = 15,
    /// A sensor extrinsic quaternion could not be normalized.
    kInvalidSensorPose = 16,
    /// The joint's motion is not a single scalar this engine can compose.
    kUnsupportedJointType = 17,
    /// A joint the chain depends on had no reported position.
    kMissingJoint = 18,
    /// The requested frame is neither a link nor a sensor.
    kUnknownFrame = 19,
};

const char* error_code_name(ErrorCode code);

struct Error
{
    ErrorCode code = ErrorCode::kOk;
    /// The joint, link, sensor or frame the failure is about. Empty when the
    /// failure is about the description as a whole.
    std::string subject;
    std::string detail;

    Error() = default;
    Error(ErrorCode code_, std::string subject_, std::string detail_)
        : code(code_), subject(std::move(subject_)), detail(std::move(detail_))
    {
    }
};

/// A value or an Error. Deliberately not std::expected, which is C++23 and so
/// out of reach of the C++20 baseline this compiles at, and not std::variant,
/// which would put a wrong-alternative access in the API's happy path.
template <typename T>
class Result
{
public:
    Result(T value) : value_(std::move(value)), ok_(true) {}
    Result(Error error) : error_(std::move(error)), ok_(false) {}

    static Result failure(ErrorCode code, std::string subject, std::string detail)
    {
        return Result(Error(code, std::move(subject), std::move(detail)));
    }

    bool ok() const { return ok_; }
    explicit operator bool() const { return ok_; }

    /// Only valid when ok(). Callers that skip the check read a default value
    /// rather than trip undefined behaviour, but that is a bug, not a feature.
    const T& value() const { return value_; }
    T& value() { return value_; }

    /// Only meaningful when !ok().
    const Error& error() const { return error_; }

    /// The lenient escape hatch, spelled out at the call site so a reviewer can
    /// see which boundaries are allowed to swallow an error.
    T value_or(T fallback) const { return ok_ ? value_ : fallback; }

private:
    T value_{};
    Error error_;
    bool ok_ = false;
};

} // namespace geometry
} // namespace cyberwave

#endif // CYBERWAVE_GEOMETRY_ERRORS_HPP
