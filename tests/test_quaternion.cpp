// Quaternion algebra, every conversion path, and the degenerate inputs the
// strict core is supposed to reject rather than paper over.
#include "test_support.hpp"

#include <cmath>

using namespace cyberwave::geometry;

namespace
{

constexpr double kPi = 3.14159265358979323846;

const Quaternion kSample = quat::from_rpy(0.4, -0.5, 0.6);

} // namespace

// --- component order --------------------------------------------------------

TEST(from_xyzw_and_from_wxyz_disagree_on_the_same_numbers)
{
    // The whole reason the named accessors exist: the same four numbers are two
    // different rotations depending on which order you read them in.
    const Quaternion as_wxyz = quat::from_wxyz(0.1, 0.2, 0.3, 0.4);
    const Quaternion as_xyzw = quat::from_xyzw(0.1, 0.2, 0.3, 0.4);
    CHECK(as_wxyz.w == 0.1);
    CHECK(as_xyzw.w == 0.4);
    CHECK(as_wxyz.x == 0.2);
    CHECK(as_xyzw.x == 0.1);
}

TEST(component_order_round_trips)
{
    const auto wxyz = quat::to_wxyz(kSample);
    const auto xyzw = quat::to_xyzw(kSample);
    CHECK_NEAR(quat::from_wxyz(wxyz), kSample, cwtest::kEps);
    CHECK_NEAR(quat::from_xyzw(xyzw), kSample, cwtest::kEps);
    CHECK(wxyz[0] == xyzw[3]);
    CHECK(wxyz[1] == xyzw[0]);
}

// --- algebra ----------------------------------------------------------------

TEST(multiply_is_hamilton_and_not_commutative)
{
    const Quaternion about_x = quat::from_axis_angle(Vector3{1, 0, 0}, kPi / 2).value();
    const Quaternion about_y = quat::from_axis_angle(Vector3{0, 1, 0}, kPi / 2).value();
    const Quaternion xy = quat::multiply(about_x, about_y);
    const Quaternion yx = quat::multiply(about_y, about_x);
    CHECK(!cwtest::same_rotation(xy, yx, 1e-9));

    // multiply(a, b) applies b first: rotating +Z by (x then y) lands on +X
    // going through the y-turn last.
    const Vector3 rotated = quat::rotate_unit(xy, Vector3{0, 0, 1});
    const Vector3 stepwise = quat::rotate_unit(about_x, quat::rotate_unit(about_y, Vector3{0, 0, 1}));
    CHECK_NEAR(rotated, stepwise, 1e-12);
}

TEST(multiply_by_identity_is_a_no_op)
{
    CHECK_NEAR(quat::multiply(kSample, Quaternion::identity()), kSample, cwtest::kEps);
    CHECK_NEAR(quat::multiply(Quaternion::identity(), kSample), kSample, cwtest::kEps);
}

TEST(normalize_scales_to_unit_norm)
{
    const Quaternion scaled{.x = 4.0, .y = 6.0, .z = 8.0, .w = 2.0};
    const auto normalized = quat::normalize(scaled);
    CHECK(normalized.ok());
    CHECK_NEAR(quat::norm(normalized.value()), 1.0, cwtest::kEps);
    // Same direction: the ratio between components is preserved.
    CHECK_NEAR(normalized.value().x / normalized.value().w, 2.0, 1e-12);
}

TEST(normalize_rejects_a_near_zero_quaternion)
{
    const auto result = quat::normalize(Quaternion{.x = 0.0, .y = 0.0, .z = 0.0, .w = 0.0});
    CHECK(!result.ok());
    CHECK(result.error().code == ErrorCode::kInvalidQuaternion);
}

TEST(normalize_rejects_non_finite_components)
{
    const double nan_value = std::nan("");
    CHECK((!quat::normalize(Quaternion{.x = 0, .y = 0, .z = 0, .w = nan_value}).ok()));
    CHECK((!quat::normalize(Quaternion{.x = INFINITY, .y = 0, .z = 0, .w = 1.0}).ok()));
}

TEST(inverse_undoes_a_rotation_even_when_unnormalized)
{
    const Quaternion scaled{.x = 2.0 * kSample.x, .y = 2.0 * kSample.y, .z = 2.0 * kSample.z, .w = 2.0 * kSample.w};
    const auto inverted = quat::inverse(scaled);
    CHECK(inverted.ok());
    const Quaternion product = quat::multiply(scaled, inverted.value());
    CHECK_NEAR(product, Quaternion::identity(), 1e-12);
}

TEST(conjugate_equals_inverse_only_for_unit_input)
{
    const auto inverted = quat::inverse(kSample);
    CHECK(inverted.ok());
    CHECK_NEAR(quat::conjugate(kSample), inverted.value(), 1e-12);

    const Quaternion scaled{.x = 0.0, .y = 0.0, .z = 0.0, .w = 2.0};
    CHECK(!cwtest::near(quat::conjugate(scaled), quat::inverse(scaled).value(), 1e-9));
}

TEST(rotate_matches_the_sandwich_product)
{
    const Vector3 v{1.0, -2.0, 3.5};
    const Quaternion q = kSample;
    // q * (0, v) * q^-1, spelled out, as the reference for the optimized form.
    const Quaternion pure{.x = v.x, .y = v.y, .z = v.z, .w = 0.0};
    const Quaternion sandwiched = quat::multiply(quat::multiply(q, pure), quat::conjugate(q));
    const Vector3 expected{sandwiched.x, sandwiched.y, sandwiched.z};
    CHECK_NEAR(quat::rotate_unit(q, v), expected, 1e-12);
}

TEST(rotate_preserves_length)
{
    const Vector3 v{1.0, -2.0, 3.5};
    const Vector3 rotated = quat::rotate_unit(kSample, v);
    CHECK_NEAR(vec::norm(rotated), vec::norm(v), 1e-12);
}

TEST(rotate_normalizes_its_input_and_rejects_a_degenerate_one)
{
    const Quaternion scaled{.x = 3.0 * kSample.x, .y = 3.0 * kSample.y, .z = 3.0 * kSample.z, .w = 3.0 * kSample.w};
    const Vector3 v{1.0, 0.0, 0.0};
    const auto rotated = quat::rotate(scaled, v);
    CHECK(rotated.ok());
    // Without the normalize, the sandwich would scale the vector by 9.
    CHECK_NEAR(vec::norm(rotated.value()), 1.0, 1e-12);
    CHECK((!quat::rotate(Quaternion{.x = 0, .y = 0, .z = 0, .w = 0}, v).ok()));
}

// --- axis-angle -------------------------------------------------------------

TEST(axis_angle_round_trips)
{
    const Vector3 axis{1.0, 2.0, -2.0}; // deliberately not unit
    const auto q = quat::from_axis_angle(axis, 1.1);
    CHECK(q.ok());
    const auto recovered = quat::to_axis_angle(q.value());
    CHECK(recovered.ok());
    CHECK_NEAR(recovered.value().angle, 1.1, 1e-12);
    CHECK_NEAR(recovered.value().axis, vec::normalize_axis(axis).value(), 1e-12);
}

TEST(from_axis_angle_normalizes_the_axis)
{
    const auto scaled = quat::from_axis_angle(Vector3{0, 0, 5}, 0.7);
    const auto unit = quat::from_axis_angle(Vector3{0, 0, 1}, 0.7);
    CHECK(scaled.ok() && unit.ok());
    CHECK_NEAR(scaled.value(), unit.value(), 1e-15);
}

TEST(from_axis_angle_rejects_a_degenerate_axis)
{
    const auto result = quat::from_axis_angle(Vector3{0, 0, 0}, 1.0);
    CHECK(!result.ok());
    CHECK(result.error().code == ErrorCode::kInvalidAxis);
}

TEST(from_axis_angle_rejects_a_non_finite_angle)
{
    CHECK((!quat::from_axis_angle(Vector3{0, 0, 1}, std::nan("")).ok()));
    CHECK((!quat::from_axis_angle(Vector3{0, 0, 1}, INFINITY).ok()));
}

TEST(to_axis_angle_of_identity_is_a_zero_turn_not_an_error)
{
    const auto result = quat::to_axis_angle(Quaternion::identity());
    CHECK(result.ok());
    CHECK_NEAR(result.value().angle, 0.0, cwtest::kEps);
    // The axis of a null rotation is arbitrary; +X keeps the round trip total.
    CHECK_NEAR(result.value().axis, (Vector3{1, 0, 0}), cwtest::kEps);
}

TEST(to_axis_angle_reports_the_short_way_round)
{
    // A rotation stored with a negative w is the same rotation as its negation;
    // the reported angle should stay within [0, pi] either way.
    const Quaternion q = quat::from_axis_angle(Vector3{0, 0, 1}, 1.5).value();
    const Quaternion negated{.x = -q.x, .y = -q.y, .z = -q.z, .w = -q.w};
    const auto from_negated = quat::to_axis_angle(negated);
    CHECK(from_negated.ok());
    CHECK(from_negated.value().angle >= 0.0 && from_negated.value().angle <= kPi + 1e-12);
    CHECK_NEAR(from_negated.value().angle, 1.5, 1e-12);
}

// --- roll/pitch/yaw ---------------------------------------------------------

TEST(from_rpy_matches_the_fixed_axis_xyz_composition)
{
    const double roll = 0.3;
    const double pitch = -0.4;
    const double yaw = 1.2;
    // Fixed-axis XYZ == intrinsic Z-Y-X == qz * qy * qx.
    const Quaternion composed = quat::multiply(quat::from_axis_angle(Vector3{0, 0, 1}, yaw).value(),
                                               quat::multiply(quat::from_axis_angle(Vector3{0, 1, 0}, pitch).value(),
                                                              quat::from_axis_angle(Vector3{1, 0, 0}, roll).value()));
    CHECK_NEAR(quat::from_rpy(roll, pitch, yaw), composed, 1e-14);
}

TEST(rpy_round_trips_away_from_gimbal_lock)
{
    const Rpy original{0.3, -0.4, 1.2};
    const auto recovered = quat::to_rpy(quat::from_rpy(original));
    CHECK(recovered.ok());
    CHECK_NEAR(recovered.value().roll, original.roll, 1e-12);
    CHECK_NEAR(recovered.value().pitch, original.pitch, 1e-12);
    CHECK_NEAR(recovered.value().yaw, original.yaw, 1e-12);
}

TEST(rpy_reconstructs_the_rotation_at_both_gimbal_lock_poles)
{
    for (double pitch : {kPi / 2, -kPi / 2})
    {
        const Quaternion original = quat::from_rpy(0.3, pitch, 1.2);
        const auto decomposed = quat::to_rpy(original);
        CHECK(decomposed.ok());
        // Roll and yaw are not separable here, so the angles will not match --
        // but they must still rebuild the same rotation.
        CHECK(cwtest::near(decomposed.value().roll, 0.0, 1e-9));
        const Quaternion rebuilt = quat::from_rpy(decomposed.value());
        CHECK_MSG(cwtest::same_rotation(rebuilt, original, 1e-9), "pitch=" + cwtest::to_string(pitch) + " rebuilt " +
                                                                      cwtest::to_string(rebuilt) + " vs " +
                                                                      cwtest::to_string(original));
    }
}

TEST(rpy_keeps_the_roll_of_an_attitude_merely_near_the_pole)
{
    // 1e-7 rad off vertical is not gimbal lock, but sin(pitch) there is
    // 1 - 5e-15 -- within three ulps of the pole's own value. Deciding the
    // singularity on sin threw this roll away and rebuilt a rotation ~1e-7
    // off; deciding it on cos(pitch) keeps it (CYB-3869).
    for (double pitch : {kPi / 2 - 1e-7, -kPi / 2 + 1e-7})
    {
        const Rpy original{0.7, pitch, -1.2};
        const Quaternion source = quat::from_rpy(original);
        const auto recovered = quat::to_rpy(source);
        CHECK(recovered.ok());
        CHECK_NEAR(recovered.value().roll, original.roll, 1e-8);
        CHECK_NEAR(recovered.value().pitch, original.pitch, 1e-8);
        CHECK_NEAR(recovered.value().yaw, original.yaw, 1e-8);
        CHECK_MSG(cwtest::same_rotation(quat::from_rpy(recovered.value()), source, 1e-9),
                  "pitch=" + cwtest::to_string(pitch));
        // to_yaw has to agree with to_rpy about where the pole starts, or the
        // two disagree by the whole of yaw across this band.
        CHECK_NEAR(quat::to_yaw(source).value(), recovered.value().yaw, 1e-12);
    }
}

TEST(to_rpy_clamps_rather_than_returning_nan_past_the_pole)
{
    // A quaternion whose sin(pitch) rounds a hair above 1 must not become NaN.
    // The clamp predates the move to atan2 and outlives it: atan2 would not
    // produce a NaN here, but it would report a pitch past the pole.
    const Quaternion straight_up = quat::from_rpy(0.0, kPi / 2, 0.0);
    const auto result = quat::to_rpy(straight_up);
    CHECK(result.ok());
    CHECK(!std::isnan(result.value().pitch));
    CHECK_NEAR(result.value().pitch, kPi / 2, 1e-7);
}

TEST(to_rpy_rejects_a_degenerate_quaternion)
{
    CHECK((!quat::to_rpy(Quaternion{.x = 0, .y = 0, .z = 0, .w = 0}).ok()));
}

// --- yaw --------------------------------------------------------------------

TEST(yaw_round_trips_through_from_yaw)
{
    for (double angle : {0.0, 0.75, -2.5, kPi})
    {
        const auto recovered = quat::to_yaw(quat::from_yaw(angle));
        CHECK(recovered.ok());
        CHECK_NEAR(recovered.value(), angle, 1e-12);
    }
}

TEST(from_yaw_agrees_with_from_rpy_and_from_axis_angle)
{
    const double angle = 0.9;
    CHECK_NEAR(quat::from_yaw(angle), quat::from_rpy(0.0, 0.0, angle), 1e-15);
    CHECK_NEAR(quat::from_yaw(angle), (quat::from_axis_angle(Vector3{0, 0, 1}, angle).value()), 1e-15);
}

TEST(yaw_normalizes_first)
{
    const Quaternion scaled{.x = 2.0 * kSample.x, .y = 2.0 * kSample.y, .z = 2.0 * kSample.z, .w = 2.0 * kSample.w};
    const auto from_scaled = quat::to_yaw(scaled);
    const auto from_unit = quat::to_yaw(kSample);
    CHECK(from_scaled.ok() && from_unit.ok());
    CHECK_NEAR(from_scaled.value(), from_unit.value(), 1e-12);
}

TEST(yaw_uses_the_stable_representation_at_gimbal_lock)
{
    const Quaternion up = quat::from_rpy(0.3, kPi / 2.0, 1.2);
    const Quaternion down = quat::from_rpy(0.3, -kPi / 2.0, 1.2);
    const auto up_yaw = quat::to_yaw(up);
    const auto down_yaw = quat::to_yaw(down);
    CHECK(up_yaw.ok() && down_yaw.ok());
    CHECK_NEAR(up_yaw.value(), 0.9, 1e-12);
    CHECK_NEAR(down_yaw.value(), 1.5, 1e-12);
}

// --- rotation matrix --------------------------------------------------------

TEST(matrix_round_trips)
{
    const auto matrix = quat::to_matrix(kSample);
    CHECK(matrix.ok());
    const auto recovered = quat::from_matrix(matrix.value());
    CHECK(recovered.ok());
    CHECK(cwtest::same_rotation(recovered.value(), kSample, 1e-12));
}

TEST(matrix_columns_are_the_rotated_basis_vectors)
{
    const auto matrix = quat::to_matrix(kSample);
    CHECK(matrix.ok());
    const Vector3 x_axis = quat::rotate_unit(kSample, Vector3{1, 0, 0});
    CHECK_NEAR((Vector3({matrix.value().at(0, 0), matrix.value().at(1, 0), matrix.value().at(2, 0)})), x_axis, 1e-12);
}

TEST(from_matrix_covers_every_shepperd_branch)
{
    // One rotation per branch: small angle (trace > 0), then 180 degrees about
    // each axis, where the trace is negative and a different diagonal wins.
    const Quaternion cases[] = {
        quat::from_rpy(0.1, 0.2, 0.3),
        quat::from_axis_angle(Vector3{1, 0, 0}, kPi).value(),
        quat::from_axis_angle(Vector3{0, 1, 0}, kPi).value(),
        quat::from_axis_angle(Vector3{0, 0, 1}, kPi).value(),
    };
    for (const Quaternion& original : cases)
    {
        const auto matrix = quat::to_matrix(original);
        CHECK(matrix.ok());
        const auto recovered = quat::from_matrix(matrix.value());
        CHECK(recovered.ok());
        CHECK_MSG(cwtest::same_rotation(recovered.value(), original, 1e-12),
                  cwtest::to_string(recovered.value()) + " vs " + cwtest::to_string(original));
    }
}

TEST(from_matrix_rejects_a_scaled_matrix)
{
    Matrix3 scaled = quat::to_matrix(kSample).value();
    for (double& component : scaled.m)
    {
        component *= 2.0;
    }
    const auto result = quat::from_matrix(scaled);
    CHECK(!result.ok());
    CHECK(result.error().code == ErrorCode::kInvalidRotationMatrix);
}

TEST(from_matrix_rejects_a_mirrored_matrix)
{
    // Flip one column: still orthonormal, but left-handed, so it is a
    // reflection and no quaternion represents it.
    Matrix3 mirrored = quat::to_matrix(kSample).value();
    mirrored.at(0, 0) = -mirrored.at(0, 0);
    mirrored.at(1, 0) = -mirrored.at(1, 0);
    mirrored.at(2, 0) = -mirrored.at(2, 0);
    const auto result = quat::from_matrix(mirrored);
    CHECK(!result.ok());
    CHECK(result.error().code == ErrorCode::kInvalidRotationMatrix);
}

TEST(from_matrix_rejects_non_finite_input)
{
    Matrix3 broken = Matrix3::identity();
    broken.at(1, 1) = std::nan("");
    const auto result = quat::from_matrix(broken);
    CHECK(!result.ok());
    CHECK(result.error().code == ErrorCode::kNonFiniteValue);
}

TEST(to_matrix_rejects_a_degenerate_quaternion)
{
    CHECK((!quat::to_matrix(Quaternion{.x = 0, .y = 0, .z = 0, .w = 0}).ok()));
}

// --- interpolation ----------------------------------------------------------

TEST(slerp_endpoints_are_the_inputs)
{
    const Quaternion a = quat::from_yaw(0.2);
    const Quaternion b = quat::from_yaw(1.4);
    CHECK(cwtest::same_rotation(quat::slerp(a, b, 0.0).value(), a, 1e-12));
    CHECK(cwtest::same_rotation(quat::slerp(a, b, 1.0).value(), b, 1e-12));
}

TEST(slerp_halfway_is_the_half_angle)
{
    const Quaternion a = quat::from_yaw(0.0);
    const Quaternion b = quat::from_yaw(1.0);
    const auto middle = quat::slerp(a, b, 0.5);
    CHECK(middle.ok());
    CHECK_NEAR(quat::to_yaw(middle.value()).value(), 0.5, 1e-12);
}

TEST(slerp_has_constant_angular_velocity)
{
    // The property that distinguishes slerp from nlerp: equal steps in t are
    // equal steps in angle.
    const Quaternion a = quat::from_yaw(0.0);
    const Quaternion b = quat::from_yaw(2.0);
    for (int step = 0; step <= 10; ++step)
    {
        const double t = step / 10.0;
        const auto sample = quat::slerp(a, b, t);
        CHECK(sample.ok());
        CHECK_NEAR(quat::to_yaw(sample.value()).value(), 2.0 * t, 1e-12);
    }
}

TEST(slerp_takes_the_short_way_round)
{
    // b is stored as the antipode of a rotation 0.2 rad away. Without the
    // hemisphere flip this interpolates the long way: a visible 350-degree
    // spin where 10 degrees was meant.
    const Quaternion a = quat::from_yaw(0.0);
    const Quaternion near = quat::from_yaw(0.2);
    const Quaternion antipodal{.x = -near.x, .y = -near.y, .z = -near.z, .w = -near.w};

    const auto middle = quat::slerp(a, antipodal, 0.5);
    CHECK(middle.ok());
    CHECK_NEAR(std::abs(quat::to_yaw(middle.value()).value()), 0.1, 1e-12);
}

TEST(slerp_of_identical_rotations_is_that_rotation)
{
    const Quaternion a = quat::from_rpy(0.3, -0.2, 1.1);
    const auto result = quat::slerp(a, a, 0.37);
    CHECK(result.ok());
    CHECK(cwtest::same_rotation(result.value(), a, 1e-12));
}

TEST(slerp_stays_unit_across_the_arc)
{
    const Quaternion a = quat::from_rpy(0.1, 0.2, 0.3);
    const Quaternion b = quat::from_rpy(-1.2, 0.9, -2.0);
    for (int step = 0; step <= 20; ++step)
    {
        const auto sample = quat::slerp(a, b, step / 20.0);
        CHECK(sample.ok());
        CHECK_NEAR(quat::norm(sample.value()), 1.0, 1e-12);
    }
}

TEST(slerp_falls_back_to_nlerp_when_nearly_parallel)
{
    // Below the threshold the slerp denominator stops being useful; the two
    // must agree closely rather than one of them producing a NaN.
    const Quaternion a = quat::from_yaw(0.0);
    const Quaternion b = quat::from_yaw(1e-4);
    const auto spherical = quat::slerp(a, b, 0.5);
    const auto linear = quat::nlerp(a, b, 0.5);
    CHECK(spherical.ok() && linear.ok());
    CHECK_NEAR(spherical.value(), linear.value(), 1e-15);
}

TEST(nlerp_endpoints_are_the_inputs)
{
    const Quaternion a = quat::from_yaw(0.2);
    const Quaternion b = quat::from_yaw(1.4);
    CHECK(cwtest::same_rotation(quat::nlerp(a, b, 0.0).value(), a, 1e-12));
    CHECK(cwtest::same_rotation(quat::nlerp(a, b, 1.0).value(), b, 1e-12));
}

TEST(nlerp_stays_unit_but_not_constant_speed)
{
    const Quaternion a = quat::from_yaw(0.0);
    const Quaternion b = quat::from_yaw(2.0);
    const auto middle = quat::nlerp(a, b, 0.5);
    CHECK(middle.ok());
    CHECK_NEAR(quat::norm(middle.value()), 1.0, 1e-12);
    // Halfway in t is halfway in angle for nlerp only by symmetry at t=0.5;
    // a quarter of the way is measurably not a quarter of the angle.
    const auto quarter = quat::nlerp(a, b, 0.25);
    CHECK(quarter.ok());
    CHECK(std::abs(quat::to_yaw(quarter.value()).value() - 0.5) > 1e-6);
}

TEST(interpolation_extrapolates_rather_than_clamping)
{
    const Quaternion a = quat::from_yaw(0.0);
    const Quaternion b = quat::from_yaw(0.4);
    const auto ahead = quat::slerp(a, b, 2.0);
    CHECK(ahead.ok());
    CHECK_NEAR(quat::to_yaw(ahead.value()).value(), 0.8, 1e-12);
}

TEST(interpolation_rejects_degenerate_and_non_finite_input)
{
    const Quaternion a = quat::from_yaw(0.3);
    const Quaternion degenerate{.x = 0, .y = 0, .z = 0, .w = 0};
    CHECK(!quat::slerp(a, degenerate, 0.5).ok());
    CHECK(!quat::slerp(degenerate, a, 0.5).ok());
    CHECK(!quat::nlerp(a, degenerate, 0.5).ok());
    CHECK(!quat::slerp(a, a, std::nan("")).ok());
    CHECK(quat::slerp(a, a, std::nan("")).error().code == ErrorCode::kNonFiniteValue);
}

int main() { return cwtest::run_all("quaternion"); }
