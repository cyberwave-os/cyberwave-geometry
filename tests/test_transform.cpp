// Rigid transform composition, inversion and the vector ops underneath them.
#include "test_support.hpp"

#include <cmath>

using namespace cyberwave::geometry;

namespace
{

constexpr double kPi = 3.14159265358979323846;

const Transform kA{Vector3{1.0, 2.0, 3.0}, quat::from_rpy(0.2, -0.3, 0.4)};
const Transform kB{Vector3{-0.5, 0.25, 2.0}, quat::from_rpy(-0.1, 0.7, 0.15)};

} // namespace

TEST(compose_with_identity_is_a_no_op)
{
    CHECK_NEAR(tf::compose(kA, Transform::identity()), kA, cwtest::kEps);
    CHECK_NEAR(tf::compose(Transform::identity(), kA), kA, cwtest::kEps);
}

TEST(compose_is_associative)
{
    const Transform c{Vector3{3.0, -1.0, 0.5}, quat::from_yaw(0.8)};
    const Transform left = tf::compose(tf::compose(kA, kB), c);
    const Transform right = tf::compose(kA, tf::compose(kB, c));
    CHECK_NEAR(left, right, 1e-12);
}

TEST(compose_agrees_with_applying_the_transforms_in_turn)
{
    const Vector3 point{0.7, -1.4, 2.1};
    const Vector3 stepwise = tf::apply(kA, tf::apply(kB, point));
    const Vector3 composed = tf::apply(tf::compose(kA, kB), point);
    CHECK_NEAR(composed, stepwise, 1e-12);
}

TEST(apply_rotates_then_translates)
{
    // A pure translation moves a point; a pure rotation does not move the
    // origin. Getting the order backwards breaks exactly one of these.
    const Transform rotation{Vector3::zero(), quat::from_yaw(kPi / 2)};
    CHECK_NEAR(tf::apply(rotation, Vector3::zero()), Vector3::zero(), cwtest::kEps);
    CHECK_NEAR((tf::apply(rotation, Vector3{1, 0, 0})), (Vector3{0, 1, 0}), 1e-15);

    const Transform both{Vector3{10, 0, 0}, quat::from_yaw(kPi / 2)};
    CHECK_NEAR((tf::apply(both, Vector3{1, 0, 0})), (Vector3{10, 1, 0}), 1e-15);
}

TEST(inverse_undoes_compose)
{
    const Transform round_trip = tf::compose(kA, tf::inverse_unit(kA));
    CHECK_NEAR(round_trip, Transform::identity(), 1e-12);
    const Transform other_way = tf::compose(tf::inverse_unit(kA), kA);
    CHECK_NEAR(other_way, Transform::identity(), 1e-12);
}

TEST(inverse_maps_a_point_back)
{
    const Vector3 point{2.5, -0.5, 1.0};
    const Vector3 forward = tf::apply(kA, point);
    CHECK_NEAR(tf::apply(tf::inverse_unit(kA), forward), point, 1e-12);
}

TEST(relative_is_the_inverse_of_compose)
{
    // Express a world-frame child in its parent's frame, then put it back.
    const Transform world_child = tf::compose(kA, kB);
    CHECK_NEAR(tf::relative(kA, world_child), kB, 1e-12);
}

TEST(normalize_scales_the_rotation_and_leaves_translation_alone)
{
    const Transform unnormalized{Vector3{1, 2, 3}, Quaternion{.x = 0, .y = 0, .z = 0, .w = 2}};
    const auto result = tf::normalize(unnormalized);
    CHECK(result.ok());
    CHECK_NEAR(result.value().translation, (Vector3{1, 2, 3}), cwtest::kEps);
    CHECK_NEAR(quat::norm(result.value().rotation), 1.0, cwtest::kEps);
}

TEST(normalize_rejects_a_degenerate_rotation)
{
    const auto result = tf::normalize(Transform{Vector3{1, 2, 3}, Quaternion{.x = 0, .y = 0, .z = 0, .w = 0}});
    CHECK(!result.ok());
    CHECK(result.error().code == ErrorCode::kInvalidQuaternion);
}

TEST(normalize_rejects_a_non_finite_translation)
{
    const Transform broken{Vector3{std::nan(""), 0, 0}, Quaternion::identity()};
    const auto result = tf::normalize(broken);
    CHECK(!result.ok());
    CHECK(result.error().code == ErrorCode::kNonFiniteValue);
}

TEST(checked_inverse_normalizes_first)
{
    // An unnormalized rotation makes inverse_unit() wrong: the conjugate is not
    // the inverse unless the quaternion is unit. inverse() must handle it.
    const Transform unnormalized{Vector3{1, 0, 0}, Quaternion{.x = 0, .y = 0, .z = 0, .w = 3}};
    const auto inverted = tf::inverse(unnormalized);
    CHECK(inverted.ok());
    const Transform normalized = tf::normalize(unnormalized).value();
    CHECK_NEAR(tf::compose(normalized, inverted.value()), Transform::identity(), 1e-12);
}

// --- vector ops -------------------------------------------------------------

TEST(cross_product_is_right_handed)
{
    CHECK_NEAR((vec::cross(Vector3{1, 0, 0}, Vector3{0, 1, 0})), (Vector3{0, 0, 1}), cwtest::kEps);
}

TEST(normalize_axis_rejects_a_zero_vector)
{
    const auto result = vec::normalize_axis(Vector3{0, 0, 0});
    CHECK(!result.ok());
    CHECK(result.error().code == ErrorCode::kInvalidAxis);
}

TEST(normalize_axis_rejects_non_finite_components)
{
    CHECK((!vec::normalize_axis(Vector3{std::nan(""), 0, 1}).ok()));
    CHECK((!vec::normalize_axis(Vector3{INFINITY, 0, 1}).ok()));
}

TEST(normalize_axis_accepts_a_very_small_but_usable_axis)
{
    // Above kMinAxisNorm: a millimetre-scale axis is still a direction.
    const auto result = vec::normalize_axis(Vector3{1e-9, 0, 0});
    CHECK(result.ok());
    CHECK_NEAR(result.value(), (Vector3{1, 0, 0}), 1e-12);
}

int main() { return cwtest::run_all("transform"); }
