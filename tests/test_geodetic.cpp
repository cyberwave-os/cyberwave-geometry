// The geodetic convention, pinned. Every case here is a statement CONVENTIONS.md
// section 9 makes in prose, written so that changing the prose without changing
// the behaviour -- or the reverse -- fails.
#include "test_support.hpp"

#include <cmath>

using namespace cyberwave::geometry;

namespace
{

constexpr double kPi = 3.14159265358979323846;

// A site with a heading, so that every conversion below would still pass if the
// environment frame happened to coincide with ENU. Zurich, roughly.
const GeoAnchor kAnchor{Geodetic{47.3769, 8.5417, 408.0}, 30.0};
const GeoAnchor kEnuAnchor{Geodetic{47.3769, 8.5417, 408.0}, 0.0};

} // namespace

// --- the frame ---------------------------------------------------------------

TEST(zero_heading_means_the_environment_frame_is_enu)
{
    // The defining property of heading_deg: at zero, +X is east and +Y is north.
    const auto east = geo::offset(kEnuAnchor.origin, Vector3{100.0, 0.0, 0.0});
    CHECK(east.ok());
    const auto local = geo::to_local(kEnuAnchor, east.value());
    CHECK(local.ok());
    CHECK_NEAR(local.value(), (Vector3{100.0, 0.0, 0.0}), 1e-6);

    const auto north = geo::offset(kEnuAnchor.origin, Vector3{0.0, 100.0, 0.0});
    CHECK(north.ok());
    const auto local_north = geo::to_local(kEnuAnchor, north.value());
    CHECK(local_north.ok());
    CHECK_NEAR(local_north.value(), (Vector3{0.0, 100.0, 0.0}), 1e-6);
}

TEST(heading_is_the_bearing_of_environment_plus_y)
{
    // heading_deg = 90 means env +Y points east, so a point 100 m east of the
    // anchor lands at env (0, 100), not (100, 0).
    const GeoAnchor rotated{kAnchor.origin, 90.0};
    const auto east = geo::offset(rotated.origin, Vector3{100.0, 0.0, 0.0});
    CHECK(east.ok());
    const auto local = geo::to_local(rotated, east.value());
    CHECK(local.ok());
    CHECK_NEAR(local.value(), (Vector3{0.0, 100.0, 0.0}), 1e-6);
}

TEST(altitude_passes_through_without_a_datum_conversion)
{
    // z is the plain difference. If this ever stops being true, a geoid model
    // has been smuggled in and the ~30 m MSL/HAE question has been answered
    // somewhere it should not have been.
    const Geodetic higher{kAnchor.origin.latitude_deg, kAnchor.origin.longitude_deg, kAnchor.origin.altitude_m + 12.5};
    const auto local = geo::to_local(kAnchor, higher);
    CHECK(local.ok());
    CHECK_NEAR(local.value().z, 12.5, 1e-9);
}

// --- scale -------------------------------------------------------------------

TEST(metres_per_degree_uses_the_ellipsoid_not_a_mean_radius)
{
    const auto equator = geo::metres_per_degree(0.0);
    CHECK(equator.ok());
    // At the equator the meridional radius is SMALLER than the semi-major axis
    // by a factor of (1 - e2); a spherical constant gets this 0.67% wrong.
    const double spherical = kPi / 180.0 * kWgs84SemiMajorAxis;
    CHECK(equator.value().latitude < spherical);
    CHECK(cwtest::near(equator.value().latitude / spherical, 1.0 - kWgs84EccentricitySquared, 1e-12));
    CHECK_NEAR(equator.value().longitude, spherical, 1e-9);

    // Metres per degree of latitude GROWS towards the poles, longitude shrinks.
    const auto high = geo::metres_per_degree(78.0);
    CHECK(high.ok());
    CHECK(high.value().latitude > equator.value().latitude);
    CHECK(high.value().longitude < 0.25 * equator.value().longitude);
}

TEST(metres_per_degree_rejects_a_latitude_off_the_ellipsoid)
{
    CHECK(!geo::metres_per_degree(91.0).ok());
    CHECK(!geo::metres_per_degree(std::nan("")).ok());
}

// --- position round trips ----------------------------------------------------

TEST(enu_between_and_offset_are_inverses)
{
    const Vector3 delta{321.5, -874.25, 13.0};
    const auto moved = geo::offset(kAnchor.origin, delta);
    CHECK(moved.ok());
    const auto recovered = geo::enu_between(kAnchor.origin, moved.value());
    CHECK(recovered.ok());
    CHECK_NEAR(recovered.value(), delta, 1e-6);
}

TEST(to_local_and_from_local_are_inverses)
{
    const Vector3 local{-412.0, 118.5, -7.25};
    const auto geodetic = geo::from_local(kAnchor, local);
    CHECK(geodetic.ok());
    const auto recovered = geo::to_local(kAnchor, geodetic.value());
    CHECK(recovered.ok());
    CHECK_NEAR(recovered.value(), local, 1e-6);
}

TEST(a_longitude_difference_across_the_antimeridian_is_metres_not_a_half_turn)
{
    // Two points 200 m apart either side of 180 degrees. Subtracting the raw
    // longitudes gives ~360 degrees, which reads as most of the way round the
    // planet and makes GPS simply never work in Fiji or Chukotka.
    const Geodetic west{-16.5, 179.999, 0.0};
    const Geodetic east{-16.5, -179.999, 0.0};
    const auto distance = geo::ground_distance(west, east);
    CHECK(distance.ok());
    CHECK(distance.value() < 500.0);

    // And the inverse hands back a wrapped longitude rather than 180.001.
    const auto crossed = geo::offset(west, Vector3{500.0, 0.0, 0.0});
    CHECK(crossed.ok());
    CHECK(crossed.value().longitude_deg < 0.0);
    CHECK(crossed.value().longitude_deg >= -180.0);
}

TEST(ground_distance_ignores_altitude)
{
    const Geodetic high{kAnchor.origin.latitude_deg, kAnchor.origin.longitude_deg, kAnchor.origin.altitude_m + 5000.0};
    const auto distance = geo::ground_distance(kAnchor.origin, high);
    CHECK(distance.ok());
    CHECK_NEAR(distance.value(), 0.0, 1e-9);
}

// --- the 6-DOF pose ----------------------------------------------------------

TEST(geo_pose_round_trips_through_the_environment_frame)
{
    const GeoPose pose{Geodetic{47.3801, 8.5500, 421.0}, quat::from_rpy(0.12, -0.34, 1.05)};
    const auto local = geo::to_local_pose(kAnchor, pose);
    CHECK(local.ok());
    const auto recovered = geo::from_local_pose(kAnchor, local.value());
    CHECK(recovered.ok());
    CHECK_NEAR(recovered.value().position.latitude_deg, pose.position.latitude_deg, 1e-11);
    CHECK_NEAR(recovered.value().position.longitude_deg, pose.position.longitude_deg, 1e-11);
    CHECK_NEAR(recovered.value().position.altitude_m, pose.position.altitude_m, 1e-9);
    CHECK_NEAR(recovered.value().orientation, pose.orientation, 1e-12);
}

TEST(an_identity_orientation_faces_east)
{
    // The one counter-intuitive consequence of building on ENU: identity is not
    // "facing north". quat_from_compass_heading exists so nobody relies on this
    // by accident, but the fact itself has to be pinned.
    const auto heading = geo::to_compass_heading_deg(Quaternion::identity());
    CHECK(heading.ok());
    CHECK_NEAR(heading.value(), 90.0, 1e-9);
}

TEST(a_geo_pose_orientation_is_rotated_by_the_anchor_heading)
{
    // Same attitude on Earth, two sites whose frames differ by 30 degrees: the
    // environment-frame rotation has to differ by exactly that yaw.
    const GeoPose pose{kAnchor.origin, quat::from_yaw(0.0)};
    const auto turned = geo::to_local_pose(kAnchor, pose);
    const auto aligned = geo::to_local_pose(kEnuAnchor, pose);
    CHECK(turned.ok() && aligned.ok());
    const auto turned_yaw = quat::to_yaw(turned.value().rotation);
    const auto aligned_yaw = quat::to_yaw(aligned.value().rotation);
    CHECK(turned_yaw.ok() && aligned_yaw.ok());
    CHECK_NEAR(turned_yaw.value() - aligned_yaw.value(), 30.0 * kPi / 180.0, 1e-12);
}

// --- compass bearings --------------------------------------------------------

TEST(compass_heading_is_clockwise_from_north)
{
    // Due north is a bearing of 0 and an ENU yaw of +90 degrees; due east is a
    // bearing of 90 and a yaw of 0. Getting the sense backwards passes at 45
    // and 225 degrees and nowhere else, which is why both are checked.
    const auto north = geo::quat_from_compass_heading(0.0);
    CHECK(north.ok());
    const auto north_yaw = quat::to_yaw(north.value());
    CHECK(north_yaw.ok());
    CHECK_NEAR(north_yaw.value(), kPi / 2.0, 1e-12);

    const auto east = geo::quat_from_compass_heading(90.0);
    CHECK(east.ok());
    const auto east_yaw = quat::to_yaw(east.value());
    CHECK(east_yaw.ok());
    CHECK_NEAR(east_yaw.value(), 0.0, 1e-12);

    const auto west = geo::quat_from_compass_heading(270.0);
    CHECK(west.ok());
    const auto west_yaw = quat::to_yaw(west.value());
    CHECK(west_yaw.ok());
    CHECK_NEAR(std::fabs(west_yaw.value()), kPi, 1e-12);
}

TEST(compass_heading_round_trips_and_wraps_to_a_positive_bearing)
{
    for (const double bearing : {0.0, 17.5, 90.0, 179.9, 270.0, 359.5})
    {
        const auto q = geo::quat_from_compass_heading(bearing);
        CHECK(q.ok());
        const auto back = geo::to_compass_heading_deg(q.value());
        CHECK(back.ok());
        CHECK(back.value() >= 0.0 && back.value() < 360.0);
        CHECK_NEAR(back.value(), bearing, 1e-9);
    }

    // An out-of-range bearing is accepted and wrapped, not rejected: a compass
    // that reports 361.0 has drifted, not broken.
    const auto wrapped = geo::quat_from_compass_heading(450.0);
    CHECK(wrapped.ok());
    const auto back = geo::to_compass_heading_deg(wrapped.value());
    CHECK(back.ok());
    CHECK_NEAR(back.value(), 90.0, 1e-9);
}

TEST(a_compass_heading_produces_a_yaw_only_rotation)
{
    // A fix plus a compass carries no attitude. Inventing a level one would be
    // indistinguishable from a measured one.
    const auto q = geo::quat_from_compass_heading(212.0);
    CHECK(q.ok());
    CHECK(q.value().x == 0.0);
    CHECK(q.value().y == 0.0);
}

// --- NED ---------------------------------------------------------------------

TEST(ned_yaw_is_a_bearing_and_ned_pitch_is_nose_up)
{
    // Feeding NED angles straight into from_rpy is a reflection about the
    // north-east diagonal. It is right at 45 degrees and wrong everywhere else,
    // so the cardinal headings are what this checks.
    const auto due_north = geo::quat_from_ned_rpy(0.0, 0.0, 0.0);
    CHECK(due_north.ok());
    const auto heading = geo::to_compass_heading_deg(due_north.value());
    CHECK(heading.ok());
    CHECK_NEAR(heading.value(), 0.0, 1e-9);

    const auto due_east = geo::quat_from_ned_rpy(0.0, 0.0, kPi / 2.0);
    CHECK(due_east.ok());
    const auto east_heading = geo::to_compass_heading_deg(due_east.value());
    CHECK(east_heading.ok());
    CHECK_NEAR(east_heading.value(), 90.0, 1e-9);

    // Positive NED pitch is nose-UP (aerospace), while positive ENU/FLU pitch is
    // nose-DOWN (ROS REP-103, rotation about +Y = left). That disagreement is the
    // whole reason `pitch_enu = -pitch_ned`, and a sign slip here points a nadir
    // camera at the sky.
    const auto nose_up = geo::quat_from_ned_rpy(0.0, 0.5, 0.0);
    CHECK(nose_up.ok());
    CHECK(quat::rotate_unit(nose_up.value(), Vector3{1.0, 0.0, 0.0}).z > 0.0);

    const auto nose_down = geo::quat_from_ned_rpy(0.0, -0.5, 0.0);
    CHECK(nose_down.ok());
    CHECK(quat::rotate_unit(nose_down.value(), Vector3{1.0, 0.0, 0.0}).z < 0.0);
}

TEST(ned_attitude_round_trips)
{
    const double roll = 0.21;
    const double pitch = -0.44;
    const double yaw = 2.05;
    const auto q = geo::quat_from_ned_rpy(roll, pitch, yaw);
    CHECK(q.ok());
    const auto back = geo::to_ned_rpy(q.value());
    CHECK(back.ok());
    CHECK_NEAR(back.value().roll, roll, 1e-12);
    CHECK_NEAR(back.value().pitch, pitch, 1e-12);
    CHECK_NEAR(back.value().yaw, yaw, 1e-12);
}

TEST(ned_yaw_comes_back_wrapped_rather_than_drifting_a_turn)
{
    const auto q = geo::quat_from_ned_rpy(0.0, 0.0, -3.0);
    CHECK(q.ok());
    const auto back = geo::to_ned_rpy(q.value());
    CHECK(back.ok());
    CHECK(back.value().yaw > -kPi && back.value().yaw <= kPi);
    CHECK_NEAR(back.value().yaw, -3.0, 1e-12);
}

// --- strictness --------------------------------------------------------------

TEST(an_out_of_range_coordinate_is_an_error_not_a_clamp)
{
    CHECK(!geo::validate(Geodetic{91.0, 0.0, 0.0}).ok());
    CHECK(!geo::validate(Geodetic{0.0, 181.0, 0.0}).ok());
    CHECK(!geo::validate(Geodetic{0.0, 0.0, std::nan("")}).ok());
    CHECK(geo::validate(Geodetic{0.0, 0.0, 0.0}).ok());

    const auto failure = geo::validate(Geodetic{91.0, 0.0, 0.0});
    CHECK(failure.error().code == ErrorCode::kInvalidGeodetic);
}

TEST(a_polar_anchor_is_refused_rather_than_amplified)
{
    // At the pole the longitude scale goes to zero, so the inverse divides by
    // something approaching nothing and hands back a longitude thousands of
    // degrees wide.
    const GeoAnchor polar{Geodetic{89.95, 0.0, 0.0}, 0.0};
    const auto checked = geo::validate(polar);
    CHECK(!checked.ok());
    CHECK(checked.error().code == ErrorCode::kInvalidGeoAnchor);
    CHECK(!geo::to_local(polar, polar.origin).ok());

    const GeoAnchor usable{Geodetic{89.5, 0.0, 0.0}, 0.0};
    CHECK(geo::validate(usable).ok());
}

TEST(a_degenerate_orientation_is_refused_rather_than_treated_as_identity)
{
    const GeoPose pose{kAnchor.origin, Quaternion{.x = 0.0, .y = 0.0, .z = 0.0, .w = 0.0}};
    const auto local = geo::to_local_pose(kAnchor, pose);
    CHECK(!local.ok());
    CHECK(local.error().code == ErrorCode::kInvalidQuaternion);
}

TEST(an_offset_past_a_pole_is_an_error)
{
    const Geodetic near_pole{89.0, 12.0, 0.0};
    CHECK(!geo::offset(near_pole, Vector3{0.0, 500'000.0, 0.0}).ok());
}

TEST(a_non_finite_heading_makes_the_anchor_unusable)
{
    const GeoAnchor broken{kAnchor.origin, std::nan("")};
    CHECK(!geo::validate(broken).ok());
    CHECK(!geo::to_local(broken, kAnchor.origin).ok());
    CHECK(!geo::from_local(broken, Vector3::zero()).ok());
}

int main() { return cwtest::run_all("geodetic"); }
