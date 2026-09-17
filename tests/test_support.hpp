// A ~100-line test registry, because the geometry core has no dependencies and
// pulling gtest in through FetchContent to assert on doubles would give it one.
#ifndef CYBERWAVE_GEOMETRY_TEST_SUPPORT_HPP
#define CYBERWAVE_GEOMETRY_TEST_SUPPORT_HPP

#include "cyberwave/geometry/geometry.hpp"

#include <string>
#include <vector>

namespace cwtest
{

using TestFn = void (*)();

struct Case
{
    const char* name;
    TestFn fn;
};

std::vector<Case>& registry();
void fail(const char* file, int line, const std::string& message);
int run_all(const char* suite_name);

struct Registrar
{
    Registrar(const char* name, TestFn fn) { registry().push_back(Case{name, fn}); }
};

/// Default comparison tolerance. Loose enough for a double round-trip through
/// transcendentals, tight enough that a genuinely wrong formula fails.
constexpr double kEps = 1e-12;

bool near(double a, double b, double tolerance);
bool near(const cyberwave::geometry::Vector3& a, const cyberwave::geometry::Vector3& b, double tolerance);
/// True when the two quaternions are the same *rotation*: q and -q both are.
bool same_rotation(const cyberwave::geometry::Quaternion& a, const cyberwave::geometry::Quaternion& b,
                   double tolerance);
bool near(const cyberwave::geometry::Quaternion& a, const cyberwave::geometry::Quaternion& b, double tolerance);
bool near(const cyberwave::geometry::Transform& a, const cyberwave::geometry::Transform& b, double tolerance);

std::string to_string(double value);
std::string to_string(const cyberwave::geometry::Vector3& v);
std::string to_string(const cyberwave::geometry::Quaternion& q);
std::string to_string(const cyberwave::geometry::Transform& t);

} // namespace cwtest

#define TEST(name)                                                                                                     \
    static void name();                                                                                                \
    static ::cwtest::Registrar cwtest_registrar_##name(#name, name);                                                   \
    static void name()

#define CHECK(condition)                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(condition))                                                                                              \
        {                                                                                                              \
            ::cwtest::fail(__FILE__, __LINE__, "CHECK failed: " #condition);                                           \
            return;                                                                                                    \
        }                                                                                                              \
    } while (false)

#define CHECK_MSG(condition, message)                                                                                  \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(condition))                                                                                              \
        {                                                                                                              \
            ::cwtest::fail(__FILE__, __LINE__, std::string("CHECK failed: " #condition " -- ") + (message));           \
            return;                                                                                                    \
        }                                                                                                              \
    } while (false)

#define CHECK_NEAR(actual, expected, tolerance)                                                                        \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!::cwtest::near((actual), (expected), (tolerance)))                                                        \
        {                                                                                                              \
            ::cwtest::fail(__FILE__, __LINE__,                                                                         \
                           std::string("CHECK_NEAR failed: " #actual " != " #expected "\n"                             \
                                       "        actual:   ") +                                                         \
                               ::cwtest::to_string(actual) + "\n        expected: " + ::cwtest::to_string(expected));  \
            return;                                                                                                    \
        }                                                                                                              \
    } while (false)

#endif // CYBERWAVE_GEOMETRY_TEST_SUPPORT_HPP
