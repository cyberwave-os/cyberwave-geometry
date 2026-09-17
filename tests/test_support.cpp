#include "test_support.hpp"

#include <cmath>
#include <cstdio>

namespace cwtest
{
namespace
{

int g_failures_in_case = 0;

} // namespace

std::vector<Case>& registry()
{
    static std::vector<Case> cases;
    return cases;
}

void fail(const char* file, int line, const std::string& message)
{
    ++g_failures_in_case;
    std::fprintf(stderr, "      %s:%d: %s\n", file, line, message.c_str());
}

int run_all(const char* suite_name)
{
    int failed_cases = 0;
    std::printf("[%s] %zu cases\n", suite_name, registry().size());
    for (const Case& test_case : registry())
    {
        g_failures_in_case = 0;
        test_case.fn();
        if (g_failures_in_case != 0)
        {
            ++failed_cases;
            std::printf("  FAIL %s\n", test_case.name);
        }
    }
    if (failed_cases == 0)
    {
        std::printf("[%s] all %zu cases passed\n", suite_name, registry().size());
        return 0;
    }
    std::printf("[%s] %d of %zu cases FAILED\n", suite_name, failed_cases, registry().size());
    return 1;
}

bool near(double a, double b, double tolerance)
{
    if (std::isnan(a) || std::isnan(b))
    {
        return false;
    }
    return std::abs(a - b) <= tolerance;
}

bool near(const cyberwave::geometry::Vector3& a, const cyberwave::geometry::Vector3& b, double tolerance)
{
    return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) && near(a.z, b.z, tolerance);
}

bool near(const cyberwave::geometry::Quaternion& a, const cyberwave::geometry::Quaternion& b, double tolerance)
{
    return near(a.w, b.w, tolerance) && near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) &&
           near(a.z, b.z, tolerance);
}

bool same_rotation(const cyberwave::geometry::Quaternion& a, const cyberwave::geometry::Quaternion& b, double tolerance)
{
    const cyberwave::geometry::Quaternion negated{.x = -b.x, .y = -b.y, .z = -b.z, .w = -b.w};
    return near(a, b, tolerance) || near(a, negated, tolerance);
}

bool near(const cyberwave::geometry::Transform& a, const cyberwave::geometry::Transform& b, double tolerance)
{
    return near(a.translation, b.translation, tolerance) && near(a.rotation, b.rotation, tolerance);
}

namespace
{

std::string number(double value)
{
    char buffer[40];
    std::snprintf(buffer, sizeof(buffer), "%.12g", value);
    return buffer;
}

} // namespace

std::string to_string(double value) { return number(value); }

std::string to_string(const cyberwave::geometry::Vector3& v)
{
    return "(x=" + number(v.x) + ", y=" + number(v.y) + ", z=" + number(v.z) + ")";
}

std::string to_string(const cyberwave::geometry::Quaternion& q)
{
    return "(w=" + number(q.w) + ", x=" + number(q.x) + ", y=" + number(q.y) + ", z=" + number(q.z) + ")";
}

std::string to_string(const cyberwave::geometry::Transform& t)
{
    return "{t=" + to_string(t.translation) + ", r=" + to_string(t.rotation) + "}";
}

} // namespace cwtest
