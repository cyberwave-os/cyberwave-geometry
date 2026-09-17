// The golden-vector case catalogue and the op dispatch that evaluates it.
//
// One list, two consumers: `golden_gen` writes the file from it, and
// `test_golden` re-evaluates every case in the committed file and compares.
// The same op names and input shapes are what the Python, TypeScript and
// Kotlin conformance runners implement, so this header is the contract for
// all four.
#ifndef CYBERWAVE_GEOMETRY_GOLDEN_OPS_HPP
#define CYBERWAVE_GEOMETRY_GOLDEN_OPS_HPP

#include "mini_json.hpp"

#include <string>

namespace golden
{

/// Tolerance for every numeric comparison in the conformance runners.
///
/// Not exact equality: libm's sin/cos differ in the last ulp between platforms
/// and languages, so a bit-for-bit golden file would fail for reasons that have
/// nothing to do with the geometry. Loose enough to absorb that, tight enough
/// that a wrong sign, a swapped component order or a transposed matrix fails.
constexpr double kTolerance = 1e-12;

/// The budget for comparing against `expected`: relative above 1, absolute
/// below it.
///
/// A flat absolute budget cannot absorb the cross-platform drift kTolerance
/// exists for: one ulp of the 1.6e7 metre offsets the far-from-anchor
/// `geo.to_local.south` cases produce is 3.7e-9, so those only ever passed on
/// the machine that generated the file.
inline double golden_budget(double expected)
{
    const double magnitude = expected < 0.0 ? -expected : expected;
    return kTolerance * (magnitude > 1.0 ? magnitude : 1.0);
}

/// The full document: metadata, robot descriptions and every case with the
/// expectation the core currently produces.
minijson::Json build_document();

/// Recompute every case in `document` from its own input and compare with the
/// stored expectation. Returns the number of mismatches and appends a
/// human-readable account of each to `report`.
int verify_document(const minijson::Json& document, std::string* report);

/// Evaluate one case. Exposed so a language binding's conformance runner has a
/// reference for what each op is supposed to return.
minijson::Json evaluate(const minijson::Json& robots, const std::string& op, const minijson::Json& input);

} // namespace golden

#endif // CYBERWAVE_GEOMETRY_GOLDEN_OPS_HPP
