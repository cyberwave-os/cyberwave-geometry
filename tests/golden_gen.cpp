// Writes or checks common/geometry/golden/geometry_golden.json.
//
//   golden_gen --write <path>   regenerate the file from the current core
//   golden_gen --check <path>   re-evaluate every case and compare
//
// The check is not a byte comparison: libm differs in the last ulp between
// platforms, so it parses the file and compares numerically with the same
// tolerance every other language's conformance runner uses.
#include "golden_ops.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

namespace
{

int usage()
{
    std::fprintf(stderr, "usage: golden_gen (--write|--check) <path>\n");
    return 2;
}

bool read_file(const std::string& path, std::string* out)
{
    std::ifstream stream(path);
    if (!stream)
    {
        return false;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    *out = buffer.str();
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        return usage();
    }
    const std::string mode = argv[1];
    const std::string path = argv[2];

    if (mode == "--write")
    {
        std::ofstream stream(path);
        if (!stream)
        {
            std::fprintf(stderr, "cannot write %s\n", path.c_str());
            return 1;
        }
        const minijson::Json document = golden::build_document();
        stream << document.dump();
        std::printf("wrote %zu cases to %s\n", document.get("cases").size(), path.c_str());
        return 0;
    }

    if (mode != "--check")
    {
        return usage();
    }

    std::string text;
    if (!read_file(path, &text))
    {
        std::fprintf(stderr, "cannot read %s -- generate it with `golden_gen --write %s`\n", path.c_str(),
                     path.c_str());
        return 1;
    }
    std::string parse_error;
    const minijson::Json document = minijson::Json::parse(text, &parse_error);
    if (!parse_error.empty())
    {
        std::fprintf(stderr, "%s is not valid JSON: %s\n", path.c_str(), parse_error.c_str());
        return 1;
    }

    const std::size_t case_count = document.get("cases").size();
    if (case_count == 0)
    {
        std::fprintf(stderr, "%s contains no cases\n", path.c_str());
        return 1;
    }

    // A binding pins itself to the core version it was conformance-tested
    // against, so a stale golden file is worth naming explicitly.
    const std::string file_version = document.get("version").as_string();
    if (file_version != golden::build_document().get("version").as_string())
    {
        std::printf("[golden] note: file version %s, core version %s\n", file_version.c_str(),
                    golden::build_document().get("version").as_string().c_str());
    }

    std::string report;
    const int mismatches = golden::verify_document(document, &report);
    if (mismatches == 0)
    {
        std::printf("[golden] all %zu cases match\n", case_count);
        return 0;
    }
    std::fputs(report.c_str(), stdout);
    std::printf("[golden] %d of %zu cases MISMATCHED\n", mismatches, case_count);
    std::printf("[golden] if this change is intended, regenerate with:\n"
                "         golden_gen --write %s\n",
                path.c_str());
    return 1;
}
