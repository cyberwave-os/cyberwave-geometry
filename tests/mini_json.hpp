// A minimal JSON value, parser and writer for the golden-vector tests.
//
// Test-only, and deliberately so: the geometry core promises to build with
// nothing but a standard library, and the golden file has to be readable by
// Python, TypeScript and Kotlin too, so it is JSON rather than some bespoke
// format that would only save this one parser.
//
// Handles exactly the subset the generator emits: objects, arrays, strings
// without escapes beyond the basics, finite numbers, booleans and null.
#ifndef CYBERWAVE_GEOMETRY_MINI_JSON_HPP
#define CYBERWAVE_GEOMETRY_MINI_JSON_HPP

#include <string>
#include <utility>
#include <vector>

namespace minijson
{

class Json
{
public:
    enum class Type
    {
        kNull,
        kBool,
        kNumber,
        kString,
        kArray,
        kObject
    };

    Json() = default;
    static Json boolean(bool value);
    static Json number(double value);
    static Json string(std::string value);
    static Json array();
    static Json object();

    Type type() const { return type_; }
    bool is_null() const { return type_ == Type::kNull; }
    bool is_object() const { return type_ == Type::kObject; }
    bool is_array() const { return type_ == Type::kArray; }

    bool as_bool() const { return bool_; }
    double as_number() const { return number_; }
    const std::string& as_string() const { return string_; }

    /// Array access.
    void push_back(Json value);
    std::size_t size() const { return items_.size(); }
    const Json& at(std::size_t index) const { return items_[index]; }

    /// Object access. Insertion order is preserved so the emitted file reads
    /// in a sensible order rather than alphabetically.
    void set(const std::string& key, Json value);
    bool has(const std::string& key) const;
    /// Returns a static null for a missing key, so a malformed golden file
    /// produces a comparison failure rather than a crash.
    const Json& get(const std::string& key) const;
    const std::vector<std::pair<std::string, Json>>& entries() const { return entries_; }

    std::string dump(int indent = 2) const;

    /// Returns a null value and sets `error` when the text will not parse.
    static Json parse(const std::string& text, std::string* error);

private:
    void dump_into(std::string& out, int indent, int depth) const;

    Type type_ = Type::kNull;
    bool bool_ = false;
    double number_ = 0.0;
    std::string string_;
    std::vector<Json> items_;
    std::vector<std::pair<std::string, Json>> entries_;
};

} // namespace minijson

#endif // CYBERWAVE_GEOMETRY_MINI_JSON_HPP
