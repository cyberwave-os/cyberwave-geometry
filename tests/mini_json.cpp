#include "mini_json.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace minijson
{
namespace
{

const Json& null_value()
{
    static const Json instance;
    return instance;
}

/// %.17g is the shortest form guaranteed to round-trip a double exactly, which
/// is what a golden file storing reference values needs.
std::string format_number(double value)
{
    char buffer[48];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    return buffer;
}

void escape_into(const std::string& text, std::string& out)
{
    out.push_back('"');
    for (char character : text)
    {
        switch (character)
        {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\t':
                out += "\\t";
                break;
            case '\r':
                out += "\\r";
                break;
            default:
                out.push_back(character);
        }
    }
    out.push_back('"');
}

struct Parser
{
    const std::string& text;
    std::size_t position = 0;
    std::string error;

    explicit Parser(const std::string& input) : text(input) {}

    void skip_whitespace()
    {
        while (position < text.size() && std::isspace(static_cast<unsigned char>(text[position])) != 0)
        {
            ++position;
        }
    }

    bool fail(const std::string& message)
    {
        if (error.empty())
        {
            error = message + " at offset " + std::to_string(position);
        }
        return false;
    }

    bool parse_value(Json& out);

    bool parse_string(std::string& out)
    {
        if (position >= text.size() || text[position] != '"')
        {
            return fail("expected string");
        }
        ++position;
        while (position < text.size() && text[position] != '"')
        {
            if (text[position] == '\\')
            {
                ++position;
                if (position >= text.size())
                {
                    return fail("truncated escape");
                }
                switch (text[position])
                {
                    case '"':
                        out.push_back('"');
                        break;
                    case '\\':
                        out.push_back('\\');
                        break;
                    case '/':
                        out.push_back('/');
                        break;
                    case 'n':
                        out.push_back('\n');
                        break;
                    case 't':
                        out.push_back('\t');
                        break;
                    case 'r':
                        out.push_back('\r');
                        break;
                    default:
                        return fail("unsupported escape");
                }
                ++position;
                continue;
            }
            out.push_back(text[position]);
            ++position;
        }
        if (position >= text.size())
        {
            return fail("unterminated string");
        }
        ++position;
        return true;
    }
};

bool Parser::parse_value(Json& out)
{
    skip_whitespace();
    if (position >= text.size())
    {
        return fail("unexpected end of input");
    }
    const char character = text[position];
    if (character == '{')
    {
        ++position;
        out = Json::object();
        skip_whitespace();
        if (position < text.size() && text[position] == '}')
        {
            ++position;
            return true;
        }
        while (true)
        {
            skip_whitespace();
            std::string key;
            if (!parse_string(key))
            {
                return false;
            }
            skip_whitespace();
            if (position >= text.size() || text[position] != ':')
            {
                return fail("expected ':'");
            }
            ++position;
            Json value;
            if (!parse_value(value))
            {
                return false;
            }
            out.set(key, value);
            skip_whitespace();
            if (position < text.size() && text[position] == ',')
            {
                ++position;
                continue;
            }
            if (position < text.size() && text[position] == '}')
            {
                ++position;
                return true;
            }
            return fail("expected ',' or '}'");
        }
    }
    if (character == '[')
    {
        ++position;
        out = Json::array();
        skip_whitespace();
        if (position < text.size() && text[position] == ']')
        {
            ++position;
            return true;
        }
        while (true)
        {
            Json value;
            if (!parse_value(value))
            {
                return false;
            }
            out.push_back(value);
            skip_whitespace();
            if (position < text.size() && text[position] == ',')
            {
                ++position;
                continue;
            }
            if (position < text.size() && text[position] == ']')
            {
                ++position;
                return true;
            }
            return fail("expected ',' or ']'");
        }
    }
    if (character == '"')
    {
        std::string value;
        if (!parse_string(value))
        {
            return false;
        }
        out = Json::string(value);
        return true;
    }
    if (text.compare(position, 4, "true") == 0)
    {
        position += 4;
        out = Json::boolean(true);
        return true;
    }
    if (text.compare(position, 5, "false") == 0)
    {
        position += 5;
        out = Json::boolean(false);
        return true;
    }
    if (text.compare(position, 4, "null") == 0)
    {
        position += 4;
        out = Json();
        return true;
    }
    char* end = nullptr;
    const double value = std::strtod(text.c_str() + position, &end);
    if (end == text.c_str() + position)
    {
        return fail("expected value");
    }
    position = static_cast<std::size_t>(end - text.c_str());
    out = Json::number(value);
    return true;
}

} // namespace

Json Json::boolean(bool value)
{
    Json json;
    json.type_ = Type::kBool;
    json.bool_ = value;
    return json;
}

Json Json::number(double value)
{
    Json json;
    json.type_ = Type::kNumber;
    json.number_ = value;
    return json;
}

Json Json::string(std::string value)
{
    Json json;
    json.type_ = Type::kString;
    json.string_ = std::move(value);
    return json;
}

Json Json::array()
{
    Json json;
    json.type_ = Type::kArray;
    return json;
}

Json Json::object()
{
    Json json;
    json.type_ = Type::kObject;
    return json;
}

void Json::push_back(Json value)
{
    type_ = Type::kArray;
    items_.push_back(std::move(value));
}

void Json::set(const std::string& key, Json value)
{
    type_ = Type::kObject;
    for (auto& entry : entries_)
    {
        if (entry.first == key)
        {
            entry.second = std::move(value);
            return;
        }
    }
    entries_.emplace_back(key, std::move(value));
}

bool Json::has(const std::string& key) const
{
    for (const auto& entry : entries_)
    {
        if (entry.first == key)
        {
            return true;
        }
    }
    return false;
}

const Json& Json::get(const std::string& key) const
{
    for (const auto& entry : entries_)
    {
        if (entry.first == key)
        {
            return entry.second;
        }
    }
    return null_value();
}

void Json::dump_into(std::string& out, int indent, int depth) const
{
    const std::string pad(static_cast<std::size_t>(indent * (depth + 1)), ' ');
    const std::string closing_pad(static_cast<std::size_t>(indent * depth), ' ');
    switch (type_)
    {
        case Type::kNull:
            out += "null";
            return;
        case Type::kBool:
            out += bool_ ? "true" : "false";
            return;
        case Type::kNumber:
            out += format_number(number_);
            return;
        case Type::kString:
            escape_into(string_, out);
            return;
        case Type::kArray:
        {
            if (items_.empty())
            {
                out += "[]";
                return;
            }
            out += "[\n";
            for (std::size_t index = 0; index < items_.size(); ++index)
            {
                out += pad;
                items_[index].dump_into(out, indent, depth + 1);
                out += index + 1 < items_.size() ? ",\n" : "\n";
            }
            out += closing_pad + "]";
            return;
        }
        case Type::kObject:
        {
            if (entries_.empty())
            {
                out += "{}";
                return;
            }
            out += "{\n";
            for (std::size_t index = 0; index < entries_.size(); ++index)
            {
                out += pad;
                escape_into(entries_[index].first, out);
                out += ": ";
                entries_[index].second.dump_into(out, indent, depth + 1);
                out += index + 1 < entries_.size() ? ",\n" : "\n";
            }
            out += closing_pad + "}";
            return;
        }
    }
}

std::string Json::dump(int indent) const
{
    std::string out;
    dump_into(out, indent, 0);
    out.push_back('\n');
    return out;
}

Json Json::parse(const std::string& text, std::string* error)
{
    Parser parser(text);
    Json value;
    if (!parser.parse_value(value))
    {
        if (error != nullptr)
        {
            *error = parser.error;
        }
        return Json();
    }
    parser.skip_whitespace();
    if (parser.position != text.size())
    {
        if (error != nullptr)
        {
            *error = "trailing content at offset " + std::to_string(parser.position);
        }
        return Json();
    }
    return value;
}

} // namespace minijson
