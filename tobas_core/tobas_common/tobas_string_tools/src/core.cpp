// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_string_tools/core.hpp"

#include <algorithm>
#include <map>
#include <regex>
#include <sstream>

namespace tobas
{
namespace str
{
std::vector<std::string> split(const std::string& _s, char _c)
{
  std::vector<std::string> res;
  if (_s.find(_c) == std::string::npos) {
    res.push_back(_s);
  }
  else {
    size_t first = 0;
    size_t last = _s.find_first_of(_c);
    while (first < _s.size()) {
      std::string subStr(_s, first, last - first);
      res.push_back(subStr);
      first = last + 1;
      last = _s.find_first_of(_c, first);
      if (last == std::string::npos) {
        last = _s.size();
      }
    }
  }
  return res;
}

std::expected<std::pair<std::string, std::string>, std::string> rsplit(const std::string& _s, char _c)
{
  // Find the last specified character.
  const auto pos = _s.rfind(_c);

  if (pos != std::string::npos) {
    std::string before = _s.substr(0, pos);
    std::string after = _s.substr(pos + 1);
    return std::pair{ before, after };
  }
  else {
    return std::unexpected("String '" + _s + "' does not contain '" + _c + "'.");
  }
}

std::string lstrip(const std::string& _s, const std::string& _del)
{
  if (_s.find(_del) == 0) {
    return _s.substr(_del.length());
  }
  else {
    return _s;
  }
}

std::string rstrip(const std::string& _s, const std::string& _del)
{
  if (_s.size() >= _del.size() && _s.compare(_s.size() - _del.size(), _del.size(), _del) == 0) {
    return _s.substr(0, _s.size() - _del.size());
  }
  else {
    return _s;
  }
}

std::string stripQuates(const std::string& _s)
{
  if (_s.front() == '"' && _s.back() == '"') {
    return _s.substr(1, _s.size() - 2);
  }
  return _s;
}

std::string trim(const std::string& _s)
{
  // Return as-is for an empty string. Otherwise, `find_(first,last)_not_of` would return `inf`.
  if (_s.empty()) {
    return {};
  }

  constexpr char del[] = " \t\n\r\f\v";
  const auto first = _s.find_first_not_of(del);
  const auto last = _s.find_last_not_of(del);

  return _s.substr(first, last - first + 1);
}

std::vector<std::string> splitLines(const std::string& _text)
{
  std::vector<std::string> lines;
  std::istringstream stream(_text);
  std::string line;
  while (std::getline(stream, line)) {
    lines.push_back(line);
  }
  return lines;
}

std::string deleteNl(const std::string& _s)
{
  std::string res;
  for (const auto& ch : _s) {
    if (ch != '\r' && ch != '\n') {
      res += ch;
    }
  }
  return res;
}

std::string toLower(std::string _arg)
{
  transform(_arg.begin(), _arg.end(), _arg.begin(), [](char _c) { return std::tolower(_c); });
  return _arg;
}

std::string toUpper(std::string _arg)
{
  transform(_arg.begin(), _arg.end(), _arg.begin(), [](char _c) { return std::toupper(_c); });
  return _arg;
}

std::string replace(std::string _s, const std::string& _from, const std::string& _to)
{
  size_t start_pos = 0;
  while ((start_pos = _s.find(_from, start_pos)) != std::string::npos) {
    _s.replace(start_pos, _from.length(), _to);
    start_pos += _to.length();  // Set the next search position.
  }
  return _s;
}

std::string sanitize(const char* _s)
{
  if (!_s) {
    return {};
  }

  std::string out(_s);

  // Replace newlines and tabs with spaces.
  for (auto& c : out) {
    switch (c) {
      case '\n':
      case '\r':
      case '\t':
      case '\v':
      case '\f':
        c = ' ';
        break;
      default:
        break;
    }
  }

  // Remove other control characters (0x00-0x1F, 0x7F).
  out.erase(remove_if(out.begin(), out.end(), [](char _ch) { return (_ch < 0x20 || _ch == 0x7F); }), out.end());

  // Collapse consecutive spaces to one.
  out.erase(unique(out.begin(), out.end(), [](char _a, char _b) { return _a == ' ' && _b == ' '; }), out.end());

  // Trim leading and trailing spaces.
  const auto notspace = [](char _c) { return _c != ' '; };
  const auto first = find_if(out.begin(), out.end(), notspace);
  if (first == out.end()) {
    return {};  // All spaces.
  }
  const auto last = find_if(out.rbegin(), out.rend(), notspace).base();
  return std::string(first, last);
}

bool isValidFileName(const std::string& _file_name)
{
  if (_file_name.empty() || _file_name == "." || _file_name == "..") {
    return false;
  }

  // Reject path separators and reserved punctuation.
  constexpr char kInvalidChars[] = R"(<>:"/\|?*)";
  if (_file_name.find_first_of(kInvalidChars) != std::string::npos) {
    return false;
  }

  // Check ASCII control bytes, including NUL, without rejecting UTF-8 bytes.
  return std::none_of(_file_name.begin(), _file_name.end(), [](unsigned char _ch) { return _ch < 0x20 || _ch == 0x7F; });
}

bool isValidEmail(const std::string& _email)
{
  const std::regex pattern(R"(\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,7}\b)");
  return std::regex_match(_email, pattern);
}

std::string convertToSuperscript(const std::string& _input)
{
  // Superscript mapping table.
  const std::map<std::string, std::string> superscripts = {
    { "^0", "⁰" }, { "^1", "¹" }, { "^2", "²" }, { "^3", "³" }, { "^4", "⁴" },
    { "^5", "⁵" }, { "^6", "⁶" }, { "^7", "⁷" }, { "^8", "⁸" }, { "^9", "⁹" },
  };

  std::string output;
  size_t pos = 0;

  while (pos < _input.size()) {
    if (_input[pos] == '^' && pos + 1 < _input.size() && std::isdigit(_input[pos + 1])) {
      const auto key = _input.substr(pos, 2);  // Get "^N".
      const auto it = superscripts.find(key);
      if (it != superscripts.end()) {
        output += it->second;  // Convert to superscript.
        pos += 2;
        continue;
      }
    }
    output += _input[pos];
    ++pos;
  }

  return output;
}

std::string pascalFromTitle(const std::string& _title_case)
{
  return regex_replace(_title_case, std::regex(" "), "");
}

std::string pascalFromSnake(const std::string& _snake_case)
{
  std::stringstream result;
  std::stringstream ss(_snake_case);
  std::string item;
  while (std::getline(ss, item, '_')) {
    item[0] = std::toupper(item[0]);
    result << item;
  }
  return result.str();
}

std::string titleFromSnake(const std::string& _snake_case)
{
  std::stringstream result;
  std::stringstream ss(_snake_case);
  std::string item;
  bool first = true;
  while (std::getline(ss, item, '_')) {
    item[0] = std::toupper(item[0]);
    if (!first) {
      result << ' ';
    }
    result << item;
    first = false;
  }
  return result.str();
}

std::string snakeFromPascal(const std::string& _pascal_case)
{
  std::stringstream result;
  for (size_t i = 0; i < _pascal_case.size(); ++i) {
    if (isupper(_pascal_case[i]) && i > 0) {
      result << '_';
    }
    result << static_cast<char>(std::tolower(_pascal_case[i]));
  }
  return result.str();
}

std::string snakeFromTitle(const std::string& _title_case)
{
  return snakeFromPascal(pascalFromTitle(_title_case));
}
}  // namespace str
}  // namespace tobas
