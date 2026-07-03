// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_string_tools/core.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <map>
#include <regex>

namespace tobas
{
namespace str
{
std::vector<std::string> split(const std::string& s, const char& c)
{
  std::vector<std::string> res;
  if (s.find(c) == std::string::npos) {
    res.push_back(s);
  }
  else {
    size_t first = 0;
    size_t last = s.find_first_of(c);
    while (first < s.size()) {
      std::string subStr(s, first, last - first);
      res.push_back(subStr);
      first = last + 1;
      last = s.find_first_of(c, first);
      if (last == std::string::npos) {
        last = s.size();
      }
    }
  }
  return res;
}

std::pair<std::string, std::string> rsplit(const std::string& s, const char& c)
{
  // Find the last specified character.
  size_t pos = s.rfind(c);

  if (pos != std::string::npos) {
    std::string before = s.substr(0, pos);
    std::string after = s.substr(pos + 1);
    return { before, after };
  }
  else {
    std::cerr << "String \"" << s << "\" does not contain '" << c << "'" << std::endl;
    return { s, "" };
  }
}

std::string lstrip(const std::string& s, const std::string& del)
{
  if (s.find(del) == 0) {
    return s.substr(del.length());
  }
  else {
    return s;
  }
}

std::string rstrip(const std::string& s, const std::string& del)
{
  if (s.size() >= del.size() && s.compare(s.size() - del.size(), del.size(), del) == 0) {
    return s.substr(0, s.size() - del.size());
  }
  else {
    return s;
  }
}

std::string stripQuates(const std::string& s)
{
  if (s.front() == '"' && s.back() == '"') {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

std::string trim(const std::string& s)
{
  // Return as-is for an empty string. Otherwise, `find_(first,last)_not_of` would return `inf`.
  if (s.empty()) {
    return {};
  }

  static constexpr char del[] = " \t\n\r\f\v";
  const auto first = s.find_first_not_of(del);
  const auto last = s.find_last_not_of(del);

  return s.substr(first, last - first + 1);
}

std::vector<std::string> splitLines(const std::string& text)
{
  std::vector<std::string> lines;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    lines.push_back(line);
  }
  return lines;
}

std::string deleteNl(const std::string& s)
{
  std::string res;
  for (const auto& ch : s) {
    if (ch != '\r' && ch != '\n') {
      res += ch;
    }
  }
  return res;
}

std::string toLower(std::string arg)
{
  transform(arg.begin(), arg.end(), arg.begin(), [](char c) { return std::tolower(c); });
  return arg;
}

std::string toUpper(std::string arg)
{
  transform(arg.begin(), arg.end(), arg.begin(), [](char c) { return std::toupper(c); });
  return arg;
}

std::string replace(std::string s, const std::string& from, const std::string& to)
{
  size_t start_pos = 0;
  while ((start_pos = s.find(from, start_pos)) != std::string::npos) {
    s.replace(start_pos, from.length(), to);
    start_pos += to.length();  // Set the next search position.
  }
  return s;
}

std::string sanitize(const char* s)
{
  if (!s) {
    return {};
  }

  std::string out(s);

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
  out.erase(remove_if(out.begin(), out.end(), [](char ch) { return (ch < 0x20 || ch == 0x7F); }), out.end());

  // Collapse consecutive spaces to one.
  out.erase(unique(out.begin(), out.end(), [](char a, char b) { return a == ' ' && b == ' '; }), out.end());

  // Trim leading and trailing spaces.
  const auto notspace = [](char c) { return c != ' '; };
  const auto first = find_if(out.begin(), out.end(), notspace);
  if (first == out.end()) {
    return {};  // All spaces.
  }
  const auto last = find_if(out.rbegin(), out.rend(), notspace).base();
  return std::string(first, last);
}

bool contains(const std::string& s, const std::string& sub)
{
  return s.find(sub) != std::string::npos;
}

bool contains(const std::string& s, const char& sub)
{
  return s.find(sub) != std::string::npos;
}

bool isValidFileName(const std::string& file_name)
{
  if (file_name.empty()) {
    return false;
  }

  std::regex invalid_chars(R"([<>:\"/\\|?*])");
  return !std::regex_search(file_name, invalid_chars);
}

bool isValidEmail(const std::string& email)
{
  const std::regex pattern(R"(\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,7}\b)");
  return std::regex_match(email, pattern);
}

std::string convertToSuperscript(const std::string& input)
{
  // Superscript mapping table.
  const std::map<std::string, std::string> superscripts = {
    { "^0", "⁰" }, { "^1", "¹" }, { "^2", "²" }, { "^3", "³" }, { "^4", "⁴" },
    { "^5", "⁵" }, { "^6", "⁶" }, { "^7", "⁷" }, { "^8", "⁸" }, { "^9", "⁹" },
  };

  std::string output;
  size_t pos = 0;

  while (pos < input.size()) {
    if (input[pos] == '^' && pos + 1 < input.size() && std::isdigit(input[pos + 1])) {
      const auto key = input.substr(pos, 2);  // Get "^N".
      const auto it = superscripts.find(key);
      if (it != superscripts.end()) {
        output += it->second;  // Convert to superscript.
        pos += 2;
        continue;
      }
    }
    output += input[pos];
    ++pos;
  }

  return output;
}

std::string pascalFromTitle(const std::string& title_case)
{
  return regex_replace(title_case, std::regex(" "), "");
}

std::string pascalFromSnake(const std::string& snake_case)
{
  std::stringstream result;
  std::stringstream ss(snake_case);
  std::string item;
  while (std::getline(ss, item, '_')) {
    item[0] = std::toupper(item[0]);
    result << item;
  }
  return result.str();
}

std::string titleFromSnake(const std::string& snake_case)
{
  std::stringstream result;
  std::stringstream ss(snake_case);
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

std::string snakeFromPascal(const std::string& pascal_case)
{
  std::stringstream result;
  for (size_t i = 0; i < pascal_case.size(); ++i) {
    if (isupper(pascal_case[i]) && i > 0) {
      result << '_';
    }
    result << static_cast<char>(std::tolower(pascal_case[i]));
  }
  return result.str();
}

std::string snakeFromTitle(const std::string& title_case)
{
  return snakeFromPascal(pascalFromTitle(title_case));
}
}  // namespace str
}  // namespace tobas
