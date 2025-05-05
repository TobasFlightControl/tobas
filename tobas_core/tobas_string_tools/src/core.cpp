#include "../include/tobas_string_tools/core.hpp"

#include <cstdint>
#include <algorithm>
#include <map>
#include <regex>
#include <iostream>

using namespace std;

namespace str
{
vector<string> split(const string& str, const char& del)
{
  vector<string> res;
  if (str.find(del) == string::npos) {
    res.push_back(str);
  }
  else {
    size_t first = 0;
    size_t last = str.find_first_of(del);
    while (first < str.size()) {
      string subStr(str, first, last - first);
      res.push_back(subStr);
      first = last + 1;
      last = str.find_first_of(del, first);
      if (last == string::npos) {
        last = str.size();
      }
    }
  }
  return res;
}

pair<string, string> rsplit(const string& str, const char& c)
{
  // 最後の'/'を探す
  size_t pos = str.rfind(c);

  if (pos != string::npos) {
    string before = str.substr(0, pos);
    string after = str.substr(pos + 1);
    return { before, after };
  }
  else {
    cerr << "String \"" << str << "\" does not contain '" << c << "'" << endl;
    return { str, "" };
  }
}

string lstrip(const string& str, const string& del)
{
  if (str.find(del) == 0) {
    return str.substr(del.length());
  }
  else {
    return str;
  }
}

string rstrip(const string& str, const string& del)
{
  if (str.size() >= del.size() && str.compare(str.size() - del.size(), del.size(), del) == 0) {
    return str.substr(0, str.size() - del.size());
  }
  else {
    return str;
  }
}

string stripQuates(const string& str)
{
  if (str.front() == '"' && str.back() == '"') {
    return str.substr(1, str.size() - 2);
  }
  return str;
}

string trim(const string& str)
{
  // 空文字のときはそのまま返す．でないとfind_(first, last)_not_ofがinfを返してしまう．
  if (str == "") {
    return "";
  }

  static constexpr char del[] = " \t\n\r\f\v";
  const auto first = str.find_first_not_of(del);
  const auto last = str.find_last_not_of(del);

  return str.substr(first, last - first + 1);
}

vector<string> splitLines(const string& text)
{
  vector<string> lines;
  istringstream stream(text);
  string line;
  while (getline(stream, line)) {
    lines.push_back(line);
  }
  return lines;
}

string deleteNl(const string& str)
{
  string res;
  for (const auto& ch : str) {
    if (ch != '\r' && ch != '\n') {
      res += ch;
    }
  }
  return res;
}

string toLower(string arg)
{
  transform(arg.begin(), arg.end(), arg.begin(), [](uint8_t c) { return tolower(c); });
  return arg;
}

string toUpper(string arg)
{
  transform(arg.begin(), arg.end(), arg.begin(), [](uint8_t c) { return toupper(c); });
  return arg;
}

string replace(string str, const string& from, const string& to)
{
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();  // 次の検索位置を設定
  }
  return str;
}

bool contains(const string& str, const string& sub)
{
  return str.find(sub) != string::npos;
}

bool contains(const string& str, const char& sub)
{
  return str.find(sub) != string::npos;
}

bool startsWith(const string& str, const string& prefix)
{
  if (prefix.size() > str.size()) {
    return false;
  }
  return std::equal(prefix.begin(), prefix.end(), str.begin());
}

bool endsWith(const string& str, const string& suffix)
{
  if (suffix.size() > str.size()) {
    return false;
  }
  return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
}

bool isValidFileName(const string& file_name)
{
  if (file_name.empty()) {
    return false;
  }

  regex invalid_chars(R"([<>:\"/\\|?*])");
  return !regex_search(file_name, invalid_chars);
}

bool isValidEmail(const string& email)
{
  const regex pattern(R"(\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,7}\b)");
  return regex_match(email, pattern);
}

string convertToSuperscript(const string& input)
{
  // 上付き文字の対応表
  const map<string, string> superscripts = {
    { "^0", "⁰" }, { "^1", "¹" }, { "^2", "²" }, { "^3", "³" }, { "^4", "⁴" },
    { "^5", "⁵" }, { "^6", "⁶" }, { "^7", "⁷" }, { "^8", "⁸" }, { "^9", "⁹" },
  };

  string output;
  size_t pos = 0;

  while (pos < input.size()) {
    if (input[pos] == '^' && pos + 1 < input.size() && isdigit(input[pos + 1])) {
      const auto key = input.substr(pos, 2);  // "^N" を取得
      if (superscripts.find(key) != superscripts.end()) {
        output += superscripts.at(key);  // 上付き文字に変換
        pos += 2;
        continue;
      }
    }
    output += input[pos];
    ++pos;
  }

  return output;
}

string pascalFromTitle(const string& title_case)
{
  return regex_replace(title_case, regex(" "), "");
}

string pascalFromSnake(const string& snake_case)
{
  stringstream result;
  stringstream ss(snake_case);
  string item;
  while (getline(ss, item, '_')) {
    item[0] = toupper(item[0]);
    result << item;
  }
  return result.str();
}

string titleFromSnake(const string& snake_case)
{
  stringstream result;
  stringstream ss(snake_case);
  string item;
  bool first = true;
  while (getline(ss, item, '_')) {
    item[0] = toupper(item[0]);
    if (!first) {
      result << " ";
    }
    result << item;
    first = false;
  }
  return result.str();
}

string snakeFromPascal(const string& pascal_case)
{
  stringstream result;
  for (size_t i = 0; i < pascal_case.size(); ++i) {
    if (isupper(pascal_case[i]) && i > 0) {
      result << "_";
    }
    result << static_cast<char>(tolower(pascal_case[i]));
  }
  return result.str();
}

string snakeFromTitle(const string& title_case)
{
  return snakeFromPascal(pascalFromTitle(title_case));
}
}  // namespace str
