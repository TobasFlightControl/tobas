#include "tobas_string_tools/core.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <map>
#include <regex>

using namespace std;

namespace str
{
vector<string> split(const string& s, const char& c)
{
  vector<string> res;
  if (s.find(c) == string::npos) {
    res.push_back(s);
  }
  else {
    size_t first = 0;
    size_t last = s.find_first_of(c);
    while (first < s.size()) {
      string subStr(s, first, last - first);
      res.push_back(subStr);
      first = last + 1;
      last = s.find_first_of(c, first);
      if (last == string::npos) {
        last = s.size();
      }
    }
  }
  return res;
}

pair<string, string> rsplit(const string& s, const char& c)
{
  // 最後の'/'を探す
  size_t pos = s.rfind(c);

  if (pos != string::npos) {
    string before = s.substr(0, pos);
    string after = s.substr(pos + 1);
    return { before, after };
  }
  else {
    cerr << "String \"" << s << "\" does not contain '" << c << "'" << endl;
    return { s, "" };
  }
}

string lstrip(const string& s, const string& del)
{
  if (s.find(del) == 0) {
    return s.substr(del.length());
  }
  else {
    return s;
  }
}

string rstrip(const string& s, const string& del)
{
  if (s.size() >= del.size() && s.compare(s.size() - del.size(), del.size(), del) == 0) {
    return s.substr(0, s.size() - del.size());
  }
  else {
    return s;
  }
}

string stripQuates(const string& s)
{
  if (s.front() == '"' && s.back() == '"') {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

string trim(const string& s)
{
  // 空文字のときはそのまま返す．でないとfind_(first, last)_not_ofがinfを返してしまう．
  if (s.empty()) {
    return {};
  }

  static constexpr char del[] = " \t\n\r\f\v";
  const auto first = s.find_first_not_of(del);
  const auto last = s.find_last_not_of(del);

  return s.substr(first, last - first + 1);
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

string deleteNl(const string& s)
{
  string res;
  for (const auto& ch : s) {
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

string replace(string s, const string& from, const string& to)
{
  size_t start_pos = 0;
  while ((start_pos = s.find(from, start_pos)) != string::npos) {
    s.replace(start_pos, from.length(), to);
    start_pos += to.length();  // 次の検索位置を設定
  }
  return s;
}

string sanitize(const char* s)
{
  if (!s) {
    return {};
  }

  string out(s);

  // 改行・タブ類をスペースに
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

  // 他の制御文字 (0x00-0x1F, 0x7F) を削除
  out.erase(remove_if(out.begin(), out.end(), [](uint8_t ch) { return (ch < 0x20 || ch == 0x7F); }), out.end());

  // 連続スペースを1つに
  out.erase(unique(out.begin(), out.end(), [](char a, char b) { return a == ' ' && b == ' '; }), out.end());

  // 前後のスペースをトリム
  auto notspace = [](uint8_t c) { return c != ' '; };
  auto first = find_if(out.begin(), out.end(), notspace);
  if (first == out.end()) {
    return {};  // 全部スペース
  }
  auto last = find_if(out.rbegin(), out.rend(), notspace).base();
  return string(first, last);
}

bool contains(const string& s, const string& sub)
{
  return s.find(sub) != string::npos;
}

bool contains(const string& s, const char& sub)
{
  return s.find(sub) != string::npos;
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
