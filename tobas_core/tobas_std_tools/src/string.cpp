#include <algorithm>

#include "../include/tobas_std_tools/string.hpp"

using namespace std;

namespace tobas_std
{
vector<string> split(const string& str, const char& del)
{
  vector<string> res;
  if (str.find(del) == string::npos)
  {
    res.push_back(str);
  }
  else
  {
    size_t first = 0;
    size_t last = str.find_first_of(del);
    while (first < str.size())
    {
      string subStr(str, first, last - first);
      res.push_back(subStr);
      first = last + 1;
      last = str.find_first_of(del, first);
      if (last == string::npos)
      {
        last = str.size();
      }
    }
  }
  return res;
}

string deleteNl(const string& str)
{
  string res;
  for (const auto& ch : str)
    if (ch != '\r' && ch != '\n')
      res += ch;
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

bool contains(const string& str, const string& sub)
{
  return str.find(sub) != string::npos;
}

bool contains(const string& str, const char& sub)
{
  return str.find(sub) != string::npos;
}
}  // namespace tobas_std
