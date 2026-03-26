#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string str = "hoge_fuga_hoge_fuga";
  cout << "Original      : " << str << endl;
  cout << "lstrip(\"hoge\"): " << tobas::str::lstrip(str, "hoge") << endl;
  cout << "rstrip(\"fuga\"): " << tobas::str::rstrip(str, "fuga") << endl;
  cout << "rstrip(\"piyo\"): " << tobas::str::rstrip(str, "piyo") << endl;
}
