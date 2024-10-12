#include <iostream>

#include <tobas_std_tools/string.hpp>

using namespace std;

int main()
{
  const string str = "hoge_fuga_hoge_fuga";
  cout << "Original      : " << str << endl;
  cout << "lstrip(\"hoge\"): " << tobas_std::lstrip(str, "hoge") << endl;
  cout << "rstrip(\"fuga\"): " << tobas_std::rstrip(str, "fuga") << endl;
  cout << "rstrip(\"piyo\"): " << tobas_std::rstrip(str, "piyo") << endl;
}
