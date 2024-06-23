#include <iostream>

#include <tobas_path_tools/join.hpp>

using namespace std;

int main()
{
  cout << "join(\"hoge\") = " << path::join("hoge") << endl;
  cout << "join(\"hoge\", \"fuga\") = " << path::join("hoge", "fuga") << endl;
  cout << "join(\"hoge\", \"fuga\", \"piyo\") = " << path::join("hoge", "fuga", "piyo") << endl;
  cout << "join(\"hoge/\", \"/fuga\", \"piyo/\") = " << path::join("hoge/", "/fuga", "piyo/") << endl;
}
