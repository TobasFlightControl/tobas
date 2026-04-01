// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_path_tools/join.hpp>

using namespace std;

int main()
{
  cout << "join(\"hoge\") = " << tobas::path::join("hoge") << endl;
  cout << "join(\"hoge\", \"fuga\") = " << tobas::path::join("hoge", "fuga") << endl;
  cout << "join(\"hoge\", \"fuga\", \"piyo\") = " << tobas::path::join("hoge", "fuga", "piyo") << endl;
  cout << "join(\"hoge/\", \"/fuga\", \"piyo/\") = " << tobas::path::join("hoge/", "/fuga", "piyo/") << endl;
}
