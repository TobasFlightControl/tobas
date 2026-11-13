// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string str = "hoge_fuga_hoge_fuga";
  cout << "Original      : " << str << '\n';
  cout << "lstrip('hoge'): " << tobas::str::lstrip(str, "hoge") << '\n';
  cout << "rstrip('fuga'): " << tobas::str::rstrip(str, "fuga") << '\n';
  cout << "rstrip('piyo'): " << tobas::str::rstrip(str, "piyo") << '\n';
}
