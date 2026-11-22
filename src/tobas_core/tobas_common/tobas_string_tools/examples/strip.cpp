// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <string>

#include <tobas_string_tools/core.hpp>

int main()
{
  const std::string str = "hoge_fuga_hoge_fuga";
  std::cout << "Original      : " << str << '\n';
  std::cout << "lstrip('hoge'): " << tobas::str::lstrip(str, "hoge") << '\n';
  std::cout << "rstrip('fuga'): " << tobas::str::rstrip(str, "fuga") << '\n';
  std::cout << "rstrip('piyo'): " << tobas::str::rstrip(str, "piyo") << '\n';
}
