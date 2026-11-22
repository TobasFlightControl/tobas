// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_path_tools/join.hpp>

int main()
{
  std::cout << "join('hoge') = " << tobas::path::join("hoge") << '\n';
  std::cout << "join('hoge', 'fuga') = " << tobas::path::join("hoge", "fuga") << '\n';
  std::cout << "join('hoge', 'fuga', 'piyo') = " << tobas::path::join("hoge", "fuga", "piyo") << '\n';
  std::cout << "join('hoge/', '/fuga', 'piyo/') = " << tobas::path::join("hoge/", "/fuga", "piyo/") << '\n';
}
