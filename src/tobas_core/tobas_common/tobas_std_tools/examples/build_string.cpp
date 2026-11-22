// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_std_tools/stream.hpp>

int main()
{
  const auto result = tobas::st::buildString("Number: ", 42, " and ", 3.14);
  std::cout << result << '\n';
}
