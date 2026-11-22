// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <string>

#include <tobas_string_tools/core.hpp>

int main()
{
  const std::string input = "This is a test. Testing is fun.";
  const auto output = tobas::str::replace(input, "is", "was");

  std::cout << "Input : " << input << '\n';
  std::cout << "Output: " << output << '\n';
}
