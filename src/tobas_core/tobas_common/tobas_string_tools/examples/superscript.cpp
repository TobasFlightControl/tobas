// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <string>

#include <tobas_string_tools/core.hpp>

int main()
{
  const std::string input = "This is a test: x^2 + y^3 = z^4.";
  const std::string output = tobas::str::convertToSuperscript(input);

  std::cout << "Input : " << input << '\n';
  std::cout << "Output: " << output << '\n';
}
