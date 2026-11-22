// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <ios>
#include <iostream>
#include <string>

#include <tobas_string_tools/core.hpp>

int main()
{
  const std::string valid_email = "example@example.com";
  const std::string invalid_email = "invalid@.com";

  std::cout << std::boolalpha;
  std::cout << valid_email << ": " << tobas::str::isValidEmail(valid_email) << '\n';
  std::cout << invalid_email << ": " << tobas::str::isValidEmail(invalid_email) << '\n';
  std::cout << std::noboolalpha;
}
