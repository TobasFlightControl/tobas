// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <string>

#include <tobas_string_tools/core.hpp>

int main()
{
  const std::string input = "example/directory/file.txt";
  const auto output = tobas::str::rsplit(input, '/');

  if (!output) {
    std::cerr << output.error() << '\n';
    return 1;
  }

  std::cout << "Input : " << input << '\n';
  std::cout << "Output: " << output->first << ", " << output->second << '\n';
}
