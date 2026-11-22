// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>

#include <tobas_linux/execute_command.hpp>

int main()
{
  const char* cmd = "date";
  const auto result = tobas::linux::executeCommand(cmd);
  if (!result) {
    std::cerr << "Command failed: " << result.error() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "Command: " << cmd << '\n';
  std::cout << "Result : " << *result << '\n';

  return EXIT_SUCCESS;
}
