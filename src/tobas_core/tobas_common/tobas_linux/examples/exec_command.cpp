// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_linux/execute_command.hpp>

using namespace std;

int main()
{
  const char* cmd = "date";
  const auto result = tobas::linux::executeCommand(cmd);
  if (!result) {
    cerr << "Command failed: " << result.error() << '\n';
    return EXIT_FAILURE;
  }

  cout << "Command: " << cmd << '\n';
  cout << "Result : " << *result << '\n';

  return EXIT_SUCCESS;
}
