// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string input = "example/directory/file.txt";
  const auto output = tobas::str::rsplit(input, '/');

  cout << "Input : " << input << endl;
  cout << "Output: " << output.first << ", " << output.second << endl;
}
