// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string input = "This is a test: x^2 + y^3 = z^4.";
  const string output = tobas::str::convertToSuperscript(input);

  cout << "Input : " << input << endl;
  cout << "Output: " << output << endl;
}
