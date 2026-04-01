// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string input = "This is a test. Testing is fun.";
  const auto output = tobas::str::replace(input, "is", "was");

  cout << "Input : " << input << endl;
  cout << "Output: " << output << endl;
}
