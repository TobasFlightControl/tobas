// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string valid_email = "example@example.com";
  const string invalid_email = "invalid@.com";

  cout << boolalpha;
  cout << valid_email << ": " << tobas::str::isValidEmail(valid_email) << endl;
  cout << invalid_email << ": " << tobas::str::isValidEmail(invalid_email) << endl;
  cout << noboolalpha;
}
