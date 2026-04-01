// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_std_tools/stream.hpp>

using namespace std;

int main()
{
  const auto result = tobas::st::buildString("Number: ", 42, " and ", 3.14);
  cout << result << endl;
}
