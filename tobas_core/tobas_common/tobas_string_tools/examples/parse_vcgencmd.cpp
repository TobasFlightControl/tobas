// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_string_tools/core.hpp>

using namespace std;

int main()
{
  const string vcgencmd_out = "frequency(48)=1500345728\n";
  const auto freq_str = tobas::str::deleteNl(tobas::str::split(vcgencmd_out, '=').back());
  const auto freq = stoul(freq_str);
  cout << "CPU frequency: " << freq << " [Hz]" << endl;
}
