// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <string>

#include <tobas_string_tools/core.hpp>

int main()
{
  const std::string vcgencmd_out = "frequency(48)=1500345728\n";
  const auto freq_str = tobas::str::deleteNl(tobas::str::split(vcgencmd_out, '=').back());
  const auto freq = std::stoul(freq_str);
  std::cout << "CPU frequency: " << freq << " [Hz]\n";
}
