// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstddef>
#include <iostream>
#include <vector>

#include <tobas_std_tools/vector.hpp>

int main()
{
  for (const auto n : { 1000000.0f, 10000000.0f, 100000000.0f, 1000000000.0f }) {
    std::vector<float> values(n, 1.0);
    const auto naive_sum = tobas::st::sum(values);
    const auto kahan_sum = tobas::st::fsum(values);
    std::cout << "n = " << n << "\t: Naive Summation = " << naive_sum << ", Kahan Summation = " << kahan_sum << '\n';
  }
}
