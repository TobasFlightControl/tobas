// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <numbers>

#include <tobas_algorithm/core.hpp>

int main()
{
  using std::numbers::pi;

  for (const auto angle : { -4 * pi, -pi, -pi / 2, 0.0, pi / 2, pi, 3 * pi }) {
    std::cout << "Original: " << angle << ", Wrapped: " << tobas::algo::wrapPi(angle) << '\n';
  }
}
