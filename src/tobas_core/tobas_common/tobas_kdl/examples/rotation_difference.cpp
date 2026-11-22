// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <numbers>

#include <tobas_kdl/angle_axis.hpp>
#include <tobas_kdl/euler.hpp>

int main()
{
  using std::numbers::pi;

  const tobas::kdl::Euler o_rot_a(pi / 2.0, 0, 0);
  const tobas::kdl::Euler o_rot_b(pi / 2.0, 0, pi / 2.0);
  const tobas::kdl::AngleAxis o_angle_axis_ab = o_rot_b - o_rot_a;

  std::cout << "Angle-Axis wrt. O: " << o_angle_axis_ab << '\n';
}
