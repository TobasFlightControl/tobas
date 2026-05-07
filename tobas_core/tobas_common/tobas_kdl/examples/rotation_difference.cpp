// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_kdl/euler.hpp>

using namespace std;

int main()
{
  const tobas::kdl::Euler O_Rot_A(M_PI_2, 0, 0);
  const tobas::kdl::Euler O_Rot_B(M_PI_2, 0, M_PI_2);
  const tobas::kdl::AngleAxis O_AngleAxis_AB = O_Rot_B - O_Rot_A;
  cout << "Angle-Axis wrt. O: " << O_AngleAxis_AB << endl;
}
