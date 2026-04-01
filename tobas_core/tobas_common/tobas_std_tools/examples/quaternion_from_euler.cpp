// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 4) {
    cerr << "Usage: " << argv[0] << " <Roll> <Pitch> <Yaw> [degree]" << endl;
    return EXIT_FAILURE;
  }

  const auto roll = tobas::st::deg2rad(atof(argv[1]));
  const auto pitch = tobas::st::deg2rad(atof(argv[2]));
  const auto yaw = tobas::st::deg2rad(atof(argv[3]));

  const auto [qx, qy, qz, qw] = tobas::st::quaternionFromEuler(roll, pitch, yaw);
  cout << "Hamilton: " << qw << ", " << qx << ", " << qy << ", " << qz << endl;

  return EXIT_SUCCESS;
}
