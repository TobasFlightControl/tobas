// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include <tobas_ic_drivers/bmm150.hpp>

using namespace std;

int main(int _argc, char** _argv)
{
  if (_argc != 2) {
    cerr << "Usage: " << _argv[0] << " <I2C Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::driver::BMM150 mag;
  double mx, my, mz;

  if (!mag.initialize(device)) {
    cerr << "Failed to initialize magnetometer." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!mag.readMag(mx, my, mz)) {
      cerr << "Failed to read magnetic field." << endl;
      return EXIT_FAILURE;
    }

    cout << "Magnetic Field [μT]: " << mx << ", " << my << ", " << mz << endl;

    this_thread::sleep_for(1s);
  }

  return EXIT_SUCCESS;
}
