// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <tobas_ic_drivers/bmm150.hpp>

using namespace std::chrono_literals;

int main(int _argc, char* _argv[])
{
  if (_argc != 2) {
    std::cerr << "Usage: " << _argv[0] << " <I2C Device>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::driver::BMM150 mag;
  double mx, my, mz;

  if (!mag.initialize(device)) {
    std::cerr << "Failed to initialize magnetometer.\n";
    return EXIT_FAILURE;
  }

  while (true) {
    if (!mag.readMag(mx, my, mz)) {
      std::cerr << "Failed to read magnetic field.\n";
      return EXIT_FAILURE;
    }
    std::cout << "Magnetic Field [μT]: " << mx << ", " << my << ", " << mz << '\n';
    std::this_thread::sleep_for(1s);
  }

  return EXIT_SUCCESS;
}
