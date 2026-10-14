// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_ic_drivers/stmicro/iis2mdc.hpp"

using namespace std;
using namespace std::chrono_literals;

int main(int _argc, char** _argv)
{
  if (_argc != 2) {
    cerr << "Usage: " << _argv[0] << " <I2C Device>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::stm::IIS2MDC mag;
  double mx, my, mz;

  if (!mag.initialize(device)) {
    cerr << "Failed to initialize magnetometer.\n";
    return EXIT_FAILURE;
  }

  while (true) {
    if (!mag.readMag(mx, my, mz)) {
      cerr << "Failed to read magnetic field.\n";
      continue;
    }

    cout << "Magnetic Field [gauss]: " << mx << ", " << my << ", " << mz << '\n';

    this_thread::sleep_for(1s);
  }

  return EXIT_SUCCESS;
}
