// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/iis2mdc.hpp"

using namespace std;
using namespace std::chrono_literals;

int main()
{
  tobas::fc1xx::IIS2MDC mag;
  double mx, my, mz;

  if (!mag.initialize()) {
    cerr << "Failed to initialize magnetometer." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!mag.readMag(mx, my, mz)) {
      cerr << "Failed to read magnetic field." << endl;
      continue;
    }

    cout << "Magnetic Field [gauss]: " << mx << ", " << my << ", " << mz << endl;

    this_thread::sleep_for(1s);
  }

  return EXIT_SUCCESS;
}
