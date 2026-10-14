// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/battery.hpp"

using namespace std;

int main()
{
  tobas::fc1xx::Battery battery;
  float voltage, current;

  if (!battery.initialize()) {
    cerr << "Failed to initialize ADC.\n";
    return EXIT_FAILURE;
  }

  while (true) {
    if (!battery.read(voltage, current)) {
      cerr << "Failed to read battery status.\n";
      continue;
    }

    cout << "Voltage [V]: " << voltage << '\n';
    cout << "Current [A]: " << current << '\n';

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
