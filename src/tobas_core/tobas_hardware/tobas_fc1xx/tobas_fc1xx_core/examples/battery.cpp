// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/battery.hpp"

using namespace std::chrono_literals;

int main()
{
  tobas::fc1xx::Battery battery;
  float voltage, current;

  if (!battery.initialize()) {
    std::cerr << "Failed to initialize ADC.\n";
    return EXIT_FAILURE;
  }

  while (true) {
    if (!battery.read(voltage, current)) {
      std::cerr << "Failed to read battery status.\n";
      continue;
    }

    std::cout << "Voltage [V]: " << voltage << '\n';
    std::cout << "Current [A]: " << current << '\n';

    std::this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
