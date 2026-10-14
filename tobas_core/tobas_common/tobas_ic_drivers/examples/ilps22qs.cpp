// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_ic_drivers/stmicro/ilps22qs.hpp"

using namespace std;

int main(int _argc, char** _argv)
{
  if (_argc != 2) {
    cerr << "Usage: " << _argv[0] << " <I2C Device>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::stm::ILPS22QS baro;
  double pres, temp;

  if (!baro.initialize(device)) {
    cerr << "Failed to initialize barometer.\n";
    return EXIT_FAILURE;
  }

  while (true) {
    if (!baro.readPressure(pres)) {
      cerr << "Failed to read pressure.\n";
      continue;
    }

    if (!baro.readTemperature(temp)) {
      cerr << "Failed to read temperature.\n";
      continue;
    }

    cout << "Pressure [hPa]     : " << pres / 100.0 << '\n';
    cout << "Temperature [degC]: " << temp << '\n';

    this_thread::sleep_for(1s);
  }

  return EXIT_SUCCESS;
}
