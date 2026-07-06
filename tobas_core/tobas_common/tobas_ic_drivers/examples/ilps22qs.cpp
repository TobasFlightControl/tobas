// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_ic_drivers/stmicro/ilps22qs.hpp"

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <I2C Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];

  tobas::stm::ILPS22QS baro;
  double pres, temp;

  if (!baro.initialize(device)) {
    cerr << "Failed to initialize barometer." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!baro.readPressure(pres)) {
      cerr << "Failed to read pressure." << endl;
      continue;
    }

    if (!baro.readTemperature(temp)) {
      cerr << "Failed to read temperature." << endl;
      continue;
    }

    cout << "Pressure [hPa]     : " << pres / 100.0 << endl;
    cout << "Temperature [degC]: " << temp << endl;

    this_thread::sleep_for(1s);
  }

  return EXIT_SUCCESS;
}
