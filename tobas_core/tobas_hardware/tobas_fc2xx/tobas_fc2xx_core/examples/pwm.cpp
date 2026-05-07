// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_fc2xx_core/pwm_batt_imu.hpp"

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 3) {
    cerr << "Usage: " << argv[0] << " <Channel> <Period>" << endl;
    return EXIT_FAILURE;
  }
  const size_t channel = stoul(argv[1]);
  const uint16_t period = stoi(argv[2]);

  tobas::fc2xx::PwmBattImu driver;

  if (!driver.initialize()) {
    cerr << "Failed to initialize PWM driver." << endl;
    return EXIT_FAILURE;
  }

  uint16_t periods[tobas::fc2xx::PwmBattImu::kPwmChannels] = {};
  periods[channel] = period;

  cout << "Commanding the PWM period..." << endl;
  while (true) {
    driver.setPwmPeriod(periods);

    if (!driver.transfer()) {
      cerr << "Failed to communicate with the micro controller." << endl;
      continue;
    }

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
