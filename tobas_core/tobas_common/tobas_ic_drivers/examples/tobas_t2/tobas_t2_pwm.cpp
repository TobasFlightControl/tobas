// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <tobas_ic_drivers/tobas_t2/pwm_batt_imu.hpp>

using namespace std;

int main(int _argc, char** _argv)
{
  if (_argc != 4) {
    cerr << "Usage: " << _argv[0] << " <SPI Device> <Channel> <Period>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];
  const auto channel = stoul(_argv[2]);
  const auto period = stoul(_argv[3]);

  if (channel >= tobas::t2::PwmBattImu::kPwmChannels) {
    cerr << "PWM channel is out of range.\n";
    return EXIT_FAILURE;
  }

  tobas::t2::PwmBattImu driver;

  if (!driver.initialize(device)) {
    cerr << "Failed to initialize PWM driver.\n";
    return EXIT_FAILURE;
  }

  uint16_t periods[tobas::t2::PwmBattImu::kPwmChannels] = {};
  periods[channel] = period;

  cout << "Commanding the PWM period...\n";
  while (true) {
    driver.setPwmPeriod(periods);

    if (const auto result = driver.transfer(); !result) {
      cerr << "Failed to communicate with the MCU: " << result.error() << '\n';
      continue;
    }

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
