// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_fc2xx_core/pwm_batt_imu.hpp"

using namespace std;

int main()
{
  tobas::fc2xx::PwmBattImu driver;

  if (!driver.initialize()) {
    cerr << "Failed to initialize the driver.\n";
    return EXIT_FAILURE;
  }

  double volt, curr;
  double ax, ay, az;
  double gx, gy, gz;
  double dgx, dgy, dgz;

  while (true) {
    if (const auto result = driver.transfer(); !result) {
      cerr << "Failed to communicate with the MCU: " << result.error() << '\n';
      continue;
    }

    driver.getBattVoltage(volt);
    driver.getBattCurrent(curr);
    driver.getRawAccel(ax, ay, az);
    driver.getRawGyro(gx, gy, gz);
    driver.getRawDGyro(dgx, dgy, dgz);

    cout << "-----\n";
    cout << "Voltage [V]     : " << volt << '\n';
    cout << "Current [A]     : " << curr << '\n';
    cout << "Accel [m/s^2]   : " << ax << ", " << ay << ", " << az << '\n';
    cout << "Gyro [rad/s]    : " << gx << ", " << gy << ", " << gz << '\n';
    cout << "D-Gyro [rad/s^2]: " << dgx << ", " << dgy << ", " << dgz << '\n';

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
