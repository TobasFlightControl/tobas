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
    cerr << "Failed to initialize the driver." << endl;
    return EXIT_FAILURE;
  }

  double volt, curr;
  double ax, ay, az;
  double gx, gy, gz;
  double dgx, dgy, dgz;

  while (true) {
    if (!driver.transfer()) {
      cerr << "Failed to communicate with the micro controller." << endl;
      continue;
    }

    driver.getBattVoltage(volt);
    driver.getBattCurrent(curr);
    driver.getRawAccel(ax, ay, az);
    driver.getRawGyro(gx, gy, gz);
    driver.getRawDGyro(dgx, dgy, dgz);

    cout << "-----" << endl;
    cout << "Voltage [V]     : " << volt << endl;
    cout << "Current [A]     : " << curr << endl;
    cout << "Accel [m/s^2]   : " << ax << ", " << ay << ", " << az << endl;
    cout << "Gyro [rad/s]    : " << gx << ", " << gy << ", " << gz << endl;
    cout << "D-Gyro [rad/s^2]: " << dgx << ", " << dgy << ", " << dgz << endl;

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
