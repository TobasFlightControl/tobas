// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <tobas_mcu_driver/tobas_t2/pwm_batt_imu.hpp>

using namespace std::chrono_literals;

int main(int _argc, char* _argv[])
{
  if (_argc != 2) {
    std::cerr << "Usage: " << _argv[0] << " <SPI Device>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::t2::PwmBattImu driver;

  if (!driver.initialize(device)) {
    std::cerr << "Failed to initialize the driver.\n";
    return EXIT_FAILURE;
  }

  double volt, curr;
  double ax, ay, az;
  double gx, gy, gz;
  double dgx, dgy, dgz;

  while (true) {
    if (const auto result = driver.transfer(); !result) {
      std::cerr << "Failed to communicate with the MCU: " << result.error() << '\n';
      continue;
    }

    driver.getBattVoltage(volt);
    driver.getBattCurrent(curr);
    driver.getRawAccel(ax, ay, az);
    driver.getRawGyro(gx, gy, gz);
    driver.getRawDGyro(dgx, dgy, dgz);

    std::cout << "-----\n";
    std::cout << "Voltage [V]     : " << volt << '\n';
    std::cout << "Current [A]     : " << curr << '\n';
    std::cout << "Accel [m/s^2]   : " << ax << ", " << ay << ", " << az << '\n';
    std::cout << "Gyro [rad/s]    : " << gx << ", " << gy << ", " << gz << '\n';
    std::cout << "D-Gyro [rad/s^2]: " << dgx << ", " << dgy << ", " << dgz << '\n';

    std::this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
