// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cmath>
#include <iostream>

#include <tobas_fc2xx_core/pwm_batt_imu.hpp>
#include <tobas_ic_drivers/stmicro/iis2mdc.hpp>
#include <tobas_ic_drivers/stmicro/ilps22qs.hpp>
#include <tobas_ic_drivers/ublox/zed_f9p.hpp>
#include <tobas_math/linalg.hpp>
#include <tobas_sbus_driver/sbus.hpp>
#include <tobas_std_tools/ansi_text_styles.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_time_tools/rate.hpp>

using namespace std;
using namespace std::chrono_literals;
namespace ch = std::chrono;

bool testBattImu()
{
  tobas::fc2xx::PwmBattImu driver;

  if (!driver.initialize()) {
    cerr << "Failed to initialize ADC." << endl;
    return EXIT_FAILURE;
  }

  this_thread::sleep_for(200ms);

  double volt, curr;
  double ax, ay, az;
  double gx, gy, gz;
  double dgx, dgy, dgz;
  tobas::tim::Rate rate(5ms);

  for (int _ = 0; _ < 200; ++_) {
    if (!driver.transfer()) {
      cerr << "Failed to communicate with the MCU." << endl;
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

    if (volt < 5.0 || 50.0 < volt) {
      cerr << "Abnormal voltage detected." << endl;
      return false;
    }
    if (curr <= 0.0 || 10.0 < curr) {
      cerr << "Abnormal current detected." << endl;
      return false;
    }
    if (tobas::math::norm(ax, ay, az - tobas::st::kGravity) > 1.0) {
      cerr << "Abnormal accel detected." << endl;
      return false;
    }
    if (tobas::math::norm(gx, gy, gz) > 0.3) {
      cerr << "Abnormal gyro detected." << endl;
      return false;
    }

    rate.sleep();
  }

  return true;
}

bool testMagnetometer()
{
  tobas::stm::IIS2MDC mag;

  if (!mag.initialize("/dev/i2c-1")) {
    cerr << "Failed to initialize magnetometer." << endl;
    return false;
  }

  this_thread::sleep_for(200ms);

  double mx, my, mz;
  tobas::tim::Rate rate(20ms);

  for (int _ = 0; _ < 50; ++_) {
    if (!mag.readMag(mx, my, mz)) {
      cerr << "Failed to read magnetic field." << endl;
      return false;
    }

    cout << "Magnetic Field [gauss]: " << mx << ", " << my << ", " << mz << endl;

    if (tobas::math::norm(mx, my, mz) > 1.5) {  // Allow up to 3 times the standard magnetic field strength (~0.5).
      cerr << "Abnormal magnetic field detected." << endl;
      return false;
    }

    rate.sleep();
  }

  return true;
}

bool testBarometer()
{
  tobas::stm::ILPS22QS baro;

  if (!baro.initialize("/dev/i2c-1")) {
    cerr << "Failed to initialize barometer." << endl;
    return false;
  }

  this_thread::sleep_for(200ms);

  double pres, temp;
  tobas::tim::Rate rate(20ms);

  for (int _ = 0; _ < 50; ++_) {
    if (!baro.readPressure(pres)) {
      cerr << "Failed to read pressure." << endl;
      return false;
    }
    if (!baro.readTemperature(temp)) {
      cerr << "Failed to read temperature." << endl;
      return false;
    }

    const auto pres_hpa = pres / 100.0;

    cout << "Pressure [hPa]    : " << pres_hpa << endl;
    cout << "Temperature [degC]: " << temp << endl;

    if (pres_hpa < 900.0 || 1100.0 < pres_hpa) {
      cerr << "Abnormal air pressure detected." << endl;
      return false;
    }
    if (temp < 0.0 || 80.0 < temp) {
      cerr << "Abnormal temperature detected." << endl;
      return false;
    }

    rate.sleep();
  }

  return true;
}

bool testGnssReceiver()
{
  tobas::ublox::ZEDF9P gnss;

  if (!gnss.initialize("/dev/spidev1.0")) {
    cerr << "Failed to initialize GNSS driver." << endl;
    return false;
  }

  if (!gnss.configureMeasurementRate(1000)) {
    cerr << "Failed to configure measurement rate." << endl;
    return false;
  }

  // Enable GNSS
  if (!gnss.enableGps()) {
    cerr << "Failed to enable GPS." << endl;
    return false;
  }
  if (!gnss.enableSbas()) {
    cerr << "Failed to enable SBAS." << endl;
    return false;
  }
  if (!gnss.enableQzss()) {
    cerr << "Failed to enable QZSS." << endl;
    return false;
  }

  // Enable messages
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_PVT, true)) {
    cerr << "Failed to enable NAV_PVT message." << endl;
    return false;
  }

  const auto start_time = ch::steady_clock::now();

  while (ch::steady_clock::now() - start_time < 3s) {
    if (!gnss.update(false)) {
      cerr << "Failed to update GNSS driver." << endl;
      return false;
    }

    if (gnss.latestClass() != tobas::ublox::ZEDF9P::CLASS_NAV) {
      continue;
    }

    switch (gnss.latestId()) {
      case tobas::ublox::ZEDF9P::NAV_PVT:
        return true;
      default:
        continue;
    }
  }

  cerr << "Timeout before receiving the first GPS message." << endl;
  return false;
}

bool testSbus()
{
  bool sbus_received = false;

  tobas::SBUS sbus([&sbus_received](const tobas::SBUS::Packet&) { sbus_received = true; });

  if (!sbus.initialize("/dev/ttyAMA0")) {
    cerr << "Failed to initialize S.BUS driver." << endl;
    return false;
  }

  sbus.start();

  const auto start_time = ch::steady_clock::now();
  while (ch::steady_clock::now() - start_time < 1min) {
    if (sbus_received) {
      return true;
    }
    this_thread::sleep_for(10ms);
  }

  cerr << "Timeout before the first S.BUS package." << endl;
  return false;
}

int main()
{
  // PM & IMU
  cout << "Testing PM & IMU..." << endl;
  if (!testBattImu()) {
    cerr << "PM & IMU test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "PM & IMU test passed." << endl;

  // Magnetometer
  cout << "Testing magnetometer..." << endl;
  if (!testMagnetometer()) {
    cerr << "Magnetometer test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "Magnetometer test passed." << endl;

  // Barometer
  cout << "Testing baro..." << endl;
  if (!testBarometer()) {
    cerr << "Barometer test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "Barometer test passed." << endl;

  // GNSS Receiver
  cout << "Testing GNSS receiver..." << endl;
  if (!testGnssReceiver()) {
    cerr << "GNSS receiver test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "GNSS receiver test passed." << endl;

  // S.BUS
  cout << "Testing S.BUS..." << endl;
  if (!testSbus()) {
    cerr << "S.BUS test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "S.BUS test passed." << endl;

  cout << GREEN_PREFIX << "All tests passed." << COLOR_RESET << endl;
  return EXIT_SUCCESS;
}
