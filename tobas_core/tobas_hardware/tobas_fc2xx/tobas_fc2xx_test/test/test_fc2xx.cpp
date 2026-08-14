// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cmath>
#include <iostream>

#include <tobas_fc2xx_core/pwm_batt_imu.hpp>
#include <tobas_ic_drivers/stmicro/iis2mdc.hpp>
#include <tobas_ic_drivers/stmicro/ilps22qs.hpp>
#include <tobas_ic_drivers/ublox/zed_f9p.hpp>
#include <tobas_sbus_driver/sbus.hpp>
#include <tobas_std_tools/ansi_text_styles.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_time_tools/rate.hpp>

using namespace std::chrono_literals;
namespace ch = std::chrono;

bool testBattImu()
{
  tobas::fc2xx::PwmBattImu driver;

  if (!driver.initialize()) {
    std::cerr << "Failed to initialize ADC." << std::endl;
    return EXIT_FAILURE;
  }

  std::this_thread::sleep_for(200ms);

  double volt, curr;
  double ax, ay, az;
  double gx, gy, gz;
  double dgx, dgy, dgz;
  tobas::tim::Rate rate(5ms);

  for (int _ = 0; _ < 200; ++_) {
    if (!driver.transfer()) {
      std::cerr << "Failed to communicate with the MCU." << std::endl;
      continue;
    }

    driver.getBattVoltage(volt);
    driver.getBattCurrent(curr);
    driver.getRawAccel(ax, ay, az);
    driver.getRawGyro(gx, gy, gz);
    driver.getRawDGyro(dgx, dgy, dgz);

    std::cout << "-----" << std::endl;
    std::cout << "Voltage [V]     : " << volt << std::endl;
    std::cout << "Current [A]     : " << curr << std::endl;
    std::cout << "Accel [m/s^2]   : " << ax << ", " << ay << ", " << az << std::endl;
    std::cout << "Gyro [rad/s]    : " << gx << ", " << gy << ", " << gz << std::endl;
    std::cout << "D-Gyro [rad/s^2]: " << dgx << ", " << dgy << ", " << dgz << std::endl;

    if (volt < 5.0 || 50.0 < volt) {
      std::cerr << "Abnormal voltage detected." << std::endl;
      return false;
    }
    if (curr <= 0.0 || 10.0 < curr) {
      std::cerr << "Abnormal current detected." << std::endl;
      return false;
    }
    if (std::hypot(ax, ay, az - tobas::st::kGravity) > 1.0) {
      std::cerr << "Abnormal accel detected." << std::endl;
      return false;
    }
    if (std::hypot(gx, gy, gz) > 0.3) {
      std::cerr << "Abnormal gyro detected." << std::endl;
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
    std::cerr << "Failed to initialize magnetometer." << std::endl;
    return false;
  }

  std::this_thread::sleep_for(200ms);

  double mx, my, mz;
  tobas::tim::Rate rate(20ms);

  for (int _ = 0; _ < 50; ++_) {
    if (!mag.readMag(mx, my, mz)) {
      std::cerr << "Failed to read magnetic field." << std::endl;
      return false;
    }

    std::cout << "Magnetic Field [gauss]: " << mx << ", " << my << ", " << mz << std::endl;

    if (std::hypot(mx, my, mz) > 1.5) {  // Allow up to 3 times the standard magnetic field strength (~0.5).
      std::cerr << "Abnormal magnetic field detected." << std::endl;
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
    std::cerr << "Failed to initialize barometer." << std::endl;
    return false;
  }

  std::this_thread::sleep_for(200ms);

  double pres, temp;
  tobas::tim::Rate rate(20ms);

  for (int _ = 0; _ < 50; ++_) {
    if (!baro.readPressure(pres)) {
      std::cerr << "Failed to read pressure." << std::endl;
      return false;
    }
    if (!baro.readTemperature(temp)) {
      std::cerr << "Failed to read temperature." << std::endl;
      return false;
    }

    const auto pres_hpa = pres / 100.0;

    std::cout << "Pressure [hPa]    : " << pres_hpa << std::endl;
    std::cout << "Temperature [degC]: " << temp << std::endl;

    if (pres_hpa < 900.0 || 1100.0 < pres_hpa) {
      std::cerr << "Abnormal air pressure detected." << std::endl;
      return false;
    }
    if (temp < 0.0 || 80.0 < temp) {
      std::cerr << "Abnormal temperature detected." << std::endl;
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
    std::cerr << "Failed to initialize GNSS driver." << std::endl;
    return false;
  }

  if (!gnss.configureMeasurementRate(1000)) {
    std::cerr << "Failed to configure measurement rate." << std::endl;
    return false;
  }

  // Enable GNSS.
  if (!gnss.enableGps()) {
    std::cerr << "Failed to enable GPS." << std::endl;
    return false;
  }
  if (!gnss.enableSbas()) {
    std::cerr << "Failed to enable SBAS." << std::endl;
    return false;
  }
  if (!gnss.enableQzss()) {
    std::cerr << "Failed to enable QZSS." << std::endl;
    return false;
  }

  // Enable messages.
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_PVT, true)) {
    std::cerr << "Failed to enable NAV_PVT message." << std::endl;
    return false;
  }

  const auto start_time = ch::steady_clock::now();

  while (ch::steady_clock::now() - start_time < 3s) {
    if (!gnss.update(false)) {
      std::cerr << "Failed to update GNSS driver." << std::endl;
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

  std::cerr << "Timeout before receiving the first GPS message." << std::endl;
  return false;
}

bool testSbus()
{
  bool sbus_received = false;

  tobas::SBUS sbus([&sbus_received](const tobas::SBUS::Packet&) { sbus_received = true; });

  if (!sbus.initialize("/dev/ttyAMA0")) {
    std::cerr << "Failed to initialize S.BUS driver." << std::endl;
    return false;
  }

  sbus.start();

  const auto start_time = ch::steady_clock::now();
  while (ch::steady_clock::now() - start_time < 1min) {
    if (sbus_received) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }

  std::cerr << "Timeout before the first S.BUS package." << std::endl;
  return false;
}

int main()
{
  // PM & IMU
  std::cout << "Testing PM & IMU..." << std::endl;
  if (!testBattImu()) {
    std::cerr << "PM & IMU test failed." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "PM & IMU test passed." << std::endl;

  // Magnetometer
  std::cout << "Testing magnetometer..." << std::endl;
  if (!testMagnetometer()) {
    std::cerr << "Magnetometer test failed." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Magnetometer test passed." << std::endl;

  // Barometer
  std::cout << "Testing baro..." << std::endl;
  if (!testBarometer()) {
    std::cerr << "Barometer test failed." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Barometer test passed." << std::endl;

  // GNSS Receiver
  std::cout << "Testing GNSS receiver..." << std::endl;
  if (!testGnssReceiver()) {
    std::cerr << "GNSS receiver test failed." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "GNSS receiver test passed." << std::endl;

  // S.BUS
  std::cout << "Testing S.BUS..." << std::endl;
  if (!testSbus()) {
    std::cerr << "S.BUS test failed." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "S.BUS test passed." << std::endl;

  std::cout << GREEN_PREFIX << "All tests passed." << COLOR_RESET << std::endl;
  return EXIT_SUCCESS;
}
