// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cmath>
#include <iostream>

#include <tobas_ic_drivers/stmicro/iis2mdc.hpp>
#include <tobas_ic_drivers/stmicro/ilps22qs.hpp>
#include <tobas_ic_drivers/ublox/gnss_receiver.hpp>
#include <tobas_ic_drivers/ublox/ubx_transport_spi.hpp>
#include <tobas_mcu_driver/tobas_t2/pwm_batt_imu.hpp>
#include <tobas_sbus_driver/sbus.hpp>
#include <tobas_std_tools/ansi_text_styles.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_time_tools/rate.hpp>

using namespace std::chrono_literals;
namespace ch = std::chrono;

bool testBattImu()
{
  tobas::t2::PwmBattImu driver;

  if (!driver.initialize("/dev/spidev0.1")) {
    std::cerr << "Failed to initialize ADC.\n";
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
      std::cerr << "Failed to communicate with the MCU.\n";
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

    if (volt < 5.0 || 50.0 < volt) {
      std::cerr << "Abnormal voltage detected.\n";
      return false;
    }
    if (curr <= 0.0 || 10.0 < curr) {
      std::cerr << "Abnormal current detected.\n";
      return false;
    }
    if (std::hypot(ax, ay, az - tobas::st::kGravity) > 1.0) {
      std::cerr << "Abnormal accel detected.\n";
      return false;
    }
    if (std::hypot(gx, gy, gz) > 0.3) {
      std::cerr << "Abnormal gyro detected.\n";
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
    std::cerr << "Failed to initialize magnetometer.\n";
    return false;
  }

  std::this_thread::sleep_for(200ms);

  double mx, my, mz;
  tobas::tim::Rate rate(20ms);

  for (int _ = 0; _ < 50; ++_) {
    if (!mag.readMag(mx, my, mz)) {
      std::cerr << "Failed to read magnetic field.\n";
      return false;
    }

    std::cout << "Magnetic Field [gauss]: " << mx << ", " << my << ", " << mz << '\n';

    if (std::hypot(mx, my, mz) > 1.5) {  // Allow up to 3 times the standard magnetic field strength (~0.5).
      std::cerr << "Abnormal magnetic field detected.\n";
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
    std::cerr << "Failed to initialize barometer.\n";
    return false;
  }

  std::this_thread::sleep_for(200ms);

  double pres, temp;
  tobas::tim::Rate rate(20ms);

  for (int _ = 0; _ < 50; ++_) {
    if (!baro.readPressure(pres)) {
      std::cerr << "Failed to read pressure.\n";
      return false;
    }
    if (!baro.readTemperature(temp)) {
      std::cerr << "Failed to read temperature.\n";
      return false;
    }

    const auto pres_hpa = pres / 100.0;

    std::cout << "Pressure [hPa]    : " << pres_hpa << '\n';
    std::cout << "Temperature [degC]: " << temp << '\n';

    if (pres_hpa < 900.0 || 1100.0 < pres_hpa) {
      std::cerr << "Abnormal air pressure detected.\n";
      return false;
    }
    if (temp < 0.0 || 80.0 < temp) {
      std::cerr << "Abnormal temperature detected.\n";
      return false;
    }

    rate.sleep();
  }

  return true;
}

bool testGnssReceiver()
{
  tobas::ublox::GnssReceiver gnss(std::make_unique<tobas::ublox::UbxTransportSpi>("/dev/spidev1.0"));

  if (!gnss.initialize()) {
    std::cerr << "Failed to initialize GNSS driver.\n";
    return false;
  }

  if (!gnss.configureMeasurementRate(1000)) {
    std::cerr << "Failed to configure measurement rate.\n";
    return false;
  }

  // Enable GNSS.
  if (!gnss.enableGps()) {
    std::cerr << "Failed to enable GPS.\n";
    return false;
  }
  if (!gnss.enableSbas()) {
    std::cerr << "Failed to enable SBAS.\n";
    return false;
  }
  if (!gnss.enableQzss()) {
    std::cerr << "Failed to enable QZSS.\n";
    return false;
  }

  // Enable messages.
  if (!gnss.enableMessage(tobas::ublox::GnssReceiver::CLASS_NAV, tobas::ublox::GnssReceiver::NAV_PVT, true)) {
    std::cerr << "Failed to enable NAV_PVT message.\n";
    return false;
  }

  const auto start_time = ch::steady_clock::now();

  while (ch::steady_clock::now() - start_time < 3s) {
    if (!gnss.update(true)) {
      std::cerr << "Failed to receive a GNSS message.\n";
      return false;
    }

    if (gnss.latestClass() != tobas::ublox::GnssReceiver::CLASS_NAV) {
      continue;
    }

    switch (gnss.latestId()) {
      case tobas::ublox::GnssReceiver::NAV_PVT:
        return true;
      default:
        continue;
    }
  }

  std::cerr << "Timeout before receiving the first GPS message.\n";
  return false;
}

bool testSbus()
{
  bool sbus_received = false;

  tobas::SBUS sbus([&sbus_received](const tobas::SBUS::Packet&) { sbus_received = true; });

  if (!sbus.initialize("/dev/ttyAMA0")) {
    std::cerr << "Failed to initialize S.BUS driver.\n";
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

  std::cerr << "Timeout before the first S.BUS package.\n";
  return false;
}

int main()
{
  // PM & IMU
  std::cout << "Testing PM & IMU...\n";
  if (!testBattImu()) {
    std::cerr << "PM & IMU test failed.\n";
    return EXIT_FAILURE;
  }
  std::cout << "PM & IMU test passed.\n";

  // Magnetometer
  std::cout << "Testing magnetometer...\n";
  if (!testMagnetometer()) {
    std::cerr << "Magnetometer test failed.\n";
    return EXIT_FAILURE;
  }
  std::cout << "Magnetometer test passed.\n";

  // Barometer
  std::cout << "Testing baro...\n";
  if (!testBarometer()) {
    std::cerr << "Barometer test failed.\n";
    return EXIT_FAILURE;
  }
  std::cout << "Barometer test passed.\n";

  // GNSS Receiver
  std::cout << "Testing GNSS receiver...\n";
  if (!testGnssReceiver()) {
    std::cerr << "GNSS receiver test failed.\n";
    return EXIT_FAILURE;
  }
  std::cout << "GNSS receiver test passed.\n";

  // S.BUS
  std::cout << "Testing S.BUS...\n";
  if (!testSbus()) {
    std::cerr << "S.BUS test failed.\n";
    return EXIT_FAILURE;
  }
  std::cout << "S.BUS test passed.\n";

  std::cout << tobas::st::kGreenPrefix << "All tests passed." << tobas::st::kColorReset << '\n';
  return EXIT_SUCCESS;
}
