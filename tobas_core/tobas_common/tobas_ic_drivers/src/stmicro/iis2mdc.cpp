// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/stmicro/iis2mdc.hpp"

#include <iostream>

namespace tobas
{
namespace stm
{
IIS2MDC::IIS2MDC()
{
}

bool IIS2MDC::initialize(const char* i2c_device)
{
  if (!i2c_.initialize(i2c_device, 0b0011110)) {
    std::cerr << "Failed to initialize I2C device." << std::endl;
    return false;
  }

  if (!checkWhoAmI()) {
    std::cerr << "Who-Am-I check failed." << std::endl;
    return false;
  }

  if (!configure()) {
    std::cerr << "Failed to configure magnetometer." << std::endl;
    return false;
  }

  return true;
}

bool IIS2MDC::readMag(double& mx, double& my, double& mz)
{
  constexpr uint8_t kMultiReadFlag = 0x80;  // cf. 6.1.1: I2C operation (p.23)
  if (!i2c_.readBytes(OUTX_L_REG | kMultiReadFlag, sizeof(mag_buf_), mag_buf_)) {
    return false;
  }

  constexpr double kSensitivity = 1.5e-3;  // [gauss/LSB]
  mx = static_cast<double>(mag_buf_[0]) * kSensitivity;
  my = static_cast<double>(mag_buf_[1]) * kSensitivity;
  mz = static_cast<double>(mag_buf_[2]) * kSensitivity;

  return true;
}

bool IIS2MDC::checkWhoAmI()
{
  uint8_t byte;

  if (!i2c_.readByte(WHO_AM_I_REG, byte)) {
    std::cerr << "Failed to read WHO_AM_I data." << std::endl;
    return false;
  }

  if (byte != WHO_AM_I) {
    std::cerr << "Magnetometer is not recognized." << std::endl;
    return false;
  }

  return true;
}

bool IIS2MDC::configure()
{
  if (!i2c_.writeByte(CFG_REG_A, COMP_TEMP_EN | ODR_100HZ | MD_CONTINUOUS, true)) {
    std::cerr << "Failed to write to CFG_REG_A." << std::endl;
    return false;
  }

  if (!i2c_.writeByte(CFG_REG_B, LPF, true)) {
    std::cerr << "Failed to write to CFG_REG_B." << std::endl;
    return false;
  }

  if (!i2c_.writeByte(CFG_REG_C, BDU, true)) {
    std::cerr << "Failed to write to CFG_REG_C." << std::endl;
    return false;
  }

  return true;
}
}  // namespace stm
}  // namespace tobas
