// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/stmicro/ilps22qs.hpp"

#include <iostream>

namespace tobas
{
namespace stm
{
namespace
{
constexpr uint8_t kI2cAddress = 0b1011100;
constexpr double kTempScale = 100;  // [LSB/degC]
}  // namespace

ILPS22QS::ILPS22QS()
{
}

bool ILPS22QS::initialize(const char* i2c_device)
{
  if (!i2c_.initialize(i2c_device, kI2cAddress)) {
    return false;
  }

  if (!checkWhoAmI()) {
    return false;
  }

  if (!configure()) {
    return false;
  }

  return true;
}

bool ILPS22QS::readPressure(double& pressure)
{
  if (!i2c_.readBytes(PRESSURE_OUT_XL, 3, &pres_lsb_)) {
    return false;
  }

  pressure = static_cast<double>(pres_lsb_) / pres_scale_;

  return true;
}

bool ILPS22QS::readTemperature(double& temperature)
{
  if (!i2c_.readBytes(TEMP_OUT_L, 2, &temp_lsb_)) {
    return false;
  }

  temperature = static_cast<double>(temp_lsb_) / kTempScale;

  return true;
}

bool ILPS22QS::checkWhoAmI()
{
  uint8_t byte;

  if (!i2c_.readByte(WHO_AM_I_REG, byte)) {
    std::cerr << "Failed to read WHO_AM_I data." << std::endl;
    return false;
  }

  if (byte != WHO_AM_I) {
    std::cerr << "Barometer is not recognized." << std::endl;
    return false;
  }

  return true;
}

bool ILPS22QS::configure()
{
  constexpr uint8_t fs_mode = FS_MODE_1260HPA;

  if (!i2c_.writeByte(CTRL_REG1, ODR_100HZ | AVG_32, true)) {
    std::cerr << "Failed to write to CTRL_REG1." << std::endl;
    return false;
  }

  if (!i2c_.writeByte(CTRL_REG2, fs_mode | LPF_CFG_4 | ENABLE_LPF | BLOCK_DATA_UPDATE, true)) {
    std::cerr << "Failed to write to CTRL_REG2." << std::endl;
    return false;
  }

  if (!i2c_.writeByte(CTRL_REG3, IF_ADD_INC, true)) {
    std::cerr << "Failed to write to CTRL_REG3." << std::endl;
    return false;
  }

  setPressureScale(fs_mode);

  return true;
}

void ILPS22QS::setPressureScale(const uint8_t& fs_mode)
{
  switch (fs_mode) {
    case FS_MODE_1260HPA:
      pres_scale_ = 40.96;
      break;
    case FS_MODE_4060HPA:
      pres_scale_ = 20.48;
      break;
    default:
      throw;
  }
}
}  // namespace stm
}  // namespace tobas
