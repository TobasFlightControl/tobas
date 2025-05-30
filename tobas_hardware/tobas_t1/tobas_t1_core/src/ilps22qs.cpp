#include "tobas_t1_core/ilps22qs.hpp"

#include <cassert>
#include <iostream>

using namespace std;

namespace t1
{
ILPS22QS::ILPS22QS()
{
}

bool ILPS22QS::initialize()
{
  if (!i2c_.initialize(kI2cDevice, kI2cAddress)) {
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
  if (!readRegs(PRESSURE_OUT_XL, 3)) {
    return false;
  }

  const auto lsb = (i2c_.rx[2] << 16) | (i2c_.rx[1] << 8) | i2c_.rx[0];
  pressure = lsb / pres_scale_;

  return true;
}

bool ILPS22QS::readTemperature(double& temperature)
{
  if (!readRegs(TEMP_OUT_L, 2)) {
    return false;
  }

  const auto lsb = (i2c_.rx[1] << 8) | i2c_.rx[0];
  temperature = lsb / kTempScale;

  return true;
}

bool ILPS22QS::readReg(const uint8_t& addr)
{
  return i2c_.readByte(addr);
}

bool ILPS22QS::readRegs(const uint8_t& addr, const size_t& bytes)
{
  assert(bytes >= 2);
  return i2c_.readBytes(addr, bytes);
}

bool ILPS22QS::writeReg(const uint8_t& addr, const uint8_t& data)
{
  i2c_.tx[0] = data;
  return i2c_.writeByte(addr, true);
}

bool ILPS22QS::checkWhoAmI()
{
  if (!readReg(WHO_AM_I_REG)) {
    cerr << "Failed to read WHO_AM_I data." << endl;
    return false;
  }

  if (i2c_.rx[0] != WHO_AM_I) {
    cerr << "Barometer is not recognized." << endl;
    return false;
  }

  return true;
}

bool ILPS22QS::configure()
{
  constexpr uint8_t fs_mode = FS_MODE_1260HPA;

  if (!writeReg(CTRL_REG1, ODR_100HZ | AVG_32)) {
    cerr << "Failed to write to CTRL_REG1." << endl;
    return false;
  }

  if (!writeReg(CTRL_REG2, fs_mode | LPF_CFG_4 | ENABLE_LPF)) {
    cerr << "Failed to write to CTRL_REG2." << endl;
    return false;
  }

  if (!writeReg(CTRL_REG3, IF_ADD_INC)) {
    cerr << "Failed to write to CTRL_REG3." << endl;
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
}  // namespace t1
