#include <iostream>

#include "../include/tobas_t1_core/ilps22qs.hpp"

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

bool ILPS22QS::readPressure(double& pres)
{
  if (!readRegs(PRESSURE_OUT_XL, 3)) {
    return false;
  }

  const auto lsb = (i2c_.rx[2] << 16) | (i2c_.rx[1] << 8) | i2c_.rx[0];
  pres = lsb / pres_scale_;

  return true;
}

bool ILPS22QS::readTemperature(double& temp)
{
  if (!readRegs(TEMP_OUT_L, 2)) {
    return false;
  }

  const auto lsb = (i2c_.rx[1] << 8) | i2c_.rx[0];
  temp = lsb / kTempScale;

  return true;
}

bool ILPS22QS::writeReg(const uint8_t& addr, const uint8_t& data)
{
  i2c_.tx[0] = data;
  return i2c_.writeBytes(addr, 1);
}

bool ILPS22QS::readRegs(const uint8_t& addr, const size_t& bytes)
{
  if (!i2c_.readBytes(addr, bytes)) {
    return false;
  }

  return true;
}

bool ILPS22QS::checkWhoAmI()
{
  if (!readRegs(WHO_AM_I_REG, 1)) {
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

  if (!writeReg(CTRL_REG1, ODR_200HZ | AVG_32)) {
    return false;
  }

  if (!writeReg(CTRL_REG2, fs_mode | LPF_CFG_4 | ENABLE_LPF)) {
    return false;
  }

  if (!writeReg(CTRL_REG3, IF_ADD_INC)) {
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
