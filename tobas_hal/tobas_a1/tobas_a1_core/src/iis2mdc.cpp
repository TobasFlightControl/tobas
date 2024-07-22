#include <iostream>

#include "../include/tobas_a1_core/iis2mdc.hpp"
#include "../include/tobas_a1_core/constants.hpp"

using namespace std;

namespace a1
{
IIS2MDC::IIS2MDC()
{
}

bool IIS2MDC::initialize()
{
  if (!i2c_.initialize(kRasPiI2CDev, i2c_address::kMagAddress, 6))
    return false;

  if (!checkWhoAmI())
    return false;

  if (!configure())
    return false;

  return true;
}

bool IIS2MDC::readMag(double& mx, double& my, double& mz)
{
  if (!readRegs(OUTX_L_REG, 6))
    return false;

  mx = (((int16_t)i2c_.rx[1] << 8) | i2c_.rx[0]) * kSensitivity;
  my = (((int16_t)i2c_.rx[3] << 8) | i2c_.rx[2]) * kSensitivity;
  mz = (((int16_t)i2c_.rx[5] << 8) | i2c_.rx[4]) * kSensitivity;

  return true;
}

bool IIS2MDC::writeReg(const uint8_t& addr, const uint8_t& data)
{
  i2c_.tx[0] = data;
  return i2c_.writeBytes(addr, 1);
}

bool IIS2MDC::readRegs(const uint8_t& addr, const size_t& bytes)
{
  if (!i2c_.readBytes(addr | kMultiReadFlag, bytes))
    return false;

  return true;
}

bool IIS2MDC::checkWhoAmI()
{
  if (!readRegs(WHO_AM_I_REG, 1))
    return false;

  if (i2c_.rx[0] != WHO_AM_I)
  {
    cerr << "Magnetometer is not recognized." << endl;
    return false;
  }

  return true;
}

bool IIS2MDC::configure()
{
  if (!writeReg(CFG_REG_A, COMP_TEMP_EN | ODR_100HZ | MD_CONTINUOUS))
    return false;

  return true;
}
}  // namespace a1
