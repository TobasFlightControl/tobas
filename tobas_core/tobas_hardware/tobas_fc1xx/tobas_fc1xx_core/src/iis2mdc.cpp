#include "tobas_fc1xx_core/iis2mdc.hpp"

#include <iostream>

using namespace std;

namespace fc1xx
{
IIS2MDC::IIS2MDC()
{
}

bool IIS2MDC::initialize()
{
  if (!i2c_.initialize(kI2cDevice, kI2cAddress)) {
    cerr << "Failed to initialize I2C device." << endl;
    return false;
  }

  if (!checkWhoAmI()) {
    cerr << "Who-Am-I check failed." << endl;
    return false;
  }

  if (!configure()) {
    cerr << "Failed to configure magnetometer." << endl;
    return false;
  }

  // Discard the initial data stored in the registers
  double tmp[3];
  if (!readMag(tmp[0], tmp[1], tmp[2])) {
    return false;
  }

  return true;
}

bool IIS2MDC::readMag(double& mx, double& my, double& mz)
{
  if (!i2c_.readBytes(OUTX_L_REG | kMultiReadFlag, sizeof(mag_buf_), mag_buf_)) {
    return false;
  }

  mx = static_cast<double>(mag_buf_[0]) * kSensitivity;
  my = -static_cast<double>(mag_buf_[1]) * kSensitivity;
  mz = static_cast<double>(mag_buf_[2]) * kSensitivity;

  return true;
}

bool IIS2MDC::checkWhoAmI()
{
  uint8_t byte;

  if (!i2c_.readByte(WHO_AM_I_REG, byte)) {
    cerr << "Failed to read WHO_AM_I data." << endl;
    return false;
  }

  if (byte != WHO_AM_I) {
    cerr << "Magnetometer is not recognized." << endl;
    return false;
  }

  return true;
}

bool IIS2MDC::configure()
{
  if (!i2c_.writeByte(CFG_REG_A, COMP_TEMP_EN | ODR_100HZ | MD_CONTINUOUS, true)) {
    cerr << "Failed to write to CFG_REG_A." << endl;
    return false;
  }

  if (!i2c_.writeByte(CFG_REG_B, LPF, true)) {
    cerr << "Failed to write to CFG_REG_B." << endl;
    return false;
  }

  if (!i2c_.writeByte(CFG_REG_C, 0x00, true)) {
    cerr << "Failed to write to CFG_REG_C." << endl;
    return false;
  }

  return true;
}
}  // namespace fc1xx
