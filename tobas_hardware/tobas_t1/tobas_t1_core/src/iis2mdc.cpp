#include "../include/tobas_t1_core/iis2mdc.hpp"

#include <bitset>
#include <iostream>

using namespace std;

namespace t1
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

  return true;
}

bool IIS2MDC::readMag(double& mx, double& my, double& mz)
{
  if (!readRegs(OUTX_L_REG, 6)) {
    return false;
  }

  // 正負両方の値を表現するために，一度符号付き16ビット整数型に変換する必要がある
  mx = static_cast<int16_t>((i2c_.rx[1] << 8) | i2c_.rx[0]) * kSensitivity;
  my = -static_cast<int16_t>((i2c_.rx[3] << 8) | i2c_.rx[2]) * kSensitivity;
  mz = static_cast<int16_t>((i2c_.rx[5] << 8) | i2c_.rx[4]) * kSensitivity;

  return true;
}

bool IIS2MDC::writeReg(const uint8_t& addr, const uint8_t& data)
{
  i2c_.tx[0] = data;
  return i2c_.writeBytes(addr, 1);
}

bool IIS2MDC::readRegs(const uint8_t& addr, const size_t& bytes)
{
  if (!i2c_.readBytes(addr | kMultiReadFlag, bytes)) {
    cerr << "Failed to read " << bytes << " bytes from " << bitset<8>(addr) << "." << endl;
    return false;
  }

  return true;
}

bool IIS2MDC::checkWhoAmI()
{
  if (!readRegs(WHO_AM_I_REG, 1)) {
    cerr << "Failed to read WHO_AM_I data." << endl;
    return false;
  }

  if (i2c_.rx[0] != WHO_AM_I) {
    cerr << "Magnetometer is not recognized." << endl;
    return false;
  }

  return true;
}

bool IIS2MDC::configure()
{
  // XXX: サンプリング周波数が高いほどモータなど外部磁場の影響を受けやすくなる．おそらく電流値を下げるのが大事．
  if (!writeReg(CFG_REG_A, COMP_TEMP_EN | ODR_100HZ | MD_CONTINUOUS)) {
    cerr << "Failed to write to CFG_REG_A." << endl;
    return false;
  }

  if (!writeReg(CFG_REG_B, OFF_CANC | LPF)) {
    cerr << "Failed to write to CFG_REG_B." << endl;
    return false;
  }

  if (!writeReg(CFG_REG_C, 0)) {
    cerr << "Failed to write to CFG_REG_C." << endl;
    return false;
  }

  return true;
}
}  // namespace t1
