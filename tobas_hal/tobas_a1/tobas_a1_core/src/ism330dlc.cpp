#include <iostream>

#include <tobas_std_tools/universal_constants.hpp>

#include "../include/tobas_a1_core/ism330dlc.hpp"
#include "../include/tobas_a1_core/constants.hpp"

using namespace std;

namespace a1
{
ISM330DLC::ISM330DLC()
{
}

bool ISM330DLC::initialize()
{
  if (!spi_.initialize(spi_device::kImuDev, kSpiClockFreq, kSpiBufSize))
    return false;

  if (!checkWhoAmI())
    return false;

  if (!configureAcc())
    return false;

  if (!configureGyro())
    return false;

  if (!writeReg(REG_CTRL4_C, I2C_DISABLE))
    return false;

  return true;
}

bool ISM330DLC::readAcc(double& ax, double& ay, double& az)
{
  if (!readRegs(REG_OUTX_L_XL, 6))
    return false;

  for (size_t i = 0; i < 3; ++i)
    bit_data_[i] = ((int16_t)res_[2 * i + 1] << 8) | res_[2 * i];

  // TODO: 軸や符号の変換が必要かも
  ax = bit_data_[0] * acc_scale_;
  ay = bit_data_[1] * acc_scale_;
  az = bit_data_[2] * acc_scale_;

  return true;
}

bool ISM330DLC::readGyro(double& gx, double& gy, double& gz)
{
  if (!readRegs(REG_OUTX_L_G, 6))
    return false;

  for (size_t i = 0; i < 3; ++i)
    bit_data_[i] = ((int16_t)res_[2 * i + 1] << 8) | res_[2 * i];

  // TODO: 軸や符号の変換が必要かも
  gx = bit_data_[0] * gyro_scale_;
  gy = bit_data_[1] * gyro_scale_;
  gz = bit_data_[2] * gyro_scale_;

  return true;
}

bool ISM330DLC::writeReg(const uint8_t& addr, const uint8_t& data)
{
  spi_.tx[0] = addr;
  spi_.tx[1] = data;
  return spi_.transfer(2);
}

bool ISM330DLC::readRegs(const uint8_t& addr, const size_t& bytes)
{
  spi_.tx[0] = addr | kReadFlag;

  if (!spi_.transfer(bytes + 1))
    return false;

  for (size_t i = 0; i < bytes; ++i)
    res_[i] = spi_.rx[i + 1];

  return true;
}

bool ISM330DLC::checkWhoAmI()
{
  if (!readRegs(REG_WHO_AM_I, 1))
    return false;

  if (res_[0] != WHO_AM_I)
  {
    cerr << "IMU is not recognized." << endl;
    return false;
  }

  return true;
}

bool ISM330DLC::configureAcc()
{
  constexpr uint8_t scale = FS_XL_4G;

  if (!writeReg(REG_CTRL1_XL, ODR_XL_6660HZ | scale))
    return false;

  setAccScale(scale);

  return true;
}

bool ISM330DLC::configureGyro()
{
  constexpr uint8_t scale = FS_G_500DPS;

  if (!writeReg(REG_CTRL2_G, ODR_G_6660HZ | scale))
    return false;

  setGyroScale(scale);

  return true;
}

void ISM330DLC::setAccScale(const uint8_t& scale)
{
  // LSB -> mg (Linear acceleration sensitivity | 4.1 Mechanical characteristics)
  switch (scale)
  {
    case FS_XL_2G:
      acc_scale_ = 0.061;
      break;
    case FS_XL_4G:
      acc_scale_ = 0.122;
      break;
    case FS_XL_8G:
      acc_scale_ = 0.244;
      break;
    case FS_XL_16G:
      acc_scale_ = 0.488;
      break;
    default:
      throw;
  }

  // mg -> g
  acc_scale_ *= 1e-3;

  // g -> m/s^2
  acc_scale_ *= tobas_std::kGravity;
}

void ISM330DLC::setGyroScale(const uint8_t& scale)
{
  // LSB -> mdps (Angular rate sensitivity | 4.1 Mechanical characteristics)
  switch (scale)
  {
    case FS_G_125DPS:
      gyro_scale_ = 4.375;
      break;
    case FS_G_250DPS:
      gyro_scale_ = 8.75;
      break;
    case FS_G_500DPS:
      gyro_scale_ = 17.5;
      break;
    case FS_G_1000DPS:
      gyro_scale_ = 35.;
      break;
    case FS_G_2000DPS:
      gyro_scale_ = 70.;
      break;
    default:
      throw;
  }

  // mdps -> dps
  gyro_scale_ *= 1e-3;

  // dps -> rad/s
  gyro_scale_ *= tobas_std::kDeg2Rad;
}
}  // namespace a1
