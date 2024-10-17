#include <iostream>
#include <cstring>

#include <tobas_std_tools/universal_constants.hpp>

#include "../include/tobas_aso_core/ism330dlc.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;

namespace aso
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

  // 正負両方の値を表現するために，一度符号付き16ビット整数型に変換する必要がある
  ax = static_cast<int16_t>((res_[1] << 8) | res_[0]) * acc_scale_;
  ay = static_cast<int16_t>((res_[3] << 8) | res_[2]) * acc_scale_;
  az = static_cast<int16_t>((res_[5] << 8) | res_[4]) * acc_scale_;

  return true;
}

bool ISM330DLC::readGyro(double& gx, double& gy, double& gz)
{
  if (!readRegs(REG_OUTX_L_G, 6))
    return false;

  gx = static_cast<int16_t>((res_[1] << 8) | res_[0]) * gyro_scale_;
  gy = static_cast<int16_t>((res_[3] << 8) | res_[2]) * gyro_scale_;
  gz = static_cast<int16_t>((res_[5] << 8) | res_[4]) * gyro_scale_;

  return true;
}

bool ISM330DLC::readRegs(const uint8_t& addr, const size_t& bytes)
{
  spi_.tx[0] = addr | kReadFlag;

  if (!spi_.transfer(bytes + 1))
    return false;

  memcpy(res_, spi_.rx + 1, bytes);

  return true;
}

bool ISM330DLC::writeReg(const uint8_t& addr, const uint8_t& data)
{
  spi_.tx[0] = addr;
  spi_.tx[1] = data;
  return spi_.transfer(2);
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

  if (!writeReg(REG_CTRL1_XL, ODR_XL_1660HZ | scale))
    return false;

  // Anti-aliasing (fc = 1660 / 9 = 184Hz)
  if (!writeReg(REG_CTRL8_XL, LPF2_XL_EN | HPCF_XL_9))
    return false;

  setAccScale(scale);

  return true;
}

bool ISM330DLC::configureGyro()
{
  constexpr uint8_t scale = FS_G_500DPS;

  if (!writeReg(REG_CTRL2_G, ODR_G_1660HZ | scale))
    return false;

  // Enable LPF1
  if (!writeReg(REG_CTRL4_C, I2C_DISABLE | LPF1_SEL_G))
    return false;

  // Anti-aliasing (fc = 168Hz)
  if (!writeReg(REG_CTRL6_C, FTYPE_0))
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
}  // namespace aso
