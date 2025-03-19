#include <iostream>
#include <cstring>

#include <tobas_std_tools/universal_constants.hpp>

#include "../include/tobas_ic_drivers/stmicro/ism330dlc.hpp"

using namespace std;

namespace stm
{
ISM330DLC::ISM330DLC()
{
}

bool ISM330DLC::initialize(const char* spi_device)
{
  if (!spi_.initialize(spi_device, tx_buf_, rx_buf_, kSPIClockFreq))
    return false;

  if (!checkWhoAmI())
    return false;

  // Set full scales
  if (!setAccelFullScale(fs_xl_t::FS_XL_2G))
    return false;
  if (!setGyroFullScale(fs_g_t::FS_G_250DPS))
    return false;

  // Disable I2C
  if (!writeReg(REG_CTRL4_C, I2C_DISABLE))
    return false;

  return true;
}

bool ISM330DLC::setAccelOutputDataRate(odr_xl_t odr)
{
  if (!readRegs(REG_CTRL1_XL, 1))
    return false;

  auto ctrl1_xl = res_[0];
  ctrl1_xl &= 0b00001111;

  switch (odr)
  {
    case odr_xl_t::ODR_XL_26HZ:
      ctrl1_xl |= ODR_XL_26HZ;
      break;
    case odr_xl_t::ODR_XL_52HZ:
      ctrl1_xl |= ODR_XL_52HZ;
      break;
    case odr_xl_t::ODR_XL_104HZ:
      ctrl1_xl |= ODR_XL_104HZ;
      break;
    case odr_xl_t::ODR_XL_208HZ:
      ctrl1_xl |= ODR_XL_208HZ;
      break;
    case odr_xl_t::ODR_XL_416HZ:
      ctrl1_xl |= ODR_XL_416HZ;
      break;
    case odr_xl_t::ODR_XL_833HZ:
      ctrl1_xl |= ODR_XL_833HZ;
      break;
    case odr_xl_t::ODR_XL_1660HZ:
      ctrl1_xl |= ODR_XL_1660HZ;
      break;
    case odr_xl_t::ODR_XL_3330HZ:
      ctrl1_xl |= ODR_XL_3330HZ;
      break;
    case odr_xl_t::ODR_XL_6660HZ:
      ctrl1_xl |= ODR_XL_6660HZ;
      break;
    default:
      cerr << "Invalid accelerometer output data rate type: " << (int)odr << endl;
      return false;
  }

  if (!writeReg(REG_CTRL1_XL, ctrl1_xl))
    return false;

  return true;
}

bool ISM330DLC::setGyroOutputDataRate(odr_g_t odr)
{
  if (!readRegs(REG_CTRL2_G, 1))
    return false;

  auto ctrl2_g = res_[0];
  ctrl2_g &= 0b00001111;

  switch (odr)
  {
    case odr_g_t::ODR_G_26HZ:
      ctrl2_g |= ODR_G_26HZ;
      break;
    case odr_g_t::ODR_G_52HZ:
      ctrl2_g |= ODR_G_52HZ;
      break;
    case odr_g_t::ODR_G_104HZ:
      ctrl2_g |= ODR_G_104HZ;
      break;
    case odr_g_t::ODR_G_208HZ:
      ctrl2_g |= ODR_G_208HZ;
      break;
    case odr_g_t::ODR_G_416HZ:
      ctrl2_g |= ODR_G_416HZ;
      break;
    case odr_g_t::ODR_G_833HZ:
      ctrl2_g |= ODR_G_833HZ;
      break;
    case odr_g_t::ODR_G_1660HZ:
      ctrl2_g |= ODR_G_1660HZ;
      break;
    case odr_g_t::ODR_G_3330HZ:
      ctrl2_g |= ODR_G_3330HZ;
      break;
    case odr_g_t::ODR_G_6660HZ:
      ctrl2_g |= ODR_G_6660HZ;
      break;
    default:
      cerr << "Invalid gyroscope output data rate type: " << (int)odr << endl;
      return false;
  }

  if (!writeReg(REG_CTRL2_G, ctrl2_g))
    return false;

  return true;
}

bool ISM330DLC::setAccelFullScale(fs_xl_t fs)
{
  if (!readRegs(REG_CTRL1_XL, 1))
    return false;

  auto ctrl1_xl = res_[0];
  ctrl1_xl &= 0b11110011;

  switch (fs)
  {
    case fs_xl_t::FS_XL_2G:
      ctrl1_xl |= FS_XL_2G;
      acc_scale_ = 0.061;
      break;
    case fs_xl_t::FS_XL_4G:
      ctrl1_xl |= FS_XL_4G;
      acc_scale_ = 0.122;
      break;
    case fs_xl_t::FS_XL_8G:
      ctrl1_xl |= FS_XL_8G;
      acc_scale_ = 0.244;
      break;
    case fs_xl_t::FS_XL_16G:
      ctrl1_xl |= FS_XL_16G;
      acc_scale_ = 0.488;
      break;
    default:
      cerr << "Invalid accelerometer full scale type: " << (int)fs << endl;
      return false;
  }

  if (!writeReg(REG_CTRL1_XL, ctrl1_xl))
    return false;

  // LSB -> mg -> g -> m/s^2 (Linear acceleration sensitivity | 4.1 Mechanical characteristics)
  acc_scale_ *= 1e-3;
  acc_scale_ *= tobas_std::kGravity;

  return true;
}

bool ISM330DLC::setGyroFullScale(fs_g_t fs)
{
  if (!readRegs(REG_CTRL2_G, 1))
    return false;

  auto ctrl2_g = res_[0];
  ctrl2_g &= 0b11110001;

  switch (fs)
  {
    case fs_g_t::FS_G_125DPS:
      ctrl2_g |= FS_G_125DPS;
      gyro_scale_ = 4.375;
      break;
    case fs_g_t::FS_G_250DPS:
      ctrl2_g |= FS_G_250DPS;
      gyro_scale_ = 8.75;
      break;
    case fs_g_t::FS_G_500DPS:
      ctrl2_g |= FS_G_500DPS;
      gyro_scale_ = 17.5;
      break;
    case fs_g_t::FS_G_1000DPS:
      ctrl2_g |= FS_G_1000DPS;
      gyro_scale_ = 35.;
      break;
    case fs_g_t::FS_G_2000DPS:
      ctrl2_g |= FS_G_2000DPS;
      gyro_scale_ = 70.;
      break;
    default:
      cerr << "Invalid gyroscope full scale type: " << (int)fs << endl;
      return false;
  }

  if (!writeReg(REG_CTRL2_G, ctrl2_g))
    return false;

  // mdps -> dps -> rad/s (Angular rate sensitivity | 4.1 Mechanical characteristics)
  gyro_scale_ *= 1e-3;
  gyro_scale_ *= tobas_std::kDeg2Rad;

  return true;
}

bool ISM330DLC::readAccel(double& ax, double& ay, double& az)
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
  tx_buf_[0] = addr | kReadFlag;

  if (!spi_.transfer(bytes + 1))
    return false;

  memcpy(res_, rx_buf_ + 1, bytes);

  return true;
}

bool ISM330DLC::writeReg(const uint8_t& addr, const uint8_t& data)
{
  tx_buf_[0] = addr;
  tx_buf_[1] = data;
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
}  // namespace stm
