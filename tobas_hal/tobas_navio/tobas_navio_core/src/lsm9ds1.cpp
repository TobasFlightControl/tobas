#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_std_tools/time.hpp>

#include "../include/tobas_navio_core/lsm9ds1.hpp"

namespace navio
{
LSM9DS1::LSM9DS1()
{
}

bool LSM9DS1::initialize()
{
  if (!spi_dev_imu_.initialize(kAccGyroDevice, kSpiSpeedHz))
    return false;
  if (!spi_dev_mag_.initialize(kMagDevice, kSpiSpeedHz))
    return false;

  if (readReg(spi_dev_imu_, XG_WHO_AM_I) != WHO_AM_I_ACC_GYRO)
    return false;
  if (readReg(spi_dev_mag_, M_WHO_AM_I) != WHO_AM_I_MAG)
    return false;

  initializeGyroscope();
  initializeAccelerometer();
  initializeMagnetometer();

  return true;
}

void LSM9DS1::update()
{
  updateTemperature();
  updateAccelerometer();
  updateGyroscope();
  updateMagnetometer();
}

void LSM9DS1::updateTemperature()
{
  readRegsImu(XG_OUT_TEMP_L, &response_[0], 2);
  temperature_ = static_cast<float>(((int16_t)response_[1] << 8) | response_[0]) / 256. + 25.;
}

void LSM9DS1::updateAccelerometer()
{
  readRegsImu(XG_OUT_X_L_XL, &response_[0], 6);
  for (size_t i = 0; i < 3; ++i)
    bit_data_[i] = ((int16_t)response_[2 * i + 1] << 8) | response_[2 * i];

  ax_ = -tobas_std::kGravity * (static_cast<float>(bit_data_[1]) * acc_scale_);
  ay_ = -tobas_std::kGravity * (static_cast<float>(bit_data_[0]) * acc_scale_);
  az_ = tobas_std::kGravity * (static_cast<float>(bit_data_[2]) * acc_scale_);
}

void LSM9DS1::updateGyroscope()
{
  readRegsImu(XG_OUT_X_L_G, &response_[0], 6);
  for (size_t i = 0; i < 3; ++i)
    bit_data_[i] = ((int16_t)response_[2 * i + 1] << 8) | response_[2 * i];

  gx_ = -tobas_std::deg2rad(static_cast<float>(bit_data_[1]) * gyro_scale_);
  gy_ = -tobas_std::deg2rad(static_cast<float>(bit_data_[0]) * gyro_scale_);
  gz_ = tobas_std::deg2rad(static_cast<float>(bit_data_[2]) * gyro_scale_);
}

void LSM9DS1::updateMagnetometer()
{
  readRegsMag(M_OUT_X_L_M, &response_[0], 6);
  for (size_t i = 0; i < 3; ++i)
    bit_data_[i] = ((int16_t)response_[2 * i + 1] << 8) | response_[2 * i];

  mx_ = 100. * (static_cast<float>(bit_data_[0]) * mag_scale_);
  my_ = -100. * (static_cast<float>(bit_data_[1]) * mag_scale_);
  mz_ = -100. * (static_cast<float>(bit_data_[2]) * mag_scale_);
}

uint8_t LSM9DS1::writeReg(linux::SPIdev& spi_dev, const uint8_t& write_addr, const uint8_t& write_data)
{
  uint8_t tx[2] = { write_addr, write_data };
  uint8_t rx[2] = { 0 };
  spi_dev.transfer(tx, rx, 2);
  return rx[1];
}

uint8_t LSM9DS1::readReg(linux::SPIdev& spi_dev, const uint8_t& read_addr)
{
  return writeReg(spi_dev, read_addr | kReadFlag, 0x00);
}

void LSM9DS1::readRegsImu(const uint8_t& read_addr, uint8_t* read_buf, const uint32_t& bytes)
{
  tx_[0] = read_addr | kReadFlag;
  spi_dev_imu_.transfer(tx_, rx_, bytes + 1);

  for (size_t i = 0; i < bytes; ++i)
    read_buf[i] = rx_[i + 1];
}

void LSM9DS1::readRegsMag(const uint8_t& read_addr, uint8_t* read_buf, const uint32_t& bytes)
{
  tx_[0] = read_addr | kReadFlag | kMultipleRead;
  spi_dev_mag_.transfer(tx_, rx_, bytes + 1);

  for (size_t i = 0; i < bytes; ++i)
    read_buf[i] = rx_[i + 1];
}

void LSM9DS1::initializeGyroscope()
{
  // constexpr uint8_t scale = BITS_FS_G_2000DPS;
  constexpr uint8_t scale = BITS_FS_G_500DPS;

  // Enable the 3-axes of the gyroscope
  writeReg(spi_dev_imu_, XG_CTRL_REG4, BITS_XEN_G | BITS_YEN_G | BITS_ZEN_G);

  // ジャイロに関してはノイズより遅延のほうが怖いためフィルターを最小限にする
  // cf. PX4 | MC Filter Tuning & Control Latency: https://docs.px4.io/main/en/config_mc/filter_tuning.html
  writeReg(spi_dev_imu_, XG_CTRL_REG2_G, BITS_INT_SEL_LPF1 | BITS_OUT_SEL_LPF1);

  // サンプリング周波数を最大に設定
  writeReg(spi_dev_imu_, XG_CTRL_REG1_G, BITS_ODR_G_952HZ | scale);  // LPF1のみだからカットオフ100Hzで固定

  // Set scale
  setGyroScale(scale);

  tobas_std::usleep(kInitSleep);
}

void LSM9DS1::initializeAccelerometer()
{
  // constexpr uint8_t scale = BITS_FS_XL_16G;
  constexpr uint8_t scale = BITS_FS_XL_4G;

  // Enable the three axes of the accelerometer
  writeReg(spi_dev_imu_, XG_CTRL_REG5_XL, BITS_DEC_1 | BITS_XEN_XL | BITS_YEN_XL | BITS_ZEN_XL);

  // 最大レートで最小のカットオフ周波数に設定
  writeReg(spi_dev_imu_, XG_CTRL_REG6_XL, BITS_ODR_XL_952HZ | scale | BITS_BW_SCAL_ODR | BITS_BW_XL_50HZ);

  // 高解像度モード (デジタルローパスフィルタ) の設定
  // 遅延が怖いので使わないほうがいいかも
  if (kUseHighResolutionMode)
    writeReg(spi_dev_imu_, XG_CTRL_REG7_XL, BITS_HR | BITS_DCF_50 | BITS_FDS);

  // Set scale
  setAccScale(scale);

  tobas_std::usleep(kInitSleep);
}

void LSM9DS1::initializeMagnetometer()
{
  // constexpr uint8_t scale = BITS_FS_M_16Gs;
  constexpr uint8_t scale = BITS_FS_4GAUSS;

  // Configure magnetometer
  writeReg(spi_dev_mag_, M_CTRL_REG1_M, BITS_TEMP_COMP | BITS_OM_HIGH | BITS_DO_M_80HZ);
  writeReg(spi_dev_mag_, M_CTRL_REG2_M, scale);
  writeReg(spi_dev_mag_, M_CTRL_REG3_M, BITS_MD_CONTINUOUS);
  writeReg(spi_dev_mag_, M_CTRL_REG4_M, BITS_OMZ_HIGH);
  writeReg(spi_dev_mag_, M_CTRL_REG5_M, 0x00);

  // Set scale
  setMagScale(scale);

  tobas_std::usleep(kInitSleep);
}

void LSM9DS1::setGyroScale(const uint8_t& scale)
{
  switch (scale)
  {
    case BITS_FS_G_245DPS:
      gyro_scale_ = 0.00875;
      break;
    case BITS_FS_G_500DPS:
      gyro_scale_ = 0.0175;
      break;
    case BITS_FS_G_2000DPS:
      gyro_scale_ = 0.07;
      break;
    default:
      throw;
  }
}

void LSM9DS1::setAccScale(const uint8_t& scale)
{
  switch (scale)
  {
    case BITS_FS_XL_2G:
      acc_scale_ = 0.000061;
      break;
    case BITS_FS_XL_4G:
      acc_scale_ = 0.000122;
      break;
    case BITS_FS_XL_8G:
      acc_scale_ = 0.000244;
      break;
    case BITS_FS_XL_16G:
      acc_scale_ = 0.000732;
      break;
    default:
      throw;
  }
}

void LSM9DS1::setMagScale(const uint8_t& scale)
{
  switch (scale)
  {
    case BITS_FS_4GAUSS:
      mag_scale_ = 0.00014;
      break;
    case BITS_FS_8GAUSS:
      mag_scale_ = 0.00029;
      break;
    case BITS_FS_12GAUSS:
      mag_scale_ = 0.00043;
      break;
    case BITS_FS_16GAUSS:
      mag_scale_ = 0.00058;
      break;
    default:
      throw;
  }
}
}  // namespace navio
