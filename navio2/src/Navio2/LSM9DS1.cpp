#include <cmath>

#include "../../include/Navio2/LSM9DS1.h"

#define DEVICE_ACC_GYRO "/dev/spidev0.3"
#define DEVICE_MAG "/dev/spidev0.2"

#define READ_FLAG 0x80
#define MULTIPLE_READ 0x40

#define G_SI 9.80665
#define DEG2RAD (M_PI / 180.)
#define INITIALIZE_SLEEP 200  // [us]

LSM9DS1::LSM9DS1()
  : spi_dev_imu_(DEVICE_ACC_GYRO, kSpiSpeedHz), spi_dev_mag_(DEVICE_MAG, kSpiSpeedHz)
{
}

uint8_t LSM9DS1::writeReg(SPIdev& spi_dev, const uint8_t& write_addr, const uint8_t& write_data)
{
  uint8_t tx[2] = { write_addr, write_data };
  uint8_t rx[2] = { 0 };
  spi_dev.transfer(tx, rx, 2);
  return rx[1];
}

uint8_t LSM9DS1::readReg(SPIdev& spi_dev, const uint8_t& read_addr)
{
  return writeReg(spi_dev, read_addr | READ_FLAG, 0x00);
}

void LSM9DS1::readRegsImu(const uint8_t& read_addr, uint8_t* read_buf, const uint32_t& bytes)
{
  tx_[0] = read_addr | READ_FLAG;
  spi_dev_imu_.transfer(tx_, rx_, bytes + 1);

  for (size_t i = 0; i < bytes; ++i)
    read_buf[i] = rx_[i + 1];
}

void LSM9DS1::readRegsMag(const uint8_t& read_addr, uint8_t* read_buf, const uint32_t& bytes)
{
  tx_[0] = read_addr | READ_FLAG | MULTIPLE_READ;
  spi_dev_mag_.transfer(tx_, rx_, bytes + 1);

  for (size_t i = 0; i < bytes; ++i)
    read_buf[i] = rx_[i + 1];
}

bool LSM9DS1::probe()
{
  const auto response_xg = readReg(spi_dev_imu_, XG_WHO_AM_I);
  const auto response_m = readReg(spi_dev_mag_, M_WHO_AM_I);
  return response_xg == WHO_AM_I_ACC_GYRO && response_m == WHO_AM_I_MAG;
}

void LSM9DS1::initialize()
{
  initializeGyroscope();
  initializeAccelerometer();
  initializeMagnetometer();
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
  temperature = (float)(((int16_t)response_[1] << 8) | response_[0]) / 256. + 25.;
}

void LSM9DS1::updateAccelerometer()
{
  readRegsImu(XG_OUT_X_L_XL, &response_[0], 6);
  for (size_t i = 0; i < 3; ++i)
  {
    bit_data_[i] = ((int16_t)response_[2 * i + 1] << 8) | response_[2 * i];
  }

  ax_ = -G_SI * ((float)bit_data_[1] * acc_scale_);
  ay_ = -G_SI * ((float)bit_data_[0] * acc_scale_);
  az_ = G_SI * ((float)bit_data_[2] * acc_scale_);
}

void LSM9DS1::updateGyroscope()
{
  readRegsImu(XG_OUT_X_L_G, &response_[0], 6);
  for (size_t i = 0; i < 3; ++i)
  {
    bit_data_[i] = ((int16_t)response_[2 * i + 1] << 8) | response_[2 * i];
  }

  gx_ = -DEG2RAD * ((float)bit_data_[1] * gyro_scale_);
  gy_ = -DEG2RAD * ((float)bit_data_[0] * gyro_scale_);
  gz_ = DEG2RAD * ((float)bit_data_[2] * gyro_scale_);
}

void LSM9DS1::updateMagnetometer()
{
  readRegsMag(M_OUT_X_L_M, &response_[0], 6);
  for (size_t i = 0; i < 3; ++i)
  {
    bit_data_[i] = ((int16_t)response_[2 * i + 1] << 8) | response_[2 * i];
  }

  mx_ = 100. * ((float)bit_data_[0] * mag_scale_);
  my_ = -100. * ((float)bit_data_[1] * mag_scale_);
  mz_ = -100. * ((float)bit_data_[2] * mag_scale_);
}

void LSM9DS1::initializeGyroscope()
{
  // constexpr uint8_t scale = BITS_FS_G_2000DPS;
  constexpr uint8_t scale = BITS_FS_G_500DPS;

  // Enable the 3-axes of the gyroscope
  writeReg(spi_dev_imu_, XG_CTRL_REG4, BITS_XEN_G | BITS_YEN_G | BITS_ZEN_G);

  // Configure gyroscope
  writeReg(spi_dev_imu_, XG_CTRL_REG1_G, BITS_ODR_G_952HZ | scale | BITS_BW_G_0);

  // Set scale
  setGyroScale(scale);

  usleep(INITIALIZE_SLEEP);
}

void LSM9DS1::initializeAccelerometer()
{
  // constexpr uint8_t scale = BITS_FS_XL_16G;
  constexpr uint8_t scale = BITS_FS_XL_4G;

  // Enable the three axes of the accelerometer
  writeReg(spi_dev_imu_, XG_CTRL_REG5_XL, BITS_XEN_XL | BITS_YEN_XL | BITS_ZEN_XL);

  // Configure accelerometer
  writeReg(
    spi_dev_imu_, XG_CTRL_REG6_XL, BITS_ODR_XL_952HZ | scale | BITS_BW_SCAL_ODR | BITS_BW_XL_50HZ);
  writeReg(spi_dev_imu_, XG_CTRL_REG7_XL, BITS_HR | BITS_DCF_50);  // HR mode (Digital LPF)

  // Set scale
  setAccScale(scale);

  usleep(INITIALIZE_SLEEP);
}

void LSM9DS1::initializeMagnetometer()
{
  // constexpr uint8_t scale = BITS_FS_M_16Gs;
  constexpr uint8_t scale = BITS_FS_M_4Gs;

  // Configure magnetometer
  writeReg(spi_dev_mag_, M_CTRL_REG1_M, BITS_TEMP_COMP | BITS_OM_HIGH | BITS_ODR_M_80HZ);
  writeReg(spi_dev_mag_, M_CTRL_REG2_M, scale);
  writeReg(spi_dev_mag_, M_CTRL_REG3_M, BITS_MD_CONTINUOUS);
  writeReg(spi_dev_mag_, M_CTRL_REG4_M, BITS_OMZ_HIGH);
  writeReg(spi_dev_mag_, M_CTRL_REG5_M, 0x00);

  // Set scale
  setMagScale(scale);

  usleep(INITIALIZE_SLEEP);
}

void LSM9DS1::setGyroScale(uint8_t scale)
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
  }
}

void LSM9DS1::setAccScale(uint8_t scale)
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
  }
}

void LSM9DS1::setMagScale(uint8_t scale)
{
  switch (scale)
  {
    case BITS_FS_M_4Gs:
      mag_scale_ = 0.00014;
      break;
    case BITS_FS_M_8Gs:
      mag_scale_ = 0.00029;
      break;
    case BITS_FS_M_12Gs:
      mag_scale_ = 0.00043;
      break;
    case BITS_FS_M_16Gs:
      mag_scale_ = 0.00058;
      break;
  }
}
