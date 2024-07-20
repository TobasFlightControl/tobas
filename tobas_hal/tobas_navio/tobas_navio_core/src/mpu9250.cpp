#include <cassert>

#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_std_tools/time.hpp>

#include "../include/tobas_navio_core/mpu9250.hpp"

namespace navio
{
MPU9250::MPU9250()
{
}

bool MPU9250::initialize()
{
  if (!spi_dev_.initialize(kDevice, kSpiClockFreq, kSpiBufSize))
    return false;

  if (!probe())
    return false;

  uint8_t MPU_Init_Data[kInitRegNum][2] = {
    //{0x80, MPUREG_PWR_MGMT_1},     // Reset Device - Disabled because it seems to corrupt
    // initialisation of AK8963
    { 0x01, MPUREG_PWR_MGMT_1 },      // Clock Source
    { 0x00, MPUREG_PWR_MGMT_2 },      // Enable Acc & Gyro
    { 0x00, MPUREG_CONFIG },          // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
    { 0x18, MPUREG_GYRO_CONFIG },     // +-2000dps
    { 3 << 3, MPUREG_ACCEL_CONFIG },  // +-16G
    { 0x08, MPUREG_ACCEL_CONFIG_2 },  // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
    { 0x30, MPUREG_INT_PIN_CFG },     //
    //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
    //{0x20, MPUREG_USER_CTRL},      // Enable AUX
    { 0x20, MPUREG_USER_CTRL },     // I2C Master mode
    { 0x0D, MPUREG_I2C_MST_CTRL },  //  I2C configuration multi-master  IIC 400KHz

    { AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR },  // Set the I2C slave addres of AK8963 and set for
                                                // write.
    //{0x09, MPUREG_I2C_SLV4_CTRL},
    //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

    { AK8963_CNTL2, MPUREG_I2C_SLV0_REG },  // I2C slave 0 register address from where to begin data transfer
    { 0x01, MPUREG_I2C_SLV0_DO },           // Reset AK8963
    { 0x81, MPUREG_I2C_SLV0_CTRL },         // Enable I2C and set 1 byte

    { AK8963_CNTL1, MPUREG_I2C_SLV0_REG },  // I2C slave 0 register address from where to begin data transfer
    { 0x12, MPUREG_I2C_SLV0_DO },           // Register value to continuous measurement in 16bit
    { 0x81, MPUREG_I2C_SLV0_CTRL }          // Enable I2C and set 1 byte

  };

  // setAccScale(BITS_FS_16G);
  // setGyroScale(BITS_FS_2000DPS);
  setAccScale(BITS_FS_4G);
  setGyroScale(BITS_FS_500DPS);

  for (size_t i = 0; i < kInitRegNum; ++i)
  {
    writeReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
    tobas_std::msleep(100);  // I2C must slow down the write speed, otherwise it won't work
  }

  calibMag();

  return true;
}

void MPU9250::update()
{
  uint8_t response[21];
  int16_t bit_data[3];

  // Send I2C command at first
  writeReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
  writeReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);  // I2C slave 0 register address from where to begin data transfer
  writeReg(MPUREG_I2C_SLV0_CTRL, 0x87);       // Read 7 bytes from the magnetometer
  // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2
  // register 0x09 the AK8963A will unlatch the data registers for the next measurement.

  readRegs(MPUREG_ACCEL_XOUT_H, response, 21);

  // Get accelerometer value
  for (size_t i = 0; i < 3; ++i)
    bit_data[i] = ((int16_t)response[i * 2] << 8) | response[i * 2 + 1];
  ax_ = tobas_std::kGravity * static_cast<float>(bit_data[0]) / acc_divider_;
  ay_ = tobas_std::kGravity * static_cast<float>(bit_data[1]) / acc_divider_;
  az_ = tobas_std::kGravity * static_cast<float>(bit_data[2]) / acc_divider_;

  // Get temperature
  bit_data[0] = ((int16_t)response[3 * 2] << 8) | response[3 * 2 + 1];
  temperature_ = ((bit_data[0] - 21) / 333.87) + 21;

  // Get gyroscope value
  for (size_t i = 4; i < 7; ++i)
    bit_data[i - 4] = ((int16_t)response[i * 2] << 8) | response[i * 2 + 1];
  gx_ = tobas_std::deg2rad(static_cast<float>(bit_data[0]) / gyro_divider_);
  gy_ = tobas_std::deg2rad(static_cast<float>(bit_data[1]) / gyro_divider_);
  gz_ = tobas_std::deg2rad(static_cast<float>(bit_data[2]) / gyro_divider_);

  // Get Magnetometer value
  for (size_t i = 7; i < 10; ++i)
    bit_data[i - 7] = ((int16_t)response[i * 2 + 1] << 8) | response[i * 2];
  mx_ = static_cast<float>(bit_data[0]) * magnetometer_ASA_[0];
  my_ = static_cast<float>(bit_data[1]) * magnetometer_ASA_[1];
  mz_ = static_cast<float>(bit_data[2]) * magnetometer_ASA_[2];
}

bool MPU9250::probe()
{
  uint8_t responseXG, responseM;

  responseXG = readReg(MPUREG_WHOAMI | READ_FLAG);

  writeReg(MPUREG_USER_CTRL, 0x20);                             // I2C Master mode
  writeReg(MPUREG_I2C_MST_CTRL, 0x0D);                          // I2C configuration multi-master  IIC 400KHz
  writeReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
  writeReg(MPUREG_I2C_SLV0_REG, AK8963_WIA);  // I2C slave 0 register address from where to begin data transfer
  writeReg(MPUREG_I2C_SLV0_CTRL, 0x81);       // Read 1 byte from the magnetometer
  tobas_std::msleep(10);
  responseM = readReg(MPUREG_EXT_SENS_DATA_00);

  return responseXG == 0x71 && responseM == 0x48;
}

uint8_t MPU9250::writeReg(uint8_t addr, uint8_t data)
{
  spi_dev_.tx[0] = addr;
  spi_dev_.tx[1] = data;
  spi_dev_.transfer(2);
  return spi_dev_.rx[1];
}

uint8_t MPU9250::readReg(uint8_t addr)
{
  return writeReg(addr | READ_FLAG, 0x00);
}

void MPU9250::readRegs(uint8_t addr, uint8_t* buf, size_t bytes)
{
  spi_dev_.tx[0] = addr | READ_FLAG;
  spi_dev_.transfer(bytes + 1);
  for (size_t i = 0; i < bytes; ++i)
    buf[i] = spi_dev_.rx[i + 1];
}

void MPU9250::setAccScale(uint8_t scale)
{
  writeReg(MPUREG_ACCEL_CONFIG, scale);

  switch (scale)
  {
    case BITS_FS_2G:
      acc_divider_ = 16384;
      break;
    case BITS_FS_4G:
      acc_divider_ = 8192;
      break;
    case BITS_FS_8G:
      acc_divider_ = 4096;
      break;
    case BITS_FS_16G:
      acc_divider_ = 2048;
      break;
  }
}

void MPU9250::setGyroScale(uint8_t scale)
{
  writeReg(MPUREG_GYRO_CONFIG, scale);

  switch (scale)
  {
    case BITS_FS_250DPS:
      gyro_divider_ = 131;
      break;
    case BITS_FS_500DPS:
      gyro_divider_ = 65.5;
      break;
    case BITS_FS_1000DPS:
      gyro_divider_ = 32.8;
      break;
    case BITS_FS_2000DPS:
      gyro_divider_ = 16.4;
      break;
  }
}

void MPU9250::calibAcc()
{
  uint8_t response[4];
  // read current acc scale
  const auto temp_scale = writeReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);
  setAccScale(BITS_FS_8G);
  // ENABLE SELF TEST need modify
  // temp_scale=writeReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

  readRegs(MPUREG_SELF_TEST_X, response, 4);
  calib_data_[0] = ((response[0] & 11100000) >> 3) | ((response[3] & 00110000) >> 4);
  calib_data_[1] = ((response[1] & 11100000) >> 3) | ((response[3] & 00001100) >> 2);
  calib_data_[2] = ((response[2] & 11100000) >> 3) | ((response[3] & 00000011));

  setAccScale(temp_scale);
}

void MPU9250::calibMag()
{
  uint8_t response[3];
  writeReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
  writeReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX);  // I2C slave 0 register address from where to begin data transfer
  writeReg(MPUREG_I2C_SLV0_CTRL, 0x83);        // Read 3 bytes from the magnetometer

  // writeReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
  tobas_std::msleep(10);
  // response[0]=writeReg(MPUREG_EXT_SENS_DATA_01 | READ_FLAG, 0x00);    //Read I2C
  readRegs(MPUREG_EXT_SENS_DATA_00, response, 3);

  // response=writeReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C
  for (size_t i = 0; i < 3; ++i)
  {
    const auto& data = response[i];
    magnetometer_ASA_[i] = ((data - 128) / 256 + 1) * Magnetometer_Sensitivity_Scale_Factor;
  }
}
}  // namespace navio
