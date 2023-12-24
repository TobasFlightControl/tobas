#pragma once

#include "./Common/SPIdev.h"
#include "./Common/InertialSensor.h"

/**
 * @brief datasheet: https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
 */
class LSM9DS1 : public InertialSensor
{
  static constexpr uint32_t kSpiSpeedHz = 10000000;  // Maximum frequency is 10MHz

public:
  explicit LSM9DS1();

  void initialize() override;
  bool probe() override;
  void update() override;

  void updateTemperature();
  void updateAccelerometer();
  void updateGyroscope();
  void updateMagnetometer();

private:
  enum who_am_i_t : uint8_t
  {
    WHO_AM_I_ACC_GYRO = 0x68,
    WHO_AM_I_MAG = 0x3D,
  };

  enum registers_t : uint8_t
  {
    XG_ACT_THS = 0x04,
    XG_ACT_DUR = 0x05,
    XG_INT_GEN_CFG_XL = 0x06,
    XG_INT_GEN_THS_X_XL = 0x07,
    XG_INT_GEN_THS_Y_XL = 0x08,
    XG_INT_GEN_THS_Z_XL = 0x09,
    XG_INT_GEN_DUR_XL = 0x0A,
    XG_REFERENCE_G = 0x0B,
    XG_INT1_CTRL = 0x0C,
    XG_INT2_CTRL = 0x0D,
    XG_WHO_AM_I = 0x0F,  // should return 0x68
    XG_CTRL_REG1_G = 0x10,
    XG_CTRL_REG2_G = 0x11,
    XG_CTRL_REG3_G = 0x12,
    XG_ORIENT_CFG_G = 0x13,
    XG_INT_GEN_SRC_G = 0x14,
    XG_OUT_TEMP_L = 0x15,
    XG_OUT_TEMP_H = 0x16,
    XG_STATUS_REG = 0x17,
    XG_OUT_X_L_G = 0x18,
    XG_OUT_X_H_G = 0x19,
    XG_OUT_Y_L_G = 0x1A,
    XG_OUT_Y_H_G = 0x1B,
    XG_OUT_Z_L_G = 0x1C,
    XG_OUT_Z_H_G = 0x1D,
    XG_CTRL_REG4 = 0x1E,
    XG_CTRL_REG5_XL = 0x1F,
    XG_CTRL_REG6_XL = 0x20,
    XG_CTRL_REG7_XL = 0x21,
    XG_CTRL_REG8 = 0x22,
    XG_CTRL_REG9 = 0x23,
    XG_CTRL_REG10 = 0x24,
    XG_INT_GEN_SRC_XL = 0x26,
    XG_OUT_X_L_XL = 0x28,
    XG_OUT_X_H_XL = 0x29,
    XG_OUT_Y_L_XL = 0x2A,
    XG_OUT_Y_H_XL = 0x2B,
    XG_OUT_Z_L_XL = 0x2C,
    XG_OUT_Z_H_XL = 0x2D,
    XG_FIFO_CTRL = 0x2E,
    XG_FIFO_SRC = 0x2F,
    XG_INT_GEN_CFG_G = 0x30,
    XG_INT_GEN_THS_XH_G = 0x31,
    XG_INT_GEN_THS_XL_G = 0x32,
    XG_INT_GEN_THS_YH_G = 0x33,
    XG_INT_GEN_THS_YL_G = 0x34,
    XG_INT_GEN_THS_ZH_G = 0x35,
    XG_INT_GEN_THS_ZL_G = 0x36,
    XG_INT_GEN_DUR_G = 0x37,

    M_OFFSET_X_REG_L_M = 0x05,
    M_OFFSET_X_REG_H_M = 0x06,
    M_OFFSET_Y_REG_L_M = 0x07,
    M_OFFSET_Y_REG_H_M = 0x08,
    M_OFFSET_Z_REG_L_M = 0x09,
    M_OFFSET_Z_REG_H_M = 0x0A,
    M_WHO_AM_I = 0x0F,  // should return 0x3D
    M_CTRL_REG1_M = 0x20,
    M_CTRL_REG2_M = 0x21,
    M_CTRL_REG3_M = 0x22,
    M_CTRL_REG4_M = 0x23,
    M_CTRL_REG5_M = 0x24,
    M_STATUS_REG_M = 0x27,
    M_OUT_X_L_M = 0x28,
    M_OUT_X_H_M = 0x29,
    M_OUT_Y_L_M = 0x2A,
    M_OUT_Y_H_M = 0x2B,
    M_OUT_Z_L_M = 0x2C,
    M_OUT_Z_H_M = 0x2D,
    M_INT_CFG_M = 0x30,
    M_INT_SRC_M = 0x31,
    M_INT_THS_L_M = 0x32,
    M_INT_THS_H_M = 0x33,
  };

  enum gyro_config_t : uint8_t
  {
    BITS_XEN_G = 0x08,
    BITS_YEN_G = 0x10,
    BITS_ZEN_G = 0x20,
    BITS_ODR_G_14900mHZ = 0x20,
    BITS_ODR_G_59500mHZ = 0x40,
    BITS_ODR_G_119HZ = 0b011 << 5,
    BITS_ODR_G_238HZ = 0b100 << 5,
    BITS_ODR_G_476HZ = 0b101 << 5,
    BITS_ODR_G_952HZ = 0b110 << 5,
    BITS_FS_G_245DPS = 0x00,
    BITS_FS_G_500DPS = 0x08,
    BITS_FS_G_2000DPS = 0x18,
    BITS_BW_G_0 = 0b00,  // Minimum cutoff frequency
    BITS_BW_G_1 = 0b01,
    BITS_BW_G_2 = 0b10,
    BITS_BW_G_3 = 0b11,  // Maximum cutoff frequency
  };

  enum acc_config_t : uint8_t
  {
    BITS_XEN_XL = 0x08,
    BITS_YEN_XL = 0x10,
    BITS_ZEN_XL = 0x20,
    BITS_ODR_XL_10HZ = 0x20,
    BITS_ODR_XL_50HZ = 0x40,
    BITS_ODR_XL_119HZ = 0x60,
    BITS_ODR_XL_238HZ = 0x80,
    BITS_ODR_XL_476HZ = 0xA0,
    BITS_ODR_XL_952HZ = 0xC0,
    BITS_FS_XL_2G = 0x00,
    BITS_FS_XL_4G = 0x10,
    BITS_FS_XL_8G = 0x18,
    BITS_FS_XL_16G = 0x08,
    BITS_BW_SCAL_ODR = 1 << 2,
    BITS_BW_XL_50HZ = 0b11,
    BITS_BW_XL_105HZ = 0b10,
    BITS_BW_XL_211HZ = 0b01,
    BITS_BW_XL_408HZ = 0b00,
    BITS_HR = 1 << 7,
    BITS_DCF_9 = 0b10 << 5,
    BITS_DCF_50 = 0b00 << 5,
    BITS_DCF_100 = 0b01 << 5,
    BITS_DCF_400 = 0b11 << 5,
  };

  enum mag_config_t : uint8_t
  {
    BITS_TEMP_COMP = 0x80,
    BITS_OM_LOW = 0x00,
    BITS_OM_MEDIUM = 0x20,
    BITS_OM_HIGH = 0x40,
    BITS_OM_ULTRA_HIGH = 0x60,
    BITS_ODR_M_625mHZ = 0x00,
    BITS_ODR_M_1250mHZ = 0x04,
    BITS_ODR_M_250mHZ = 0x08,
    BITS_ODR_M_5HZ = 0x0C,
    BITS_ODR_M_10HZ = 0x10,
    BITS_ODR_M_20HZ = 0x14,
    BITS_ODR_M_40HZ = 0x18,
    BITS_ODR_M_80HZ = 0x1C,
    BITS_FS_M_4Gs = 0x00,
    BITS_FS_M_8Gs = 0x20,
    BITS_FS_M_12Gs = 0x40,
    BITS_FS_M_16Gs = 0x60,
    BITS_MD_CONTINUOUS = 0x00,
    BITS_MD_SINGLE = 0x01,
    BITS_MD_POWERDOWN = 0x02,
    BITS_OMZ_LOW = 0x00,
    BITS_OMZ_MEDIUM = 0x04,
    BITS_OMZ_HIGH = 0x08,
    BITS_OMZ_ULTRA_HIGH = 0x0C,
  };

  uint8_t writeReg(SPIdev& spi_dev, const uint8_t& write_addr, const uint8_t& write_data);
  uint8_t readReg(SPIdev& spi_dev, const uint8_t& read_addr);
  void readRegsImu(const uint8_t& read_addr, uint8_t* read_buf, const uint32_t& bytes);
  void readRegsMag(const uint8_t& read_addr, uint8_t* read_buf, const uint32_t& bytes);

  void initializeGyroscope();
  void initializeAccelerometer();
  void initializeMagnetometer();

  void setGyroScale(uint8_t scale);
  void setAccScale(uint8_t scale);
  void setMagScale(uint8_t scale);

  SPIdev spi_dev_imu_;
  SPIdev spi_dev_mag_;

  float gyro_scale_;
  float acc_scale_;
  float mag_scale_;

  uint8_t tx_[255] = { 0 };
  uint8_t rx_[255] = { 0 };
  uint8_t response_[6];
  int16_t bit_data_[3];
};
