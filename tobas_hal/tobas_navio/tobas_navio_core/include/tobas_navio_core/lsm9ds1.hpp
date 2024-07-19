#pragma once

#include <tobas_linux/spi_dev.hpp>

#include "./imu.hpp"

namespace navio
{
/**
 * @brief Datasheet: https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
 */
class LSM9DS1 : public InertialSensor
{
  static constexpr char kAccGyroDevice[] = "/dev/spidev0.3";
  static constexpr char kMagDevice[] = "/dev/spidev0.2";
  static constexpr uint32_t kSpiClockFreq = 10'000'000;  // Maximum frequency is 10MHz
  static constexpr uint8_t kReadFlag = 0x80;
  static constexpr uint8_t kMultipleRead = 0x40;
  static constexpr size_t kInitSleep = 200;  // [us]
  static constexpr bool kUseHighResolutionMode = false;

public:
  explicit LSM9DS1();

  bool initialize() override;
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
    // CTRL_REG1_G
    BITS_ODR_G_119HZ = 0b011 << 5,
    BITS_ODR_G_238HZ = 0b100 << 5,
    BITS_ODR_G_476HZ = 0b101 << 5,
    BITS_ODR_G_952HZ = 0b110 << 5,
    BITS_FS_G_245DPS = 0b00 << 3,
    BITS_FS_G_500DPS = 0b01 << 3,
    BITS_FS_G_2000DPS = 0b11 << 3,
    BITS_BW_G_0 = 0b00 << 0,  // fc = 33Hz (ODR = 952Hz)
    BITS_BW_G_1 = 0b01 << 0,  // fc = 40Hz (ODR = 952Hz)
    BITS_BW_G_2 = 0b10 << 0,  // fc = 58Hz (ODR = 952Hz)
    BITS_BW_G_3 = 0b11 << 0,  // fc = 100Hz (ODR = 952Hz)

    // CTRL_REG2_G
    BITS_INT_SEL_LPF1 = 0b00 << 2,
    BITS_INT_SEL_LPF1_HPF = 0b01 << 2,
    BITS_INT_SEL_LPF1_HPF_LPF2 = 0b10 << 2,
    BITS_OUT_SEL_LPF1 = 0b00 << 0,
    BITS_OUT_SEL_LPF1_HPF = 0b01 << 0,
    BITS_OUT_SEL_LPF1_HPF_LPF2 = 0b10 << 0,

    // CTRL_REG4
    BITS_ZEN_G = 1 << 5,
    BITS_YEN_G = 1 << 4,
    BITS_XEN_G = 1 << 3,
    BITS_LIR_XL1 = 1 << 1,
    BITS_4D_XL1 = 1 << 0,
  };

  enum acc_config_t : uint8_t
  {
    // CTRL_REG5_XL
    BITS_DEC_1 = 0b00 << 6,  // No decimation
    BITS_DEC_2 = 0b01 << 6,  // Update every 2 samples
    BITS_DEC_4 = 0b10 << 6,  // Update every 2 samples
    BITS_DEC_8 = 0b11 << 6,  // Update every 2 samples
    BITS_ZEN_XL = 1 << 5,    // Accelerometer’s Z-axis output enable
    BITS_YEN_XL = 1 << 4,    // Accelerometer’s Y-axis output enable
    BITS_XEN_XL = 1 << 3,    // Accelerometer’s X-axis output enable

    // CTRL_REG6_XL
    BITS_ODR_XL_10HZ = 0b001 << 5,
    BITS_ODR_XL_50HZ = 0b010 << 5,
    BITS_ODR_XL_119HZ = 0b011 << 5,
    BITS_ODR_XL_238HZ = 0b100 << 5,
    BITS_ODR_XL_476HZ = 0b101 << 5,
    BITS_ODR_XL_952HZ = 0b110 << 5,
    BITS_FS_XL_2G = 0b00 << 3,
    BITS_FS_XL_4G = 0b10 << 3,
    BITS_FS_XL_8G = 0b11 << 3,
    BITS_FS_XL_16G = 0b01 << 3,
    BITS_BW_SCAL_ODR = 1 << 2,
    BITS_BW_XL_50HZ = 0b11 << 0,
    BITS_BW_XL_105HZ = 0b10 << 0,
    BITS_BW_XL_211HZ = 0b01 << 0,
    BITS_BW_XL_408HZ = 0b00 << 0,

    // CTRL_REG7_XL
    BITS_HR = 1 << 7,
    BITS_DCF_9 = 0b10 << 5,    // fc = ODR / 9
    BITS_DCF_50 = 0b00 << 5,   // fc = ODR / 50
    BITS_DCF_100 = 0b01 << 5,  // fc = ODR / 100
    BITS_DCF_400 = 0b11 << 5,  // fc = ODR / 400
    BITS_FDS = 1 << 2,
    BITS_HPIS1 = 1 << 0,
  };

  enum mag_config_t : uint8_t
  {
    // CTRL_REG1_M
    BITS_TEMP_COMP = 1 << 7,
    BITS_OM_LOW = 0b00 << 5,
    BITS_OM_MEDIUM = 0b01 << 5,
    BITS_OM_HIGH = 0b10 << 5,
    BITS_OM_ULTRA_HIGH = 0b11 << 5,
    BITS_DO_M_5HZ = 0b011 << 2,
    BITS_DO_M_10HZ = 0b100 << 2,
    BITS_DO_M_20HZ = 0b001 << 2,
    BITS_DO_M_40HZ = 0b110 << 2,
    BITS_DO_M_80HZ = 0b111 << 2,
    BITS_FAST_ODR = 1 << 1,
    BITS_ST = 1 << 0,

    // CTRL_REG2_M
    BITS_FS_4GAUSS = 0b00 << 5,
    BITS_FS_8GAUSS = 0b01 << 5,
    BITS_FS_12GAUSS = 0b10 << 5,
    BITS_FS_16GAUSS = 0b11 << 5,
    BITS_REBOOT = 1 << 3,
    BITS_SOFT_RST = 1 << 2,

    // CTRL_REG3_M
    BITS_I2C_DISABLE = 1 << 7,
    BITS_LP = 1 << 5,
    BITS_SIM = 1 << 2,
    BITS_MD_CONTINUOUS = 0b00 << 0,
    BITS_MD_SINGLE = 0b01 << 0,
    BITS_MD_POWERDOWN = 0b10 << 0,

    // CTRL_REG4_M
    BITS_OMZ_LOW = 0b00 << 2,
    BITS_OMZ_MEDIUM = 0b01 << 2,
    BITS_OMZ_HIGH = 0b10 << 2,
    BITS_OMZ_ULTRA_HIGH = 0b11 << 2,
    BITS_BLE = 1 << 1,

    // CTRL_REG5_M
    BITS_FAST_READ = 1 << 7,
    BITS_BDU = 1 << 6,
  };

  linux::SPIdev spi_dev_imu_;
  linux::SPIdev spi_dev_mag_;

  float gyro_scale_;
  float acc_scale_;
  float mag_scale_;

  uint8_t tx_[255] = { 0 };
  uint8_t rx_[255] = { 0 };
  uint8_t response_[6];
  int16_t bit_data_[3];

  uint8_t writeReg(linux::SPIdev& spi_dev, const uint8_t& write_addr, const uint8_t& write_data);
  uint8_t readReg(linux::SPIdev& spi_dev, const uint8_t& read_addr);
  void readRegsImu(const uint8_t& read_addr, uint8_t* read_buf, const uint32_t& bytes);
  void readRegsMag(const uint8_t& read_addr, uint8_t* read_buf, const uint32_t& bytes);

  void initializeGyroscope();
  void initializeAccelerometer();
  void initializeMagnetometer();

  void setGyroScale(const uint8_t& scale);
  void setAccScale(const uint8_t& scale);
  void setMagScale(const uint8_t& scale);
};
}  // namespace navio
