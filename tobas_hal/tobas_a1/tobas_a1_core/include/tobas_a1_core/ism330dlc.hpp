#pragma once

#include <tobas_linux/spi_dev.hpp>

namespace a1
{
/**
 * @brief Datasheet: https://www.st.com/resource/en/datasheet/ism330dlc.pdf
 */
class ISM330DLC
{
  static constexpr uint32_t kSpiClockFreq = 10'000'000;  // Maximum frequency is 10MHz
  static constexpr size_t kSpiBufSize = 6;
  static constexpr uint8_t kReadFlag = 0x80;

public:
  explicit ISM330DLC();

  bool initialize();

  bool readAcc(double& ax, double& ay, double& az);
  bool readGyro(double& gx, double& gy, double& gz);

private:
  /* 9: Register mapping (p.37) */
  enum register_t : uint8_t
  {
    // Who I am ID
    REG_WHO_AM_I = 0x0F,

    // Accelerometer and gyroscope control registers
    REG_CTRL1_XL = 0x10,
    REG_CTRL2_G = 0x11,
    REG_CTRL3_C = 0x12,
    REG_CTRL4_C = 0x13,
    REG_CTRL5_C = 0x14,
    REG_CTRL6_C = 0x15,
    REG_CTRL7_G = 0x16,
    REG_CTRL8_XL = 0x17,
    REG_CTRL9_XL = 0x18,
    REG_CTRL10_C = 0x19,

    // Temperature output data registers
    REG_OUT_TEMP_L = 0x20,
    REG_OUT_TEMP_H = 0x21,

    // Gyroscope output registers for GP and OIS data
    REG_OUTX_L_G = 0x22,
    REG_OUTX_H_G = 0x23,
    REG_OUTY_L_G = 0x24,
    REG_OUTY_H_G = 0x25,
    REG_OUTZ_L_G = 0x26,
    REG_OUTZ_H_G = 0x27,

    // Accelerometer output registers
    REG_OUTX_L_XL = 0x28,
    REG_OUTX_H_XL = 0x29,
    REG_OUTY_L_XL = 0x2A,
    REG_OUTY_H_XL = 0x2B,
    REG_OUTZ_L_XL = 0x2C,
    REG_OUTZ_H_XL = 0x2D,
  };

  enum who_am_i_t : uint8_t
  {
    WHO_AM_I = 0b01101010,
  };

  enum acc_config_t : uint8_t
  {
    // CTRL1_XL
    ODR_XL_26HZ = 0b0010 << 4,
    ODR_XL_52HZ = 0b0011 << 4,
    ODR_XL_104HZ = 0b0100 << 4,
    ODR_XL_208HZ = 0b0101 << 4,
    ODR_XL_416HZ = 0b0110 << 4,
    ODR_XL_833HZ = 0b0111 << 4,
    ODR_XL_1660HZ = 0b1000 << 4,
    ODR_XL_3330HZ = 0b1001 << 4,
    ODR_XL_6660HZ = 0b1010 << 4,
    FS_XL_2G = 0b00 << 2,
    FS_XL_4G = 0b10 << 2,
    FS_XL_8G = 0b11 << 2,
    FS_XL_16G = 0b01 << 2,
    LPF1_BW_SEL_2 = 0 << 1,  // fc = ODR/2
    LPF1_BW_SEL_4 = 1 << 1,  // fc = ODR/4
    BW0_XL_1500HZ = 0 << 0,
    BW0_XL_400HZ = 1 << 0,

    // CTRL2_G
    ODR_G_26HZ = 0b0010 << 4,
    ODR_G_52HZ = 0b0011 << 4,
    ODR_G_104HZ = 0b0100 << 4,
    ODR_G_208HZ = 0b0101 << 4,
    ODR_G_416HZ = 0b0110 << 4,
    ODR_G_833HZ = 0b0111 << 4,
    ODR_G_1660HZ = 0b1000 << 4,
    ODR_G_3330HZ = 0b1001 << 4,
    ODR_G_6660HZ = 0b1010 << 4,
    FS_G_125DPS = 0b001 << 1,
    FS_G_250DPS = 0b000 << 1,
    FS_G_500DPS = 0b010 << 1,
    FS_G_1000DPS = 0b100 << 1,
    FS_G_2000DPS = 0b110 << 1,

    // CTRL4_C
    I2C_DISABLE = 1 << 1,

    // TODO: デフォルトのままとしたControl Registerの内容をまとめる
  };

  linux::SPIdev spi_;

  double acc_scale_;   // LSB -> m/s^2
  double gyro_scale_;  // LSB -> rad/s

  uint8_t res_[kSpiBufSize - 1];  // The results of readRegs are stored.

  /* 6.5.1: SPI read (p.27) */
  bool readRegs(const uint8_t& addr, const size_t& bytes);

  /* 6.5.2: SPI write (p.28) */
  bool writeReg(const uint8_t& addr, const uint8_t& data);

  bool checkWhoAmI();
  bool configureAcc();
  bool configureGyro();

  void setAccScale(const uint8_t& scale);
  void setGyroScale(const uint8_t& scale);
};
}  // namespace a1
