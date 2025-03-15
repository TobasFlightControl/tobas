#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace driver
{
/**
 * @brief A linux driver of 6-axis IMU.
 *
 * Datasheet: https://www.st.com/resource/en/datasheet/ism330dlc.pdf
 */
class ISM330DLC
{
public:
  enum struct odr_xl_t : uint8_t
  {
    ODR_XL_26HZ,
    ODR_XL_52HZ,
    ODR_XL_104HZ,
    ODR_XL_208HZ,
    ODR_XL_416HZ,
    ODR_XL_833HZ,
    ODR_XL_1660HZ,
    ODR_XL_3330HZ,
    ODR_XL_6660HZ,
  };

  enum struct fs_xl_t : uint8_t
  {
    FS_XL_2G,
    FS_XL_4G,
    FS_XL_8G,
    FS_XL_16G,
  };

  enum struct odr_g_t : uint8_t
  {
    ODR_G_26HZ,
    ODR_G_52HZ,
    ODR_G_104HZ,
    ODR_G_208HZ,
    ODR_G_416HZ,
    ODR_G_833HZ,
    ODR_G_1660HZ,
    ODR_G_3330HZ,
    ODR_G_6660HZ,
  };

  enum struct fs_g_t : uint8_t
  {
    FS_G_125DPS,
    FS_G_250DPS,
    FS_G_500DPS,
    FS_G_1000DPS,
    FS_G_2000DPS,
  };

  explicit ISM330DLC();

  bool initialize(const char* spi_device);

  bool setAccelOutputDataRate(odr_xl_t odr);
  bool setGyroOutputDataRate(odr_g_t odr);

  bool setAccelFullScale(fs_xl_t fs);
  bool setGyroFullScale(fs_g_t fs);

  /* Read the current acceleration [m/s^2]. */
  bool readAccel(double& ax, double& ay, double& az);

  /* Read the current gyro [rad/s]. */
  bool readGyro(double& gx, double& gy, double& gz);

private:
  static constexpr size_t kSPIBufSize = 6 + 1;
  static constexpr uint32_t kSPIClockFreq = 10'000'000;  // Maximum frequency is 10MHz
  static constexpr uint8_t kReadFlag = 0x80;

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

  enum ctrl1_xl_t : uint8_t
  {
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
  };

  enum ctrl2_g_t : uint8_t
  {

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
  };

  enum ctrl3_c_t : uint8_t
  {
    // TODO
  };

  enum ctrl4_c_t : uint8_t
  {

    DEN_XL_EN = 1 << 7,
    SLEEP = 1 << 6,
    INT2_ON_INT1 = 1 << 5,
    DEN_DRDY_INT1 = 1 << 4,
    DRDY_MASK = 1 << 3,
    I2C_DISABLE = 1 << 2,
    LPF1_SEL_G = 1 << 1,
  };

  enum ctrl5_c_t : uint8_t
  {
    // TODO
  };

  enum ctrl6_c_t : uint8_t
  {

    TRIG_EN = 1 << 7,
    LVL1_EN = 1 << 6,
    LVL2_EN = 1 << 5,
    XL_HM_MODE_DISABLE = 1 << 4,
    USR_OFF_W = 1 << 3,
    FTYPE_0 = 0b10,
    FTYPE_1 = 0b01,
    FTYPE_2 = 0b00,
    FTYPE_3 = 0b11,
  };

  enum ctrl7_g_t : uint8_t
  {

    G_HM_MODE_DISABLE = 1 << 7,
    HP_EN_G = 1 << 6,
    HPM_G_16MHZ = 00 << 4,
    HPM_G_65MHZ = 01 << 4,
    HPM_G_260MHZ = 10 << 4,
    HPM_G_1040MHZ = 11 << 4,
    ROUNDING_STATUS = 1 << 2,
  };

  enum ctrl8_xl_t : uint8_t
  {

    LPF2_XL_EN = 1 << 7,
    HPCF_XL_50 = 0b00 << 5,   // fc = ODR/50
    HPCF_XL_100 = 0b01 << 5,  // fc = ODR/100
    HPCF_XL_9 = 0b10 << 5,    // fc = ODR/9
    HPCF_XL_400 = 0b11 << 5,  // fc = ODR/400
    HP_REF_MODE = 1 << 4,
    INPUT_COMPOSITE = 1 << 3,
    HP_SLOPE_XL_EN = 1 << 2,
    LOW_PASS_ON_6D = 1 << 0,
  };

  enum ctrl9_xl_t : uint8_t
  {
    // TODO
  };

  enum ctrl10_c_t : uint8_t
  {
    // TODO
  };

  linux::SPIdev spi_;
  uint8_t tx_buf_[kSPIBufSize];
  uint8_t rx_buf_[kSPIBufSize];

  double acc_scale_;   // LSB -> m/s^2
  double gyro_scale_;  // LSB -> rad/s

  uint8_t res_[6];  // The results of readRegs are stored.

  /* 6.5.1: SPI read (p.27) */
  bool readRegs(const uint8_t& addr, const size_t& bytes);

  /* 6.5.2: SPI write (p.28) */
  bool writeReg(const uint8_t& addr, const uint8_t& data);

  bool checkWhoAmI();
};
}  // namespace driver
