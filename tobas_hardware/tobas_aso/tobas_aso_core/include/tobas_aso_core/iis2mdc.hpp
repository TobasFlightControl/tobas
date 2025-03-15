#pragma once

#include <tobas_linux/i2c_dev.hpp>

namespace aso
{
/**
 * @brief A linux driver of 3-axis magnetometer.
 *
 * Datasheet: https://www.st.com/resource/en/datasheet/iis2mdc.pdf
 */
class IIS2MDC
{
  static constexpr char kI2cDevice[] = "/dev/i2c-1";
  static constexpr uint8_t kI2cAddress = 0b0011110;
  static constexpr uint8_t kMultiReadFlag = 0x80;  // cf. 6.1.1: I2C operation (p.23)
  static constexpr double kSensitivity = 1.5e-3;   // [gauss/LSB]

public:
  explicit IIS2MDC();

  bool initialize();

  /* Read the current magnetic field [gauss]. */
  bool readMag(double& mx, double& my, double& mz);

private:
  /* 7: Register mapping (p.27) */
  enum register_t : uint8_t
  {
    // Who I am ID
    WHO_AM_I_REG = 0x4F,

    // Configuration registers
    CFG_REG_A = 0x60,
    CFG_REG_B = 0x61,
    CFG_REG_C = 0x62,

    // Output registers
    OUTX_L_REG = 0x68,
    OUTX_H_REG = 0x69,
    OUTY_L_REG = 0x6A,
    OUTY_H_REG = 0x6B,
    OUTZ_L_REG = 0x6C,
    OUTZ_H_REG = 0x6D,

    // Temperature sensor registers
    TEMP_OUT_L_REG = 0x6E,
    TEMP_OUT_H_REG = 0x6F,
  };

  enum who_am_i_t : uint8_t
  {
    WHO_AM_I = 0b01000000,
  };

  enum config_t : uint8_t
  {
    // CFG_REG_A
    COMP_TEMP_EN = 1 << 7,
    REBOOT = 1 << 6,
    SOFT_RST = 1 << 5,
    LP = 1 << 4,
    ODR_10HZ = 0b00 << 2,
    ODR_20HZ = 0b01 << 2,
    ODR_50HZ = 0b10 << 2,
    ODR_100HZ = 0b11 << 2,
    MD_CONTINUOUS = 0b00 << 0,
    MD_SINGLE = 0b01 << 0,
    MD_IDLE = 0b11 << 0,

    // CFG_REG_B
    OFF_CANC_ONE_SHOT = 1 << 4,
    INT_ON_DATA_OFF = 1 << 3,
    SET_FREQ = 1 << 2,
    OFF_CANC = 1 << 1,
    LPF = 1 << 0,

    // CFG_REG_C
    INT_ON_PIN = 1 << 6,
    I2C_DIS = 1 << 5,
    BDU = 1 << 4,
    BLE = 1 << 3,
    SELF_TEST = 1 << 1,
    DRDY_ON_PIN = 1 << 0,
  };

  linux::I2Cdev i2c_;

  bool readRegs(const uint8_t& addr, const size_t& bytes);
  bool writeReg(const uint8_t& addr, const uint8_t& data);

  bool checkWhoAmI();
  bool configure();
};
}  // namespace aso
