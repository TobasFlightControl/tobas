#pragma once

#include <tobas_linux/i2c_dev.hpp>

namespace aso
{
/**
 * @brief A linux driver of full-scale barometer.
 *
 * Datasheet: https://www.st.com/resource/en/datasheet/ilps22qs.pdf
 */
class ILPS22QS
{
  static constexpr char kI2cDevice[] = "/dev/i2c-1";
  static constexpr uint8_t kI2cAddress = 0b1011100;
  static constexpr double kTempScale = 100;  // [LSB/degC]

public:
  explicit ILPS22QS();

  bool initialize();

  /* Read the current pressure [Pa]. */
  bool readPressure(double& pres);

  /* Read the current temperature [degC]. */
  bool readTemperature(double& temp);

private:
  /* 8: Register mapping (p.32) */
  enum register_t : uint8_t
  {
    // Interface control register
    IF_CTRL = 0x0E,

    // Who am I
    WHO_AM_I_REG = 0x0F,

    // Control registers
    CTRL_REG1 = 0x10,
    CTRL_REG2 = 0x11,
    CTRL_REG3 = 0x12,

    // Status register
    STATUS = 0x27,

    // Pressure output registers
    PRESSURE_OUT_XL = 0x28,
    PRESSURE_OUT_L = 0x29,
    PRESSURE_OUT_H = 0x2A,

    // Temperature output registers
    TEMP_OUT_L = 0x2B,
    TEMP_OUT_H = 0x2C,
  };

  enum who_am_i_t : uint8_t
  {
    WHO_AM_I = 0b10110100,
  };

  enum config_t : uint8_t
  {
    // CTRL_REG_1
    ODR_1HZ = 0b0001 << 3,
    ODR_4HZ = 0b0010 << 3,
    ODR_10HZ = 0b0011 << 3,
    ODR_25HZ = 0b0100 << 3,
    ODR_50HZ = 0b0101 << 3,
    ODR_75HZ = 0b0110 << 3,
    ODR_100HZ = 0b0111 << 3,
    ODR_200HZ = 0b1000 << 3,
    AVG_4 = 0b000 << 0,
    AVG_8 = 0b001 << 0,
    AVG_16 = 0b010 << 0,
    AVG_32 = 0b011 << 0,
    AVG_64 = 0b100 << 0,
    AVG_128 = 0b101 << 0,
    AVG_512 = 0b111 << 0,

    // CTRL_REG_2
    REBOOT_MEMORY_CONTENT = 1 << 7,
    FS_MODE_1260HPA = 0 << 6,
    FS_MODE_4060HPA = 1 << 6,
    LPF_CFG_4 = 0 << 5,
    LPF_CFG_9 = 1 << 5,
    ENABLE_LPF = 1 << 4,
    BLOCK_DATA_UPDATE = 1 << 3,
    SOFTWARE_RESET = 1 << 2,
    ONESHOT = 1 << 0,

    // CTRL_REG_3
    AH_QVAR_EN = 1 << 7,
    AH_QVAR_P_AUTO_EN = 1 << 5,
    IF_ADD_INC = 1 << 0,
  };

  enum status_t : uint8_t
  {
    TEMP_OVERRUN = 1 << 5,
    PRES_OVERRUN = 1 << 4,
    TEMP_AVAILABLE = 1 << 1,
    PRES_AVAILABLE = 1 << 0,
  };

  linux::I2Cdev i2c_;
  double pres_scale_;  // [LSB/Pa]

  bool readRegs(const uint8_t& addr, const size_t& bytes);
  bool writeReg(const uint8_t& addr, const uint8_t& data);

  bool checkWhoAmI();
  bool configure();

  void setPressureScale(const uint8_t& fs_mode);
};
}  // namespace aso
