#pragma once

#include <tobas_linux/i2c_dev.hpp>

namespace driver
{
/**
 * @brief A linux driver of 3-axis magnetometer bmm150.
 *
 * Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf
 */
class BMM150
{
  static constexpr char kI2cDevice[] = "/dev/i2c-1";
  static constexpr uint8_t kI2cAddress = 0x10;  // ref: p.36

public:
  explicit BMM150();

  bool initialize();

  /* Read the current magnetic field [μT]. axis orientation ref: p.46*/
  bool readMag(double& mx, double& my, double& mz);

private:
  /* 7: Register mapping (p.22) */
  enum register_t : uint8_t
  {
    // Who I am ID
    WHO_AM_I_REG = 0x40,

    // Output registers
    OUTX_LSB_REG = 0x42,
    OUTX_MSB_REG = 0x43,
    OUTY_LSB_REG = 0x44,
    OUTY_MSB_REG = 0x45,
    OUTZ_LSB_REG = 0x46,
    OUTZ_MSB_REG = 0x47,

    // hall resistance registers
    HALL_LSB_REG = 0x48,
    HALL_MSB_REG = 0x49,

    // interrrupt status registers
    INTERRUPT_REG = 0x4A,

    // configuration setting registers
    CFG_REG_A = 0x4B,
    CFG_REG_B = 0x4C,
    CFG_REG_C = 0x4D,
    CFG_REG_D = 0x4E,
    CFG_REG_E = 0x51,
    CFG_REG_F = 0x52,

    // trim registers
    DIG_X1_REG = 0x5D,
    DIG_Y1_REG = 0x5E,
    DIG_Z4_LSB_REG = 0x62,
    DIG_Z4_MSB_REG = 0x63,
    DIGX2_REG = 0x64,
    DIGY2_REG = 0x65,
    DIG_Z2_LSB_REG = 0x68,
    DIG_Z2_MSB_REG = 0x69,
  };

  enum chip_id_t : uint8_t
  {
    CHIP_ID = 0x32,
  };

  enum self_test_t : uint8_t
  {
    SELF_TEST_X = 1,
    SELF_TEST_Y = 1,
    SELF_TEST_Z = 1,
  };

  enum config_t : uint8_t
  {
    // CFG_REG_A
    SOFT_RST_A = 1 << 7,
    SPI3EN = 1 << 2,
    SOFT_RST_B = 1 << 1,
    PWR_ON = 1 << 0,  // 0 means suspend

    // CFG_REG_B
    ADV_SELF_TEST_NORMAL = 0b00 << 6,
    ADV_SELF_TEST_NEGATIVE = 0b10 << 6,
    ADV_SELF_TEST_POSITIVE = 0b11 << 6,
    ODR_2HZ = 0b001 << 3,
    ODR_6HZ = 0b010 << 3,
    ODR_8HZ = 0b011 << 3,
    ODR_10HZ = 0b000 << 3,
    ODR_15HZ = 0b100 << 3,
    ODR_20HZ = 0b101 << 3,
    ODR_25HZ = 0b110 << 3,
    ODR_30HZ = 0b111 << 3,
    OP_NORMAL = 0b00 << 1,
    OP_FORCED = 0b01 << 1,
    OP_SLEEP = 0b11 << 1,
    SELF_TEST = 1 << 0,

    // CFG_REG_C
    DATA_OVERRUN_EN = 1 << 7,
    OVERFLOW_INT_EN = 1 << 6,
    HIGH_INT_Z = 1 << 5,  // 1 means disable
    HIGH_INT_Y = 1 << 4,
    HIGH_INT_X = 1 << 3,
    LOW_INT_X = 1 << 2,
    LOW_INT_Y = 1 << 1,
    LOW_INT_Z = 1 << 0,

    // CFG_REG_D
    //

    // ref: p.13 recommended settings for Rep. XYZ, p.30 nXY = 1 + 2*REPXY
    // CFG_REG_E
    REPXY = 7,

    // ref: p.31 nZ = 1 + REPZ
    // CFG_REG_F
    REPZ = 26,
  };

  enum over_flow_t : int16_t
  {
    XYAXES_FLIP_OVERFLOW_ADCVAL = -4096,
    ZAXIS_HALL_OVERFLOW_ADCVAL = -16384,
    OVERFLOW_OUTPUT = -32768,
    NEGATIVE_SATURATION_Z = -32767,
    POSITIVE_SATURATION_Z = 32767,
  };

  struct TrimData
  {
    /*! trim x1 data */
    int8_t dig_x1;
    /*! trim y1 data */
    int8_t dig_y1;
    /*! trim x2 data */
    int8_t dig_x2;
    /*! trim y2 data */
    int8_t dig_y2;
    /*! trim z1 data */
    uint16_t dig_z1;
    /*! trim z2 data */
    int16_t dig_z2;
    /*! trim z3 data */
    int16_t dig_z3;
    /*! trim z4 data */
    int16_t dig_z4;
    /*! trim xy1 data */
    uint8_t dig_xy1;
    /*! trim xy2 data */
    int8_t dig_xy2;
    /*! trim xyz1 data */
    uint16_t dig_xyz1;
  } trim_data;

  linux::I2Cdev i2c_;

  bool readRegs(const uint8_t& addr, const size_t& bytes);
  bool writeReg(const uint8_t& addr, const uint8_t& data);

  bool checkWhoAmI();
  bool execSelfTest();
  bool configure();
  // ref: https://github.com/boschsensortec/BMM150_SensorAPI/blob/master/bmm150.c (not written in data sheet)
  bool readTrimRegisters();

  /**
   * @brief This internal API is used to obtain the compensated
   * magnetometer X axis data(micro-tesla) in int16_t.
   * 
   * @param mag_data_x magneto X axis raw data (int16_t)
   * @param data_r_hall hall sensor resistance raw data (uint16_t)
   * @return int16_t compensated magneto X axis data
   */
  int16_t compensateX(const int16_t& mag_data_x, const uint16_t& data_r_hall);
  /**
   * @brief This internal API is used to obtain the compensated
   * magnetometer Y axis data(micro-tesla) in int16_t.
   * 
   * @param mag_data_y magneto Y axis raw data (int16_t)
   * @param data_r_hall hall sensor resistance raw data (uint16_t)
   * @return int16_t compensated magneto Y axis data
   */
  int16_t compensateY(const int16_t& mag_data_y, const uint16_t& data_r_hall);
  /**
   * @brief This internal API is used to obtain the compensated
   * magnetometer Z axis data(micro-tesla) in int16_t.
   * 
   * @param mag_data_z magneto Z axis raw data (int16_t)
   * @param data_r_hall hall sensor resistance raw data (uint16_t)
   * @return int16_t compensated magneto Z axis data
   */
  int16_t compensateZ(const int16_t& mag_data_z, const uint16_t& data_r_hall);
};
}  // namespace driver
