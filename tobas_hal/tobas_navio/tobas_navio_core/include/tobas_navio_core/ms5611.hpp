#pragma once

#include <tobas_linux/i2c_dev.hpp>

#define MS5611_ADDRESS_CSB_LOW 0x76
#define MS5611_ADDRESS_CSB_HIGH 0x77
#define MS5611_DEFAULT_ADDRESS MS5611_ADDRESS_CSB_HIGH

#define MS5611_RA_ADC 0x00
#define MS5611_RA_RESET 0x1E

#define MS5611_RA_C0 0xA0
#define MS5611_RA_C1 0xA2
#define MS5611_RA_C2 0xA4
#define MS5611_RA_C3 0xA6
#define MS5611_RA_C4 0xA8
#define MS5611_RA_C5 0xAA
#define MS5611_RA_C6 0xAC
#define MS5611_RA_C7 0xAE

#define MS5611_RA_D1_OSR_256 0x40
#define MS5611_RA_D1_OSR_512 0x42
#define MS5611_RA_D1_OSR_1024 0x44
#define MS5611_RA_D1_OSR_2048 0x46
#define MS5611_RA_D1_OSR_4096 0x48

#define MS5611_RA_D2_OSR_256 0x50
#define MS5611_RA_D2_OSR_512 0x52
#define MS5611_RA_D2_OSR_1024 0x54
#define MS5611_RA_D2_OSR_2048 0x56
#define MS5611_RA_D2_OSR_4096 0x58

namespace navio
{
class MS5611
{
  static constexpr size_t kWaitForRefresh = 8;  // [ms]

public:
  /**
   * @brief MS5611 constructor.
   *
   * @param i2c_addr I2C address
   * @see MS5611_DEFAULT_ADDRESS
   */
  explicit MS5611(uint8_t i2c_addr = MS5611_DEFAULT_ADDRESS);

  /**
   * @brief Power on and prepare for general usage. This method reads coefficients stored in PROM.
   */
  bool initialize();

  /**
   * @brief Perform pressure and temperature reading and calculation at once.
   * Contains sleeps, better perform operations separately.
   */
  bool update();

  /**
   * @brief Get calculated temperature value.
   *
   * @return Temperature in degrees of Celsius
   */
  inline double getTemperature() const;

  /**
   * @brief Get calculated pressure value.
   *
   * @return Pressure in Pascal.
   */
  inline double getPressure() const;

private:
  linux::I2Cdev i2c_dev_;                 // I2C device
  uint8_t buf_[3];                        // I2C buffer
  uint16_t c1_, c2_, c3_, c4_, c5_, c6_;  // Calibration data
  uint32_t d1_, d2_;                      // Raw measurement data
  double temp_;                           // Calculated temperature [Celcius]
  double pres_;                           // Calculated pressure [Pa]

  /**
   * @brief Initiate the process of pressure measurement.
   *
   * @param OSR value
   * @see MS5611_RA_D1_OSR_4096
   */
  bool refreshPressure(uint8_t OSR = MS5611_RA_D1_OSR_4096);

  /**
   * @brief Initiate the process of temperature measurement.
   *
   * @param OSR value
   * @see MS5611_RA_D2_OSR_4096
   */
  bool refreshTemperature(uint8_t OSR = MS5611_RA_D2_OSR_4096);

  /**
   * @brief Read pressure value
   */
  bool readPressure();

  /**
   * @brief Read temperature value.
   */
  bool readTemperature();

  /**
   * @brief Calculate temperature and pressure calculations and perform compensation.
   * More info about these calculations is available in the datasheet.
   */
  void computePressureAndTemperature();
};

inline double MS5611::getTemperature() const
{
  return temp_;
}

inline double MS5611::getPressure() const
{
  return pres_;
}
}  // namespace navio
