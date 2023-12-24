#pragma once

#include "I2Cdev.h"

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

class MS5611
{
public:
  /** MS5611 constructor.
   * @param address I2C address
   * @see MS5611_DEFAULT_ADDRESS
   */
  explicit MS5611(uint8_t address = MS5611_DEFAULT_ADDRESS);

  /** Power on and prepare for general usage.
   * This method reads coefficients stored in PROM.
   */
  void initialize();

  /** Verify the I2C connection.
   * @return True if connection is valid, false otherwise
   */
  bool testConnection();

  /** Initiate the process of pressure measurement
   * @param OSR value
   * @see MS5611_RA_D1_OSR_4096
   */
  void refreshPressure(uint8_t OSR = MS5611_RA_D1_OSR_4096);

  /** Read pressure value
   */
  void readPressure();

  /** Initiate the process of temperature measurement
   * @param OSR value
   * @see MS5611_RA_D2_OSR_4096
   */
  void refreshTemperature(uint8_t OSR = MS5611_RA_D2_OSR_4096);

  /** Read temperature value
   */
  void readTemperature();

  /** Calculate temperature and pressure calculations and perform compensation
   *  More info about these calculations is available in the datasheet.
   */
  void calculatePressureAndTemperature();

  /** Perform pressure and temperature reading and calculation at once.
   *  Contains sleeps, better perform operations separately.
   */
  void update();

  /** Get calculated temperature value
   @return Temperature in degrees of Celsius
   */
  float getTemperature();

  /** Get calculated pressure value
 @return Pressure in millibars
 */
  float getPressure();

private:
  uint8_t devAddr;                  // I2C device adress
  uint16_t C1, C2, C3, C4, C5, C6;  // Calibration data
  uint32_t D1, D2;                  // Raw measurement data
  float TEMP;                       // Calculated temperature
  float PRES;                       // Calculated pressure
};
