#include <math.h>
#include <unistd.h>

#include "../include/tobas_navio_core/ms5611.hpp"

namespace navio
{
MS5611::MS5611(uint8_t address)
{
  dev_addr_ = address;
}

void MS5611::initialize()
{
  // Reading 6 calibration data values
  uint8_t buff[2];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_C1, 2, buff);
  c1_ = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_C2, 2, buff);
  c2_ = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_C3, 2, buff);
  c3_ = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_C4, 2, buff);
  c4_ = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_C5, 2, buff);
  c5_ = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_C6, 2, buff);
  c6_ = buff[0] << 8 | buff[1];

  update();
}

bool MS5611::testConnection()
{
  uint8_t data;
  int8_t status = I2Cdev::readByte(dev_addr_, MS5611_RA_C0, &data);
  return status > 0;
}

void MS5611::refreshPressure(uint8_t OSR)
{
  I2Cdev::writeBytes(dev_addr_, OSR, 0, 0);
}

void MS5611::readPressure()
{
  uint8_t buffer[3];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_ADC, 3, buffer);
  d1_ = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
}

void MS5611::refreshTemperature(uint8_t OSR)
{
  I2Cdev::writeBytes(dev_addr_, OSR, 0, 0);
}

void MS5611::readTemperature()
{
  uint8_t buffer[3];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_ADC, 3, buffer);
  d2_ = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
}

void MS5611::calculatePressureAndTemperature()
{
  float dT = d2_ - c5_ * pow(2, 8);
  temp_ = (2000 + ((dT * c6_) / pow(2, 23)));
  float OFF = c2_ * pow(2, 16) + (c4_ * dT) / pow(2, 7);
  float SENS = c1_ * pow(2, 15) + (c3_ * dT) / pow(2, 8);

  float T2, OFF2, SENS2;

  if (temp_ >= 2000)
  {
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;
  }
  if (temp_ < 2000)
  {
    T2 = dT * dT / pow(2, 31);
    OFF2 = 5 * pow(temp_ - 2000, 2) / 2;
    SENS2 = OFF2 / 2;
  }
  if (temp_ < -1500)
  {
    OFF2 = OFF2 + 7 * pow(temp_ + 1500, 2);
    SENS2 = SENS2 + 11 * pow(temp_ + 1500, 2) / 2;
  }

  temp_ = temp_ - T2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;

  // Final calculations
  pres_ = ((d1_ * SENS) / pow(2, 21) - OFF) / pow(2, 15) / 100;
  temp_ = temp_ / 100;
}

void MS5611::update()
{
  refreshPressure();
  usleep(10000);  // Waiting for pressure data ready
  readPressure();

  refreshTemperature();
  usleep(10000);  // Waiting for temperature data ready
  readTemperature();

  calculatePressureAndTemperature();
}
}  // namespace navio
