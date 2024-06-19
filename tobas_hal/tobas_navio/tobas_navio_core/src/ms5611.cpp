#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/time.hpp>

#include "../include/tobas_navio_core/ms5611.hpp"
#include "../include/tobas_navio_core/i2c_dev.hpp"
#include "../include/tobas_navio_core/util.hpp"

namespace navio
{
MS5611::MS5611(uint8_t address) : dev_addr_(address)
{
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

void MS5611::update()
{
  // Update pressure
  refreshPressure();
  tobas_std::msleep(kWaitForRefresh);  // Wait for pressure data ready
  readPressure();

  // Update temperature
  refreshTemperature();
  tobas_std::msleep(kWaitForRefresh);  // Wait for temperature data ready
  readTemperature();

  // In order to compute pressure, both pressure and temperature are needed.
  computePressureAndTemperature();
}

void MS5611::refreshPressure(uint8_t OSR)
{
  I2Cdev::writeBytes(dev_addr_, OSR, 0, 0);
}

void MS5611::refreshTemperature(uint8_t OSR)
{
  I2Cdev::writeBytes(dev_addr_, OSR, 0, 0);
}

void MS5611::readPressure()
{
  uint8_t buffer[3];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_ADC, 3, buffer);
  d1_ = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
}

void MS5611::readTemperature()
{
  uint8_t buffer[3];
  I2Cdev::readBytes(dev_addr_, MS5611_RA_ADC, 3, buffer);
  d2_ = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
}

void MS5611::computePressureAndTemperature()
{
  const auto dT = d2_ - c5_ * (double)(1 << 8);
  auto temp = (2000 + ((dT * c6_) / (double)(1 << 23)));
  auto off = c2_ * (double)(1 << 16) + (c4_ * dT) / (double)(1 << 7);
  auto sens = c1_ * (double)(1 << 15) + (c3_ * dT) / (double)(1 << 8);

  if (temp < 2000)
  {
    const auto temp2 = tobas_std::sqr(dT) / (double)(1U << 31);  // NOTE: (1 << 31)は符号付きだとオーバーフロー
    auto off2 = 5 * tobas_std::sqr(temp - 2000) / 2;
    auto sens2 = off2 / 2;

    if (temp < -1500)
    {
      off2 += 7 * tobas_std::sqr(temp + 1500);
      sens2 += 11 * tobas_std::sqr(temp + 1500) / 2;
    }

    temp -= temp2;
    off -= off2;
    sens -= sens2;
  }

  // Final calculations
  pres_ = ((d1_ * sens) / (double)(1 << 21) - off) / (double)(1 << 15);
  temp_ = temp / 100;
}
}  // namespace navio
