#include <tobas_math/core.hpp>
#include <tobas_std_tools/time.hpp>

#include "../include/tobas_navio_core/ms5611.hpp"
#include "../include/tobas_navio_core/util.hpp"
#include "../include/tobas_navio_core/constants.hpp"

namespace navio
{
MS5611::MS5611(uint8_t i2c_addr) : i2c_dev_(kRasPiI2CDev, i2c_addr)
{
}

bool MS5611::initialize()
{
  if (!i2c_dev_.initialize())
    return false;

  // Reading 6 calibration data values
  if (!i2c_dev_.readBytes(MS5611_RA_C1, 2, buf_))
    return false;
  c1_ = buf_[0] << 8 | buf_[1];
  if (!i2c_dev_.readBytes(MS5611_RA_C2, 2, buf_))
    return false;
  c2_ = buf_[0] << 8 | buf_[1];
  if (!i2c_dev_.readBytes(MS5611_RA_C3, 2, buf_))
    return false;
  c3_ = buf_[0] << 8 | buf_[1];
  if (!i2c_dev_.readBytes(MS5611_RA_C4, 2, buf_))
    return false;
  c4_ = buf_[0] << 8 | buf_[1];
  if (!i2c_dev_.readBytes(MS5611_RA_C5, 2, buf_))
    return false;
  c5_ = buf_[0] << 8 | buf_[1];
  if (!i2c_dev_.readBytes(MS5611_RA_C6, 2, buf_))
    return false;
  c6_ = buf_[0] << 8 | buf_[1];

  return true;
}

bool MS5611::update()
{
  // Update pressure
  if (!refreshPressure())
    return false;
  tobas_std::msleep(kWaitForRefresh);  // Wait for pressure data ready
  if (!readPressure())
    return false;

  // Update temperature
  if (!refreshTemperature())
    return false;
  tobas_std::msleep(kWaitForRefresh);  // Wait for temperature data ready
  if (!readTemperature())
    return false;

  // In order to compute pressure, both pressure and temperature are needed.
  computePressureAndTemperature();

  return true;
}

bool MS5611::refreshPressure(uint8_t OSR)
{
  return i2c_dev_.writeBytes(OSR, 0, 0);
}

bool MS5611::refreshTemperature(uint8_t OSR)
{
  return i2c_dev_.writeBytes(OSR, 0, 0);
}

bool MS5611::readPressure()
{
  if (!i2c_dev_.readBytes(MS5611_RA_ADC, 3, buf_))
    return false;

  d1_ = (buf_[0] << 16) | (buf_[1] << 8) | buf_[2];
  return true;
}

bool MS5611::readTemperature()
{
  if (!i2c_dev_.readBytes(MS5611_RA_ADC, 3, buf_))
    return false;

  d2_ = (buf_[0] << 16) | (buf_[1] << 8) | buf_[2];
  return true;
}

void MS5611::computePressureAndTemperature()
{
  const auto dT = d2_ - c5_ * (double)(1 << 8);
  auto temp = (2000 + ((dT * c6_) / (double)(1 << 23)));
  auto off = c2_ * (double)(1 << 16) + (c4_ * dT) / (double)(1 << 7);
  auto sens = c1_ * (double)(1 << 15) + (c3_ * dT) / (double)(1 << 8);

  if (temp < 2000)
  {
    const auto temp2 = math::sqr(dT) / (double)(1U << 31);  // NOTE: (1 << 31)は符号付きだとオーバーフロー
    auto off2 = 5 * math::sqr(temp - 2000) / 2;
    auto sens2 = off2 / 2;

    if (temp < -1500)
    {
      off2 += 7 * math::sqr(temp + 1500);
      sens2 += 11 * math::sqr(temp + 1500) / 2;
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
