#include <iostream>

#include <tobas_std_tools/unit_conversions.hpp>

#include "../include/tobas_aso_core/dshot.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;

namespace aso
{
DShot::DShot()
{
}

bool DShot::initialize()
{
  if (!spi_.initialize(spi_device::kDshotDev, kSpiClockFreq))
    return false;

  for (size_t ch = 0; ch < kChannelSize; ++ch)
    setThrottle(ch, DSHOT_CMD_MOTOR_STOP);

  return true;
}

bool DShot::transfer()
{
  if (!spi_.transfer(kSpiBufSize))
    return false;

  return true;
}

bool DShot::setThrottle(size_t ch, uint16_t throttle)
{
  if (!checkChannelSize(ch))
    return false;

  if (throttle >= (1 << 11))
  {
    cerr << "DSHOT thrrotle out of range." << endl;
    return false;
  }

  *(uint32_t*)(spi_.tx + ch * kChannelBytes) = (kSetThrottleCmd << 28) | throttle;
  return true;
}

bool DShot::setTargetSpeed(size_t ch, double rps)
{
  if (!checkChannelSize(ch))
    return false;

  if (rps < 0.)
  {
    cerr << "Target speed must be non-negative." << endl;
    return false;
  }

  const auto rpm = static_cast<uint32_t>(tobas_std::rps2rpm(rps));
  if (rpm >= (1 << 16))
  {
    cerr << "Target rotation speed is too large." << endl;
    return false;
  }

  *(uint32_t*)(spi_.tx + ch * kChannelBytes) = (kSetTargetRPMCmd << 28) | rpm;
  return true;
}

bool DShot::setKv(size_t ch, double kv_si)
{
  if (!checkChannelSize(ch))
    return false;

  if (kv_si <= 0.)
  {
    cerr << "Kv value must be positive." << endl;
    return false;
  }

  const auto kv = static_cast<uint32_t>(tobas_std::rps2rpm(kv_si));  // [rpm/V]
  if (kv == 0)
  {
    cerr << "Kv value is too small." << endl;
    return false;
  }
  if (kv >= (1 << 16))
  {
    cerr << "Kv value is too large." << endl;
    return false;
  }

  *(uint32_t*)(spi_.tx + ch * kChannelBytes) = (kSetKvCmd << 28) | kv;
  return true;
}

bool DShot::setInternalResistance(size_t ch, double resistance)
{
  if (!checkChannelSize(ch))
    return false;

  if (resistance <= 0.)
  {
    cerr << "Internal resistance must be positive." << endl;
    return false;
  }

  const auto resistance_mohm = static_cast<uint32_t>(resistance * 1e+3);
  if (resistance_mohm == 0)
  {
    cerr << "Internal resistance is too small." << endl;
    return false;
  }
  if (resistance_mohm >= (1 << 16))
  {
    cerr << "Internal resistance is too large." << endl;
    return false;
  }

  *(uint32_t*)(spi_.tx + ch * kChannelBytes) = (kSetResistanceCmd << 28) | resistance_mohm;
  return true;
}

bool DShot::setPropellerDiameter(size_t ch, double diameter)
{
  if (!checkChannelSize(ch))
    return false;

  if (diameter <= 0.)
  {
    cerr << "Propeller diameter must be positive." << endl;
    return false;
  }

  const auto diameter_mm = static_cast<uint32_t>(diameter * 1e+3);
  if (diameter_mm == 0)
  {
    cerr << "Propeller diameter is too small." << endl;
    return false;
  }
  if (diameter_mm >= (1 << 16))
  {
    cerr << "Propeller diameter is too large." << endl;
    return false;
  }

  *(uint32_t*)(spi_.tx + ch * kChannelBytes) = (kSetDiameterCmd << 28) | diameter_mm;
  return true;
}

bool DShot::setMomentConstant(size_t ch, double moment_const)
{
  if (!checkChannelSize(ch))
    return false;

  if (moment_const <= 0.)
  {
    cerr << "Moment constant must be positive." << endl;
    return false;
  }

  const auto moment_const_scaled = static_cast<uint32_t>(moment_const * 1e+9);
  if (moment_const_scaled == 0)
  {
    cerr << "Moment constant is too small." << endl;
    return false;
  }
  if (moment_const_scaled >= (1 << 16))
  {
    cerr << "Moment constant is too large." << endl;
    return false;
  }

  *(uint32_t*)(spi_.tx + ch * kChannelBytes) = (kSetMomentConstCmd << 28) | moment_const_scaled;
  return true;
}

bool DShot::setNumPoles(size_t ch, uint32_t num_poles)
{
  if (!checkChannelSize(ch))
    return false;

  if (num_poles == 0)
  {
    cerr << "Number of poles must be positive." << endl;
    return false;
  }

  if (num_poles % 2 != 0)
  {
    cerr << "Number of poles must be even." << endl;
    return false;
  }

  const auto half_num_poles = num_poles / 2;
  if (half_num_poles >= (1 << 16))
  {
    cerr << "Number of poles is too large." << endl;
    return false;
  }

  *(uint32_t*)(spi_.tx + ch * kChannelBytes) = (kSetHalfNumPolesCmd << 28) | half_num_poles;
  return true;
}

bool DShot::setSpeedControlGain(size_t ch, uint32_t gain)
{
  if (!checkChannelSize(ch))
    return false;

  if (gain >= (1 << 16))
  {
    cerr << "Speed control gain is too large." << endl;
    return false;
  }

  *(uint32_t*)(spi_.tx + ch * kChannelBytes) = (kSetGainCmd << 28) | gain;
  return true;
}

bool DShot::isValid(size_t ch)
{
  const auto rx = *(uint32_t*)(spi_.rx + ch * kChannelBytes);
  return (rx >> 31) & 1;
}

double DShot::getCurrentSpeed(size_t ch)
{
  const auto rx = *(uint32_t*)(spi_.rx + ch * kChannelBytes);
  const auto rpm = rx & 0xFFFF;
  return tobas_std::rpm2rps(rpm);
}

bool DShot::checkChannelSize(size_t ch)
{
  if (ch >= kChannelSize)
  {
    cerr << "DSHOT channel out of range." << endl;
    return false;
  }

  return true;
}
}  // namespace aso
