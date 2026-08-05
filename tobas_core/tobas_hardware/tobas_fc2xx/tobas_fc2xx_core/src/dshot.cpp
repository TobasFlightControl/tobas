// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_fc2xx_core/dshot.hpp"

#include <iostream>

#include <tobas_math/definitions.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace fc2xx
{
DShot::DShot() noexcept : crc_(algo::CRC32Left::CRC_32)
{
  crc_.initialize();
}

bool DShot::initialize() noexcept
{
  if (!spi_.initialize(kSpiDevice, tx_buf_, rx_buf_, kSpiClockFreq)) {
    return false;
  }

  half_num_poles_.fill(1);

  for (size_t ch = 0; ch < kChannelSize; ++ch) {
    if (!setThrottle(ch, DSHOT_CMD_MOTOR_STOP)) {
      return false;
    }
  }

  return true;
}

bool DShot::transfer() noexcept
{
  // Compute CRC.
  tx_buf_[kChannelSize] = crc_.compute((uint8_t*)tx_buf_, sizeof(uint32_t) * kChannelSize);

  // Transfer.
  if (!spi_.transfer(sizeof(tx_buf_))) {
    return false;
  }

  // Check CRC.
  const auto cs = rx_buf_[kChannelSize];
  const auto cr = crc_.compute((uint8_t*)rx_buf_, sizeof(uint32_t) * kChannelSize);
  if (cs != cr) {
    std::cerr << "CRC failed: " << std::hex << std::uppercase << cs << " != " << cr << std::dec << std::endl;
    return false;
  }

  return true;
}

bool DShot::setThrottle(size_t ch, uint16_t throttle) noexcept
{
  if (!checkChannelSize(ch)) {
    return false;
  }

  if (throttle >= (1 << 11)) {
    std::cerr << "DShot thrrotle out of range." << std::endl;
    return false;
  }

  tx_buf_[ch] = (kSetThrottleCmd << 28) | throttle;

  return true;
}

bool DShot::setTargetSpeed(size_t ch, double rps) noexcept
{
  if (!checkChannelSize(ch)) {
    return false;
  }

  if (rps < 0.0) {
    std::cerr << "Target speed must be non-negative." << std::endl;
    return false;
  }

  const auto rpm = static_cast<uint32_t>(st::rps2rpm(rps));
  if (rpm >= (1 << 16)) {
    std::cerr << "Target rotation speed is too large." << std::endl;
    return false;
  }

  tx_buf_[ch] = (kSetTargetRPMCmd << 28) | rpm;

  return true;
}

bool DShot::setKv(size_t ch, double kv_si) noexcept
{
  if (!checkChannelSize(ch)) {
    return false;
  }

  if (kv_si <= 0.0) {
    std::cerr << "Kv value must be positive." << std::endl;
    return false;
  }

  const auto kv = static_cast<uint32_t>(st::rps2rpm(kv_si));  // [rpm/V]
  if (kv == 0) {
    std::cerr << "Kv value is too small." << std::endl;
    return false;
  }
  if (kv >= (1 << 16)) {
    std::cerr << "Kv value is too large." << std::endl;
    return false;
  }

  tx_buf_[ch] = (kSetKvCmd << 28) | kv;

  return true;
}

bool DShot::setInternalResistance(size_t ch, double resistance) noexcept
{
  if (!checkChannelSize(ch)) {
    return false;
  }

  if (resistance <= 0.0) {
    std::cerr << "Internal resistance must be positive." << std::endl;
    return false;
  }

  const auto resistance_mohm = static_cast<uint32_t>(resistance * 1e+3);
  if (resistance_mohm == 0) {
    std::cerr << "Internal resistance is too small." << std::endl;
    return false;
  }
  if (resistance_mohm >= (1 << 16)) {
    std::cerr << "Internal resistance is too large." << std::endl;
    return false;
  }

  tx_buf_[ch] = (kSetResistanceCmd << 28) | resistance_mohm;

  return true;
}

bool DShot::setPropellerDiameter(size_t ch, double diameter) noexcept
{
  if (!checkChannelSize(ch)) {
    return false;
  }

  if (diameter <= 0.0) {
    std::cerr << "Propeller diameter must be positive." << std::endl;
    return false;
  }

  const auto diameter_mm = static_cast<uint32_t>(diameter * 1e+3);
  if (diameter_mm == 0) {
    std::cerr << "Propeller diameter is too small." << std::endl;
    return false;
  }
  if (diameter_mm >= (1 << 16)) {
    std::cerr << "Propeller diameter is too large." << std::endl;
    return false;
  }

  tx_buf_[ch] = (kSetDiameterCmd << 28) | diameter_mm;

  return true;
}

bool DShot::setMomentConstant(size_t ch, double moment_const) noexcept
{
  if (!checkChannelSize(ch)) {
    return false;
  }

  if (moment_const <= 0.0) {
    std::cerr << "Moment constant must be positive." << std::endl;
    return false;
  }

  const auto moment_const_scaled = static_cast<uint32_t>(moment_const * 1e+9);
  if (moment_const_scaled == 0) {
    std::cerr << "Moment constant is too small." << std::endl;
    return false;
  }
  if (moment_const_scaled >= (1 << 24)) {
    std::cerr << "Moment constant is too large." << std::endl;
    return false;
  }

  tx_buf_[ch] = (kSetMomentConstCmd << 28) | moment_const_scaled;

  return true;
}

bool DShot::setNumPoles(size_t ch, uint16_t num_poles) noexcept
{
  if (!checkChannelSize(ch)) {
    return false;
  }

  if (num_poles == 0) {
    std::cerr << "Number of poles must be positive." << std::endl;
    return false;
  }

  if (num_poles % 2 != 0) {
    std::cerr << "Number of poles must be even." << std::endl;
    return false;
  }

  const auto half_num_poles = num_poles / 2;
  if (half_num_poles >= (1 << 16)) {
    std::cerr << "Number of poles is too large." << std::endl;
    return false;
  }

  tx_buf_[ch] = (kSetHalfNumPolesCmd << 28) | half_num_poles;
  half_num_poles_.at(ch) = half_num_poles;

  return true;
}

bool DShot::setRpmControlGain(size_t ch, uint8_t gain) noexcept
{
  if (!checkChannelSize(ch)) {
    return false;
  }

  if (gain >= (1 << 8)) {
    std::cerr << "Speed control gain is too large." << std::endl;
    return false;
  }

  tx_buf_[ch] = (kSetGainCmd << 28) | gain;

  return true;
}

bool DShot::getValidity(size_t ch) noexcept
{
  return rx_buf_[ch] > 0;
}

double DShot::getSpeed(size_t ch) noexcept
{
  const auto erpm = (rx_buf_[ch] >> 0) & 0x0FFF;

  if (erpm == 0) {
    return NAN;
  }
  else if (erpm == 0x0FFF) {
    return 0.0;
  }

  const auto exp = erpm >> 9;
  const auto base = erpm & 0x01FF;
  const auto eperiod_us = (base << exp);

  const auto period_us = eperiod_us * half_num_poles_.at(ch);
  return (M_2PI * 1e+6) / static_cast<double>(period_us);
}

double DShot::getTemperature(size_t ch) noexcept
{
  const auto temperature = (rx_buf_[ch] >> 12) & 0x0F;
  return static_cast<double>(temperature << 4);
}

double DShot::getVoltage(size_t ch) noexcept
{
  const auto voltage = (rx_buf_[ch] >> 16) & 0xFF;
  return static_cast<double>(voltage) / 4;
}

double DShot::getCurrent(size_t ch) noexcept
{
  const auto current = (rx_buf_[ch] >> 24) & 0xFF;
  return static_cast<double>(current);
}

void DShot::printCurrentState(size_t ch) noexcept
{
  std::cout << "Channel " << ch << ":" << std::endl;
  std::cout << "\tValid             : " << std::boolalpha << getValidity(ch) << std::noboolalpha << std::endl;
  std::cout << "\tSpeed [rpm]       : " << st::rps2rpm(getSpeed(ch)) << std::endl;
  std::cout << "\tTemperature [degC]: " << getTemperature(ch) << std::endl;
  std::cout << "\tVoltage [V]       : " << getVoltage(ch) << std::endl;
  std::cout << "\tCurrent [A]       : " << getCurrent(ch) << std::endl;
}

void DShot::printCurrentStates() noexcept
{
  for (size_t ch = 0; ch < fc2xx::DShot::kChannelSize; ++ch) {
    printCurrentState(ch);
  }
}

bool DShot::checkChannelSize(size_t ch) noexcept
{
  if (ch >= kChannelSize) {
    std::cerr << "DShot channel out of range." << std::endl;
    return false;
  }

  return true;
}
}  // namespace fc2xx
}  // namespace tobas
