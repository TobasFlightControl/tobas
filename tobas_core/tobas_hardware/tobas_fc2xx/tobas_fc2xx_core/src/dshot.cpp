// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_fc2xx_core/dshot.hpp"

#include <cassert>
#include <format>
#include <iostream>

#include <tobas_linux/error.hpp>
#include <tobas_math/definitions.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace fc2xx
{
namespace
{
constexpr uint32_t kParameterMask = 0xFFFFFFF;
}  // namespace

DShot::DShot() noexcept : crc_(algo::CRC32Left::CRC_32)
{
  crc_.initialize();
}

bool DShot::initialize() noexcept
{
  // Initialize SPI communication.
  if (!spi_.initialize("/dev/spidev0.0", tx_buf_, rx_buf_, 12'000'000)) {
    return false;
  }

  // Initialize the number of motor pole pairs.
  half_num_poles_.fill(1);

  // Set no-operation commands for all channels.
  for (size_t ch = 0; ch < kChannelSize; ++ch) {
    setNoOperation(ch);
  }

  // Discard the first response.
  if (!spi_.transfer(sizeof(tx_buf_))) {
    return false;
  }

  return true;
}

std::expected<void, std::string> DShot::transfer() noexcept
{
  // Compute CRC.
  tx_buf_[kChannelSize] = crc_.compute((uint8_t*)tx_buf_, sizeof(uint32_t) * kChannelSize);

  // Transfer.
  if (!spi_.transfer(sizeof(tx_buf_))) {
    return std::unexpected("SPI transfer failed: " + linux::strError());
  }

  // Check CRC.
  const auto cs = rx_buf_[kChannelSize];
  const auto cr = crc_.compute((uint8_t*)rx_buf_, sizeof(uint32_t) * kChannelSize);
  if (cs != cr) {
    return std::unexpected(std::format("CRC mismatch: received 0x{:08X}, computed 0x{:08X}.", cs, cr));
  }

  return {};
}

void DShot::setThrottle(size_t ch, uint16_t throttle) noexcept
{
  assert(ch < kChannelSize);

  constexpr uint8_t kSetThrottleCmd = 0;
  tx_buf_[ch] = (kSetThrottleCmd << 28) | (throttle & kParameterMask);
}

void DShot::setTargetSpeed(size_t ch, double rps) noexcept
{
  assert(ch < kChannelSize);

  constexpr uint8_t kSetTargetRPMCmd = 1;
  const auto rpm = static_cast<uint32_t>(st::rps2rpm(std::max(rps, 0.0)));
  tx_buf_[ch] = (kSetTargetRPMCmd << 28) | (rpm & kParameterMask);
}

void DShot::setKv(size_t ch, double kv_si) noexcept
{
  assert(ch < kChannelSize);
  assert(kv_si > 0.0);

  constexpr uint8_t kSetKvCmd = 2;
  const auto kv = static_cast<uint32_t>(st::rps2rpm(kv_si));  // [rpm/V]
  tx_buf_[ch] = (kSetKvCmd << 28) | (kv & kParameterMask);
}

void DShot::setInternalResistance(size_t ch, double resistance) noexcept
{
  assert(ch < kChannelSize);
  assert(resistance > 0.0);

  constexpr uint8_t kSetResistanceCmd = 3;
  const auto resistance_mohm = static_cast<uint32_t>(resistance * 1e+3);
  tx_buf_[ch] = (kSetResistanceCmd << 28) | (resistance_mohm & kParameterMask);
}

void DShot::setPropellerDiameter(size_t ch, double diameter) noexcept
{
  assert(ch < kChannelSize);
  assert(diameter > 0.0);

  constexpr uint8_t kSetDiameterCmd = 4;
  const auto diameter_mm = static_cast<uint32_t>(diameter * 1e+3);
  tx_buf_[ch] = (kSetDiameterCmd << 28) | (diameter_mm & kParameterMask);
}

void DShot::setMomentConstant(size_t ch, double moment_const) noexcept
{
  assert(ch < kChannelSize);
  assert(moment_const > 0.0);

  constexpr uint8_t kSetMomentConstCmd = 5;
  const auto moment_const_scaled = static_cast<uint32_t>(moment_const * 1e+9);
  tx_buf_[ch] = (kSetMomentConstCmd << 28) | (moment_const_scaled & kParameterMask);
}

void DShot::setNumPoles(size_t ch, uint16_t num_poles) noexcept
{
  assert(ch < kChannelSize);
  assert(num_poles > 0);
  assert(num_poles % 2 == 0);

  constexpr uint8_t kSetHalfNumPolesCmd = 6;
  const auto half_num_poles = num_poles / 2;
  tx_buf_[ch] = (kSetHalfNumPolesCmd << 28) | (half_num_poles & kParameterMask);
  half_num_poles_.at(ch) = half_num_poles;
}

void DShot::setRpmControlGain(size_t ch, uint8_t gain) noexcept
{
  assert(ch < kChannelSize);

  constexpr uint8_t kSetGainCmd = 7;
  tx_buf_[ch] = (kSetGainCmd << 28) | (gain & kParameterMask);
}

void DShot::setNoOperation(size_t ch) noexcept
{
  assert(ch < kChannelSize);

  constexpr uint8_t kNoOperationCmd = UINT8_MAX;
  tx_buf_[ch] = (kNoOperationCmd << 28);
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
}  // namespace fc2xx
}  // namespace tobas
