// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <string>

#include <tobas_algorithm/crc.hpp>
#include <tobas_dshot/command.hpp>
#include <tobas_linux/spi_dev.hpp>

namespace tobas::t2
{
/**
 * SPI interface for DShot motor commands and ESC telemetry on Tobas T2 hardware.
 *
 * @see https://betaflight.com/docs/development/api/dshot
 */
class DShot
{
public:
  static constexpr size_t kChannelSize = 8;

  explicit DShot() noexcept;

  bool initialize(const char* _spi_device) noexcept;
  std::expected<void, std::string> transfer() noexcept;

  /** Set the DShot throttle directly. */
  void setThrottle(size_t _ch, uint16_t _throttle) noexcept;
  /** Set the target motor rotating speed [rad/s]. */
  void setTargetSpeed(size_t _ch, double _rps) noexcept;
  /** Set the Kv value [rad/s/V]. */
  void setKv(size_t _ch, double _kv_si) noexcept;
  /** Set the internal resistance [Ω]. */
  void setInternalResistance(size_t _ch, double _resistance) noexcept;
  /** Set the propeller diameter [m]. */
  void setPropellerDiameter(size_t _ch, double _diameter) noexcept;
  /** Set the moment constant scaled by the propeller diameter [Nm/(rad/s)^2/m^5]. */
  void setMomentConstant(size_t _ch, double _moment_const) noexcept;
  /** Set the number of motor poles. */
  void setNumPoles(size_t _ch, uint16_t _num_poles) noexcept;
  /** Set the motor speed control gain (2 to the x-1 power). No feedback when 0 is specified. */
  void setRpmControlGain(size_t _ch, uint8_t _gain) noexcept;
  /** Set the no-operation command. */
  void setNoOperation(size_t _ch) noexcept;

  /** Get the validity of the telemetry. */
  bool getValidity(size_t _ch) noexcept;
  /** Get the current motor rotating speed [rad/s]. */
  double getSpeed(size_t _ch) noexcept;
  /** Get the current ESC temperature [degC]. */
  double getTemperature(size_t _ch) noexcept;
  /** Get the current ESC input voltage [V]. */
  double getVoltage(size_t _ch) noexcept;
  /** Get the current ESC current [A]. */
  double getCurrent(size_t _ch) noexcept;

  void printCurrentState(size_t _ch) noexcept;
  void printCurrentStates() noexcept;

private:
  linux::SPIdev spi_;
  uint32_t tx_buf_[kChannelSize + 1] = {};
  uint32_t rx_buf_[kChannelSize + 1] = {};

  std::array<uint16_t, kChannelSize> half_num_poles_;

  algo::CRC32Left crc_;
};
}  // namespace tobas::t2
