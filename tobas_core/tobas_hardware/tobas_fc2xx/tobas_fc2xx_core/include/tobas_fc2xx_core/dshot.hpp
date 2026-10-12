// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <expected>
#include <string>

#include <tobas_algorithm/crc.hpp>
#include <tobas_linux/spi_dev.hpp>

namespace tobas
{
namespace fc2xx
{
/**
 * SPI interface for DShot motor commands and ESC telemetry on FC2XX hardware.
 *
 * @see https://betaflight.com/docs/development/api/dshot
 */
class DShot
{
public:
  static constexpr size_t kChannelSize = 8;

  enum Command : uint16_t
  {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1 = 1,
    DSHOT_CMD_BEEP2 = 2,
    DSHOT_CMD_BEEP3 = 3,
    DSHOT_CMD_BEEP4 = 4,
    DSHOT_CMD_BEEP5 = 5,
    DSHOT_CMD_ESC_INFO = 6,
    DSHOT_CMD_SPIN_DIRECTION_1 = 7,
    DSHOT_CMD_SPIN_DIRECTION_2 = 8,
    DSHOT_CMD_3D_MODE_OFF = 9,
    DSHOT_CMD_3D_MODE_ON = 10,
    DSHOT_CMD_SETTINGS_REQUEST = 11,
    DSHOT_CMD_SAVE_SETTINGS = 12,
    DSHOT_EXTENDED_TELEMETRY_ENABLE = 13,
    DSHOT_EXTENDED_TELEMETRY_DISABLE = 14,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON = 22,
    DSHOT_CMD_LED1_ON = 23,
    DSHOT_CMD_LED2_ON = 24,
    DSHOT_CMD_LED3_ON = 25,
    DSHOT_CMD_LED0_OFF = 26,
    DSHOT_CMD_LED1_OFF = 27,
    DSHOT_CMD_LED2_OFF = 28,
    DSHOT_CMD_LED3_OFF = 29,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE = 33,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 34,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY = 35,
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY = 42,
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY = 43,
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY = 44,
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY = 45,
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY = 46,
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY = 47,
  };

  explicit DShot() noexcept;

  bool initialize() noexcept;
  std::expected<void, std::string> transfer() noexcept;

  /** Set the DShot throttle directory. */
  void setThrottle(size_t ch, uint16_t throttle) noexcept;
  /** Set the target motor rotating speed [rad/s]. */
  void setTargetSpeed(size_t ch, double rps) noexcept;
  /** Set the Kv value [rad/s/V]. */
  void setKv(size_t ch, double kv_si) noexcept;
  /** Set the internal resistance [Ω]. */
  void setInternalResistance(size_t ch, double resistance) noexcept;
  /** Set the propeller diameter [m]. */
  void setPropellerDiameter(size_t ch, double diameter) noexcept;
  /** Set the moment constant scaled by the propeller diameter [Nm/(rad/s)^2/m^5]. */
  void setMomentConstant(size_t ch, double moment_const) noexcept;
  /** Set the number of motor poles. */
  void setNumPoles(size_t ch, uint16_t num_poles) noexcept;
  /** Set the motor speed control gain (2 to the x-1 power). No feedback when 0 is specified. */
  void setRpmControlGain(size_t ch, uint8_t gain) noexcept;
  /** Set the no-operation command. */
  void setNoOperation(size_t ch) noexcept;

  /** Get the validity of the telemetry. */
  bool getValidity(size_t ch) noexcept;
  /** Get the current motor rotating speed [rad/s]. */
  double getSpeed(size_t ch) noexcept;
  /** Get the current ESC temperature [degC]. */
  double getTemperature(size_t ch) noexcept;
  /** Get the current ESC input voltage [V]. */
  double getVoltage(size_t ch) noexcept;
  /** Get the current ESC current [A]. */
  double getCurrent(size_t ch) noexcept;

  void printCurrentState(size_t ch) noexcept;
  void printCurrentStates() noexcept;

private:
  linux::SPIdev spi_;
  uint32_t tx_buf_[kChannelSize + 1] = {};
  uint32_t rx_buf_[kChannelSize + 1] = {};

  std::array<uint16_t, kChannelSize> half_num_poles_;

  algo::CRC32Left crc_;
};
}  // namespace fc2xx
}  // namespace tobas
