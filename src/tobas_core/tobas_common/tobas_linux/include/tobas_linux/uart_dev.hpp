// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <termios.h>  // <asm/termios.h> does not work.

#include <cstddef>
#include <cstdint>
#include <optional>

namespace tobas::linux
{
/**
 * UART driver.
 *
 * @see [pySerial](https://github.com/pyserial/pyserial/tree/7aeea35429d15f3eefed10bbb659674638903e3a)
 */
class UARTdev
{
public:
  enum ParityMode : tcflag_t
  {
    kOdd,
    kEven,
  };

  explicit UARTdev();
  ~UARTdev();

  bool initialize(const char* _uart_dev, bool _block_mode = false);

  bool setBaudRate(uint32_t _baud_rate);
  bool setDataBits(uint8_t _data_bits);
  bool setSingleStopBit();
  bool setDoubleStopBit();
  bool enableParity(ParityMode _mode);
  bool disableParity();
  bool enableHungupClose();
  bool disableHungupClose();

  /**
   * Set the minimum receive byte count and timeout together.
   *
   * @param _minimum_chars Minimum receive byte count (`VMIN`).
   * @param _msec_100 Timeout in units of 100ms (`VTIME`).
   */
  bool setTimeout(uint8_t _minimum_chars, cc_t _msec_100);

  bool send(const uint8_t* _data, size_t _length);
  bool receive(uint8_t* _data, size_t _length);

  /** Receive 1 byte. */
  std::optional<uint8_t> receiveByte();

private:
  bool block_mode_ = false;
  int uart_fd_ = -1;
  struct termios options_;

  bool getConfig();
  bool setConfig();

  bool isStandardBaudRate(uint32_t _baud_rate);
  bool setStandardBaudRate(uint32_t _baud_rate);
  bool setNonStandardBaudRate(uint32_t _baud_rate);
};
}  // namespace tobas::linux
