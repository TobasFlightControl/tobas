// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <termios.h>  // <asm/termios.h> does not work.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
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

  bool drainOutput();

  bool setBaudRate(uint32_t _baud_rate);
  bool setDataBits(uint8_t _data_bits);
  bool setSingleStopBit();
  bool setDoubleStopBit();
  bool enableParity(ParityMode _mode);
  bool disableParity();
  bool enableHungupClose();
  bool disableHungupClose();
  bool setTimeout(cc_t _msec_100);

  /** Set the minimum number of characters that `receive()` waits for. */
  bool setMinimumChars(uint8_t _num);

  bool send(const uint8_t* _data, size_t _length);
  bool sendAll(const uint8_t* _data, size_t _length, std::chrono::milliseconds _timeout);
  bool receive(uint8_t* _data, size_t _length);

  /** Receive 1 byte. */
  std::optional<uint8_t> receiveByte();
  /** Receive 1 byte without blocking. Distinguish no data from read errors. */
  std::expected<std::optional<uint8_t>, int> tryReceiveByte();

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
