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

  /** Wait for transmitted output using tcdrain(); no strict timeout is guaranteed. */
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

  /** Transmit all bytes; retry nonblocking writes for up to one second. */
  bool send(const uint8_t* _data, size_t _length);
  bool receive(uint8_t* _data, size_t _length);

  /** Receive 1 byte using the configured blocking mode. No data returns nullopt. */
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
