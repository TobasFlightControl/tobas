// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <termios.h>  // <asm/termios.h> does not work.

#include <cstddef>
#include <cstdint>
#include <expected>
#include <map>

namespace tobas
{
namespace linux
{
/**
 * @brief UART driver.
 * cf. [pySerial](https://github.com/pyserial/pyserial/tree/7aeea35429d15f3eefed10bbb659674638903e3a)
 */
class UARTdev
{
public:
  enum class ReceiveError
  {
    kNoData,
    kDeviceError,
  };

  using ReceiveResult = std::expected<uint8_t, ReceiveError>;

  enum ParityMode : tcflag_t
  {
    kOdd,
    kEven,
  };

  explicit UARTdev();
  ~UARTdev();

  bool initialize(const char* uart_dev, bool block_mode = false) noexcept;

  bool setBaudRate(uint32_t baud_rate) noexcept;
  bool setDataBits(uint8_t data_bits);
  bool setSingleStopBit();
  bool setDoubleStopBit();
  bool enableParity(ParityMode mode);
  bool disableParity();
  bool enableHungupClose();
  bool disableHungupClose();
  bool setTimeout(cc_t msec_100);

  /* Set the minimum number of characters that receive() waits for. */
  bool setMinimumChars(uint8_t num);

  bool send(const uint8_t* data, size_t length);
  bool receive(uint8_t* data, size_t length);

  /**
   * @brief Receive one byte.
   *
   * If `_nonblock` is `true`, return `kNoData` without waiting when no byte is available.
   * Otherwise, wait until one byte is received or a device error occurs.
   * `kNoData` is temporary; `kDeviceError` indicates a communication failure.
   */
  ReceiveResult tryReceiveByte(bool _nonblock) noexcept;

  /**
   * @brief Attempt to send all specified bytes.
   *
   * Wait for the device to become writable as needed, even if its file descriptor is non-blocking.
   * A `true` result guarantees that all bytes were sent; a `false` result does not guarantee completion.
   */
  bool sendAll(const uint8_t* _data, size_t _length) noexcept;

  /* Receive 1 byte. */
  uint8_t receiveByte();

private:
  const std::map<uint32_t, uint32_t> baudrate_constants_;

  bool block_mode_ = false;
  int uart_fd_ = -1;
  struct termios options_;

  bool getConfig() noexcept;
  bool setConfig() noexcept;

  bool isStandardBaudRate(uint32_t baud_rate) noexcept;
  bool setStandardBaudRate(uint32_t baud_rate) noexcept;
  bool setNonStandardBaudRate(uint32_t baud_rate) noexcept;
};
}  // namespace linux
}  // namespace tobas
