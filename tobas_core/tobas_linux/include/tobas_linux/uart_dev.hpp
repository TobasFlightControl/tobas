#pragma once

#include <cinttypes>
#include <cstddef>
#include <asm/termbits.h>

namespace linux
{
class UARTdev
{
  static constexpr uint32_t kDefaultBaudRate = 9600;  // [bps]

public:
  enum parity_mode_t : tcflag_t
  {
    PARITY_ODD,
    PARITY_EVEN,
  };

  explicit UARTdev();
  ~UARTdev();

  bool initialize(const char* uart_dev, bool block_mode = false);

  bool setBaudRate(uint32_t baud_rate);
  bool setDataBits(uint8_t data_bits);
  bool setSingleStopBit();
  bool setDoubleStopBit();
  bool enableParity(parity_mode_t mode);
  bool disableParity();
  bool enableHungupClose();
  bool disableHungupClose();

  /* Set the minimum number of characters which we wait for in receive(). */
  bool setMinimumChars(uint8_t num);

  bool send(const uint8_t* data, size_t length);
  bool receive(uint8_t* data, size_t length);

private:
  bool block_mode_ = false;
  int uart_fd_;
  struct termios2 options_;

  bool getConfig();
  bool setConfig();
};
}  // namespace linux
