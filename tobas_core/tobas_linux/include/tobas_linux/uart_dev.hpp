#pragma once

#include <cstdint>
#include <cstddef>
#include <termios.h>  // <asm/termios.h>ではダメ

namespace linux
{
/**
 * @brief UARTドライバ．
 * cf. [pySerial](https://github.com/pyserial/pyserial/tree/7aeea35429d15f3eefed10bbb659674638903e3a)
 */
class UARTdev
{
public:
  enum parity_mode_t : tcflag_t
  {
    PARITY_ODD,
    PARITY_EVEN,
  };

  explicit UARTdev();
  ~UARTdev();

  bool initialize(const char* uart_dev, bool block_mode = false);

  bool setStandardBaudRate(uint32_t baud_rate_flag);
  bool setNonStandardBaudRate(uint32_t baud_rate);
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
  int uart_fd_ = -1;
  struct termios options_;

  bool getConfig();
  bool setConfig();
};
}  // namespace linux
