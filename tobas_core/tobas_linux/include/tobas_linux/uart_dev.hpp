#pragma once

#include <termios.h>  // XXX: <asm/termios.h>ではダメ

#include <cstdint>
#include <cstddef>
#include <map>

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

  bool setBaudRate(uint32_t baud_rate);
  bool setDataBits(uint8_t data_bits);
  bool setSingleStopBit();
  bool setDoubleStopBit();
  bool enableParity(parity_mode_t mode);
  bool disableParity();
  bool enableHungupClose();
  bool disableHungupClose();
  bool setTimeout(cc_t msec_100);

  /* Set the minimum number of characters which we wait for in receive(). */
  bool setMinimumChars(uint8_t num);

  bool send(const uint8_t* data, size_t length);
  bool receive(uint8_t* data, size_t length);

  /* Receive 1 byte. */
  uint8_t receiveByte();

private:
  const std::map<uint32_t, uint32_t> baudrate_constants_;

  bool block_mode_ = false;
  int uart_fd_ = -1;
  struct termios options_;

  bool getConfig();
  bool setConfig();

  bool isStandardBaudRate(uint32_t baud_rate);
  bool setStandardBaudRate(uint32_t baud_rate);
  bool setNonStandardBaudRate(uint32_t baud_rate);
};
}  // namespace linux
