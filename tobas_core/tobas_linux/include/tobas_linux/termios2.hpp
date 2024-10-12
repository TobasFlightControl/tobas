#pragma once

#include <cinttypes>

namespace linux
{
/**
 * @brief 標準ではないボーレートを設定する．
 * cf. pySerial: https://github.com/pyserial/pyserial
 *
 * @note pySerialのように，termios.h内のtermiosに加え，asm/termibits.h内のtermios2の両方を設定する必要がある．
 */
bool setNonStandardBaudRate(int fd, uint32_t baud_rate);
}  // namespace linux
