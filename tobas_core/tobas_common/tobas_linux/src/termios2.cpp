// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/termios2.hpp"

#include <asm/termbits.h>
#include <sys/ioctl.h>

#include <chrono>
#include <iostream>
#include <thread>

#include <tobas_std_tools/error.hpp>

using namespace std::chrono_literals;

namespace tobas::linux
{
bool setNonStandardBaudRate(int _fd, uint32_t _baud_rate)
{
  struct termios2 buf;

  if (ioctl(_fd, TCGETS2, &buf) != 0) {
    std::cerr << "Failed to get termios2 struct (TCGETS2): " << st::strError() << '\n';
    return false;
  }

  buf.c_cflag &= ~CBAUD;
  buf.c_cflag |= CBAUDEX;
  buf.c_ispeed = buf.c_ospeed = _baud_rate;

  if (ioctl(_fd, TCSETS2, &buf) != 0) {
    std::cerr << "Failed to set termios2 struct (TCSETS2): " << st::strError() << '\n';
    return false;
  }

  std::this_thread::sleep_for(1ms);

  return true;
}
}  // namespace tobas::linux
