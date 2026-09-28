// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/termios2.hpp"

#include <asm/termbits.h>
#include <sys/ioctl.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "tobas_linux/error.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace linux
{
bool setNonStandardBaudRate(int fd, uint32_t baud_rate)
{
  struct termios2 buf;

  if (ioctl(fd, TCGETS2, &buf) != 0) {
    std::cerr << "Failed to get termios2 struct (TCGETS2): " << strError() << std::endl;
    return false;
  }

  buf.c_cflag &= ~CBAUD;
  buf.c_cflag |= CBAUDEX;
  buf.c_ispeed = buf.c_ospeed = baud_rate;

  if (ioctl(fd, TCSETS2, &buf) != 0) {
    std::cerr << "Failed to set termios2 struct (TCSETS2): " << strError() << std::endl;
    return false;
  }

  std::this_thread::sleep_for(1ms);

  return true;
}
}  // namespace linux
}  // namespace tobas
