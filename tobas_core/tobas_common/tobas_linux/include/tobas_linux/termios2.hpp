// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cinttypes>

namespace tobas
{
namespace linux
{
/**
 * @brief Set a non-standard baud rate.
 * cf. pySerial: https://github.com/pyserial/pyserial
 *
 * @note As in pySerial, both `termios` in `termios.h` and `termios2` in `asm/termibits.h` must be configured.
 */
bool setNonStandardBaudRate(int fd, uint32_t baud_rate);
}  // namespace linux
}  // namespace tobas
