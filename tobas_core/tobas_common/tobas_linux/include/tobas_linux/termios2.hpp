// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cinttypes>

namespace tobas::linux
{
/**
 * Set a non-standard baud rate.
 *
 * @note As in pySerial, both `termios` in `termios.h` and `termios2` in `asm/termibits.h` must be configured.
 * @see pySerial: https://github.com/pyserial/pyserial
 */
bool setNonStandardBaudRate(int _fd, uint32_t _baud_rate);
}  // namespace tobas::linux
