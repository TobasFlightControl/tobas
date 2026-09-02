// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_keyboard/keyboard_reader.hpp"

#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace tobas
{
namespace keyboard
{
namespace
{
constexpr int kStandardInputFd = 0;
}  // namespace

KeyboardReader::KeyboardReader()
{
  tcgetattr(kStandardInputFd, &tempcopy_);
  std::memcpy(&changed_, &tempcopy_, sizeof(termios));

  changed_.c_lflag &= ~(ICANON | ECHO);
  changed_.c_cc[VEOL] = 1;
  changed_.c_cc[VEOF] = 2;

  // Set the input reception time limit.
  // https://stackoverflow.com/questions/2917881/how-to-implement-a-timeout-in-read-function-call
  changed_.c_cc[VMIN] = 0;  // Set the minimum character count to 0, so this returns immediately without input.
  changed_.c_cc[VTIME] = 0;

  tcsetattr(kStandardInputFd, TCSANOW, &changed_);
}

KeyboardReader::~KeyboardReader()
{
  tcsetattr(kStandardInputFd, TCSANOW, &tempcopy_);
}

signed char KeyboardReader::readKey()
{
  char buf = 0;
  if (read(kStandardInputFd, &buf, 1) < 0) {
    return -1;
  }
  return buf;
}
}  // namespace keyboard
}  // namespace tobas
