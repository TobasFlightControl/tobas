#pragma once

#include <termios.h>

namespace tobas_keyboard_teleop
{
class KeyboardReader
{
public:
  explicit KeyboardReader();
  ~KeyboardReader();

  char readKey();

private:
  termios tempcopy_, changed_;
};
}  // namespace tobas_keyboard_teleop
