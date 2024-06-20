#pragma once

#include <termios.h>

namespace keyboard
{
enum keycode_t : char
{
  ENTER = 0x0A,
  ESC = 0x1B,
  UP = 0x41,
  DOWN = 0x42,
  RIGHT = 0x43,
  LEFT = 0x44,
};

class KeyboardReader
{
public:
  explicit KeyboardReader();
  ~KeyboardReader();

  signed char readKey();

private:
  termios tempcopy_, changed_;
};
}  // namespace keyboard
