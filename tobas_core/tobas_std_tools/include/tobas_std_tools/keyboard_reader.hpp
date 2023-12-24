#pragma once

#include <termios.h>

namespace tobas_std
{
static constexpr char kKeyCode_W = 'w';
static constexpr char kKeyCode_S = 's';
static constexpr char kKeyCode_A = 'a';
static constexpr char kKeyCode_D = 'd';
static constexpr char kKeyCode_O = 'o';
static constexpr char kKeyCode_C = 'c';
static constexpr char kKeyCode_Up = 0x41;
static constexpr char kKeyCode_Down = 0x42;
static constexpr char kKeyCode_Right = 0x43;
static constexpr char kKeyCode_Left = 0x44;

class KeyboardReader
{
public:
  explicit KeyboardReader();
  ~KeyboardReader();

  char readKey();

private:
  termios tempcopy_, changed_;
};
}  // namespace tobas_std
