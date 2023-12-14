#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <stdexcept>

#include "../include/tobas_keyboard_teleop/keyboard_reader.hpp"
#include "../include/tobas_keyboard_teleop/constants.hpp"

using namespace std;

namespace tobas_keyboard_teleop
{
KeyboardReader::KeyboardReader()
{
  tcgetattr(kFileDescriptor, &tempcopy_);
  memcpy(&changed_, &tempcopy_, sizeof(termios));

  changed_.c_lflag &= ~(ICANON | ECHO);
  changed_.c_cc[VEOL] = 1;
  changed_.c_cc[VEOF] = 2;

  // 入力受付のタイムリミットを設定
  // https://stackoverflow.com/questions/2917881/how-to-implement-a-timeout-in-read-function-call
  changed_.c_cc[VMIN] = 0;
  changed_.c_cc[VTIME] = 0;  // タイムアウトを 0 x 10 = 0 [sec] に設定．つまり全く待たない．

  tcsetattr(kFileDescriptor, TCSANOW, &changed_);
}

KeyboardReader::~KeyboardReader()
{
  tcsetattr(kFileDescriptor, TCSANOW, &tempcopy_);
}

char KeyboardReader::readKey()
{
  char buf = 0;
  if (read(kFileDescriptor, &buf, 1) < 0)
    throw runtime_error("Failed to read keyboard input.");
  return buf;
}
}  // namespace tobas_keyboard_teleop
