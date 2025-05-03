#include <cstring>
#include <stdexcept>
#include <unistd.h>

#include "../include/tobas_keyboard/keyboard_reader.hpp"

#define STD_INPUT_FD 0  // 標準入力のファイルディスクリプタ

using namespace std;

namespace keyboard
{
KeyboardReader::KeyboardReader()
{
  tcgetattr(STD_INPUT_FD, &tempcopy_);
  memcpy(&changed_, &tempcopy_, sizeof(termios));

  changed_.c_lflag &= ~(ICANON | ECHO);
  changed_.c_cc[VEOL] = 1;
  changed_.c_cc[VEOF] = 2;

  // 入力受付のタイムリミットを設定
  // https://stackoverflow.com/questions/2917881/how-to-implement-a-timeout-in-read-function-call
  changed_.c_cc[VMIN] = 0;  // 最低文字数を0に設定．つまり入力がなくてもすぐ返す．
  changed_.c_cc[VTIME] = 0;

  tcsetattr(STD_INPUT_FD, TCSANOW, &changed_);
}

KeyboardReader::~KeyboardReader()
{
  tcsetattr(STD_INPUT_FD, TCSANOW, &tempcopy_);
}

signed char KeyboardReader::readKey()
{
  char buf = 0;
  if (read(STD_INPUT_FD, &buf, 1) < 0) {
    return -1;
  }
  return buf;
}
}  // namespace keyboard
