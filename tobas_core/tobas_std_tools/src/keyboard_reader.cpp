#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <stdexcept>

#include "../include/tobas_std_tools/keyboard_reader.hpp"

#define FILE_DESCRIPTOR 0  // 標準入力

using namespace std;

namespace tobas_std
{
KeyboardReader::KeyboardReader()
{
  tcgetattr(FILE_DESCRIPTOR, &tempcopy_);
  memcpy(&changed_, &tempcopy_, sizeof(termios));

  changed_.c_lflag &= ~(ICANON | ECHO);
  changed_.c_cc[VEOL] = 1;
  changed_.c_cc[VEOF] = 2;

  // 入力受付のタイムリミットを設定
  // https://stackoverflow.com/questions/2917881/how-to-implement-a-timeout-in-read-function-call
  changed_.c_cc[VMIN] = 0;
  changed_.c_cc[VTIME] = 0;  // タイムアウトを 0 x 10 = 0 [sec] に設定．つまり全く待たない．

  tcsetattr(FILE_DESCRIPTOR, TCSANOW, &changed_);
}

KeyboardReader::~KeyboardReader()
{
  tcsetattr(FILE_DESCRIPTOR, TCSANOW, &tempcopy_);
}

char KeyboardReader::readKey()
{
  char buf = 0;
  if (read(FILE_DESCRIPTOR, &buf, 1) < 0)
    return -1;
  return buf;
}
}  // namespace tobas_std
