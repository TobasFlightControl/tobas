#include <iostream>
#include <sys/ioctl.h>
#include <asm/termbits.h>

#include "../include/tobas_linux/termios2.hpp"
#include "../include/tobas_linux/errer.hpp"

using namespace std;

namespace linux
{
bool setNonStandardBaudRate(int fd, uint32_t baud_rate)
{
  struct termios2 buf1, buf2;

  if (ioctl(fd, TCGETS2, buf1) != 0)
  {
    cerr << "Failed to get termios2 struct (TCGETS2): " << strError() << endl;
    return false;
  }

  buf1.c_cflag &= ~CBAUD;
  buf1.c_cflag |= CBAUDEX;
  buf1.c_ispeed = buf1.c_ospeed = baud_rate;

  if (ioctl(fd, TCSETS2, buf1) != 0)
  {
    cerr << "Failed to set termios2 struct (TCSETS2): " << strError() << endl;
    return false;
  }

  // 設定が反映されたかどうか確認
  if (ioctl(fd, TCGETS2, buf2) != 0)
  {
    cerr << "Failed to get termios2 struct (TCGETS2): " << strError() << endl;
    return false;
  }
  if (buf1.c_cflag != buf2.c_cflag || buf1.c_ispeed != buf2.c_ispeed || buf1.c_ospeed != buf2.c_ospeed)
  {
    cerr << "Configurations are not reflected." << endl;
    return false;
  }

  return true;
}
}  // namespace linux
