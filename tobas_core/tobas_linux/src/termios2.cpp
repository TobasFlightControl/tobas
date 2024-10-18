#include <iostream>
#include <thread>
#include <sys/ioctl.h>
#include <asm/termbits.h>

#include "../include/tobas_linux/termios2.hpp"
#include "../include/tobas_linux/errer.hpp"

using namespace std;

namespace linux
{
bool setNonStandardBaudRate(int fd, uint32_t baud_rate)
{
  struct termios2 buf;

  if (ioctl(fd, TCGETS2, buf) != 0)
  {
    cerr << "Failed to get termios2 struct (TCGETS2): " << strError() << endl;
    return false;
  }

  buf.c_cflag &= ~CBAUD;
  buf.c_cflag |= CBAUDEX;
  buf.c_ispeed = buf.c_ospeed = baud_rate;

  if (ioctl(fd, TCSETS2, buf) != 0)
  {
    cerr << "Failed to set termios2 struct (TCSETS2): " << strError() << endl;
    return false;
  }

  this_thread::sleep_for(1ms);

  return true;
}
}  // namespace linux
