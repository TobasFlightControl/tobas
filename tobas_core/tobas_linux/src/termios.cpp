#include <termios.h>  // Cannot be included with <asm-generic/termios.h>

#include "../include/tobas_linux/termios.hpp"

namespace linux
{
int tcsendbreak(int fd, int duration)
{
  return ::tcsendbreak(fd, duration);
}

int tcdrain(int fd)
{
  return ::tcdrain(fd);
}

int tcflush(int fd, int queue_selector)
{
  return ::tcflush(fd, queue_selector);
}

int tcflow(int fd, int action)
{
  return ::tcflow(fd, action);
}
}  // namespace linux
