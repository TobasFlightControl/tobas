#include <unistd.h>

#include "../include/dh_std_tools/unix.hpp"

namespace dh_std
{
bool isSuperUser()
{
  return getuid() == 0;
}
}  // namespace dh_std
