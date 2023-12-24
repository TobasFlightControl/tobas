#include <unistd.h>

#include "../include/tobas_std_tools/unix.hpp"

namespace tobas_std
{
bool isSuperUser()
{
  return getuid() == 0;
}
}  // namespace tobas_std
