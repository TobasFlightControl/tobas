#include "tobas_linux/error.hpp"

namespace linux
{
std::string strError(int error_number)
{
  return "[Errno " + std::to_string(error_number) + "] " + strerror(error_number);
}
}  // namespace linux
