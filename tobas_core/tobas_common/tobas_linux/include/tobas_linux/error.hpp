#pragma once

#include <string.h>

#include <string>

namespace tobas
{
namespace linux
{
/* Return "[Errno errno] strerror(errno)"" */
std::string strError(int error_number = errno);
}  // namespace linux
}  // namespace tobas
