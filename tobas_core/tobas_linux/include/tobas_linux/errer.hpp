#pragma once

#include <string.h>

#include <string>

namespace linux
{
/* Return "[Errno errno] strerror(errno)"" */
std::string strError(int error_number = errno);
}  // namespace linux
