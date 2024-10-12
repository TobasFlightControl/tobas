#pragma once

#include <string>
#include <string.h>

namespace linux
{
/* Return "[Errno errno] strerror(errno)"" */
std::string strError(int error_number = errno);
}  // namespace linux
