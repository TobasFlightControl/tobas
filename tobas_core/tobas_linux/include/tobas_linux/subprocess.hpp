#pragma once

#include <vector>
#include <sys/types.h>

namespace linux
{
pid_t createSubprocess(const std::vector<char*>& command);
}
