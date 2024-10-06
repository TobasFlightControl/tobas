#pragma once

#include <cinttypes>

namespace linux
{
bool setNonStandardBaudRate(int fd, uint32_t baud_rate);
}
