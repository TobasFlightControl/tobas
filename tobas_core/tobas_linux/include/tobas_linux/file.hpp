#pragma once

namespace linux
{
int writeFile(const char* path, const char* fmt, ...);
int readFile(const char* path, const char* fmt, ...);
}  // namespace linux
