#pragma once

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])

namespace navio
{
int writeFile(const char* path, const char* fmt, ...);
int readFile(const char* path, const char* fmt, ...);
int getNavioVersion();
bool checkAPM();
}  // namespace navio
