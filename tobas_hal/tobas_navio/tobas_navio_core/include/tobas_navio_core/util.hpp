#pragma once

#include <cinttypes>

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])

namespace navio
{
template <typename T>
static constexpr T sqr(const T& x)
{
  return x * x;
}

int writeFile(const char* path, const char* fmt, ...);
int readFile(const char* path, const char* fmt, ...);
int getNavioVersion();
bool checkAPM();

/* Decode IEEE 754 single precision floating point number. */
float decodeBinary32(uint32_t bin);
}  // namespace navio
