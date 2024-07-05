#pragma once

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])

namespace navio
{
int getNavioVersion();
bool checkAPM();
}  // namespace navio
