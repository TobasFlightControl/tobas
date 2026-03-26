#pragma once

#include <array>
#include <cinttypes>
#include <iostream>
#include <vector>

namespace tobas
{
namespace st
{
template <typename T>
std::vector<uint8_t> toBytes(const T& src)
{
  std::vector<uint8_t> res;
  res.resize(sizeof(T));
  memcpy(res.data(), &src, sizeof(T));
  return res;
}

template <typename T>
bool fromBytes(const std::vector<uint8_t>& src, T& dst)
{
  if (src.size() != sizeof(T)) {
    std::cerr << "Size mismatch" << std::endl;
    return false;
  }

  memcpy(&dst, src.data(), sizeof(T));
  return true;
}

template <typename T, size_t N>
void fromBytes(const std::array<uint8_t, N>& src, T& dst)
{
  static_assert(src.size() == sizeof(T));
  memcpy(&dst, src.data(), sizeof(T));
}
}  // namespace st
}  // namespace tobas
