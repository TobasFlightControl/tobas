#pragma once

#include <sstream>
#include <string>

namespace tobas_std
{
template <typename T>
inline void addToStream(std::stringstream& ss, const T& t)
{
  ss << t;
}

template <typename T, typename... Args>
inline void addToStream(std::stringstream& ss, const T& t, const Args&... args)
{
  ss << t;
  addToStream(ss, args...);
}

template <typename... Args>
inline std::string buildString(const Args&... args)
{
  std::stringstream ss;
  addToStream(ss, args...);
  return ss.str();
}
}  // namespace tobas_std
