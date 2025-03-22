#pragma once

#include <typeinfo>
#include <cxxabi.h>

namespace tobas_std
{
template <typename T>
const char* getClassName()
{
  int status;
  const auto demangled_name = abi::__cxa_demangle(typeid(T).name(), 0, 0, &status);
  if (status == 0)
    return demangled_name;
  else
    return typeid(T).name();
}
}  // namespace tobas_std
