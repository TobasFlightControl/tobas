// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cxxabi.h>

#include <typeinfo>

namespace tobas
{
namespace st
{
template <typename T>
const char* getClassName()
{
  int status;
  const auto demangled_name = abi::__cxa_demangle(typeid(T).name(), 0, 0, &status);
  if (status == 0) {
    return demangled_name;
  }
  else {
    return typeid(T).name();
  }
}
}  // namespace st
}  // namespace tobas
