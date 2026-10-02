// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_std_tools/error.hpp"

#include <cstring>

namespace tobas
{
namespace st
{
std::string strError(int error_number)
{
  return "[Errno " + std::to_string(error_number) + "] " + std::strerror(error_number);
}
}  // namespace st
}  // namespace tobas
